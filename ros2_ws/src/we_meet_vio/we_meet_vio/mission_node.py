import json
import time
from datetime import datetime,timezone
from pathlib import Path
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped,TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import String
from std_srvs.srv import Trigger
from mavros_msgs.msg import State,ExtendedState,EstimatorStatus
from mavros_msgs.srv import SetMode,CommandBool
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType
from .geometry import Pose,PoseBuffer,corrected_height,panel_intersection
from .health import StreamGuard,ClockGuard
from .mission import Mission,Parameters,Inputs,Observation
from .work_map import WorkMap
from .perception import optical_ray
from .logging import Recorder
from .ros_common import (setup,run,SENSOR,RELIABLE,stamp,set_stamp,vector,quaternion,
                         write_vector,odom_pose)


class Flight(Node):
    def __init__(self):
        super().__init__("vio_flight")
        cfg,_=setup(self)
        self.cfg=cfg
        self.ns=cfg["mavros_namespace"].rstrip("/")
        self.mission=Mission(Parameters(**cfg["flight"]))
        self.inputs=Inputs()
        self.buffer=PoseBuffer()
        self.map=WorkMap()
        self.ground_z=None
        self.texture_ok=False
        self.range_guard=StreamGuard(20,.2,.1)
        self.detector_guard=StreamGuard(5,.3,.22)
        self.clock=ClockGuard()
        self.fc_v=None
        self.fc_v_stamp=0.
        self.bridge={}
        self.bridge_received=0.
        self.estimator=None
        self.estimator_stamp=0.
        self.params_ok=False
        self.params_stamp=0.
        self.param_reason="FC parameter cache unread"
        self.owner_ok=False
        self.owner_reason=""
        self.command_frame_ok=False
        self.command_frame_stamp=0.
        self.requests={}
        self.last_requests={}
        self.status_pub=self.create_publisher(String,"/vio_flight/status",RELIABLE)
        self.map_pub=self.create_publisher(String,"/work_map",RELIABLE)
        self.command_pub=None if self.dry else self.create_publisher(TwistStamped,self.ns+"/setpoint_velocity/cmd_vel",RELIABLE)
        self.mode_client=None if self.dry else self.create_client(SetMode,self.ns+"/set_mode")
        self.arm_client=None if self.dry else self.create_client(CommandBool,self.ns+"/cmd/arming")
        self.param_client=self.create_client(GetParameters,self.ns+"/param/get_parameters")
        self.frame_client=self.create_client(GetParameters,self.ns+"/setpoint_velocity/get_parameters")
        self.create_subscription(Odometry,"/vio/body_odometry",self.pose,SENSOR)
        self.create_subscription(Odometry,self.ns+"/local_position/odom",self.fc_odometry,SENSOR)
        self.create_subscription(Range,"/lidar/range",self.lidar,SENSOR)
        self.create_subscription(String,"/vio/status",self.bridge_status,RELIABLE)
        self.create_subscription(String,"/panels/detections",self.panels,RELIABLE)
        self.create_subscription(State,self.ns+"/state",self.state,SENSOR)
        self.create_subscription(ExtendedState,self.ns+"/extended_state",self.extended,SENSOR)
        self.create_subscription(EstimatorStatus,self.ns+"/estimator_status",self.estimator_status,SENSOR)
        self.create_service(Trigger,"/vio_flight/start",self.start)
        self.create_service(Trigger,"/vio_flight/abort",self.abort)
        self.create_timer(.5,self.check_environment)
        self.create_timer(.025,self.tick)
        self.create_timer(1.,self.save_map)
        suffix=datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")+"-"+str(time.time_ns())
        self.recorder=Recorder(Path(cfg["log_directory"]).expanduser()/suffix,cfg)
        self.last_state="IDLE"
        self.last_map_completed=False
        self.get_logger().info("Loaded VIO mission; IDLE, dry_run="+str(self.dry))

    def now_s(self):
        return self.get_clock().now().nanoseconds*1e-9

    def pose(self,msg):
        try:
            if msg.header.frame_id != "map" or msg.child_frame_id != "base_link":
                raise ValueError("body odometry frame mismatch")
            p=odom_pose(msg)
            self.buffer.add(p)
            self.inputs.pose=p
        except ValueError as e:
            self.mission.abort(self.now_s(),str(e))

    def fc_velocity(self,msg):
        if msg.header.frame_id != self.cfg["fc_world_frame"]:
            self.fc_v=None
            return
        self.fc_v=vector(msg.twist.linear)
        self.fc_v_stamp=stamp(msg)

    def fc_odometry(self,msg):
        # MAVROS 2.14 velocity_local has ENU values but labels base_link.
        # Use REP147 odom (body twist) and rotate ONCE into its map parent.
        try:
            if msg.header.frame_id!=self.cfg["fc_world_frame"] or msg.child_frame_id!="base_link":
                raise ValueError("FC odometry frame mismatch")
            self.inputs.fc_pose=odom_pose(msg)
            self.fc_v_stamp=stamp(msg)
        except ValueError:
            self.fc_v_stamp=0.

    def fc_pose(self,msg):
        if msg.header.frame_id != self.cfg["fc_world_frame"] or self.fc_v is None:
            return
        q=quaternion(msg.pose.orientation)
        self.inputs.fc_pose=Pose(stamp(msg),vector(msg.pose.position),q,self.fc_v.copy(),
                                 np.zeros(3),np.eye(6),np.eye(6))

    def state(self,msg):
        self.inputs.connected=msg.connected
        self.inputs.armed=msg.armed
        self.inputs.mode=msg.mode
        self.inputs.state_stamp=self.now_s()

    def extended(self,msg):
        self.inputs.landed=msg.landed_state==ExtendedState.LANDED_STATE_ON_GROUND
        self.inputs.extended_state_stamp=self.now_s()

    def estimator_status(self,msg):
        self.estimator=msg
        self.estimator_stamp=self.now_s()

    def bridge_status(self,msg):
        try:
            self.bridge=json.loads(msg.data)
            self.bridge_received=self.now_s()
            self.inputs.update_stamp=float(self.bridge["update_stamp"])
            self.inputs.image_stamp=float(self.bridge["image_stamp"])
        except (ValueError,KeyError,TypeError):
            self.bridge={}

    def lidar(self,msg):
        now=self.now_s()
        if self.inputs.pose is None or msg.header.frame_id!="lidar_down" or not msg.min_range<=msg.range<=msg.max_range:
            return
        if not self.range_guard.feed(stamp(msg),now):
            return
        # Range is uncorrected slant measurement. Correct exactly once with body attitude.
        try:
            pose=self.buffer.at(stamp(msg)) or self.inputs.pose
            if abs(pose.stamp-stamp(msg)) > .08:
                return
            height=corrected_height(msg.range,pose.r,self.cal["lidar_position_body"])
            self.inputs.height=height
            self.inputs.height_stamp=stamp(msg)
        except ValueError:
            return

    def panels(self,msg):
        now=self.now_s()
        try:
            packet=json.loads(msg.data)
            source=float(packet["stamp"])
            if packet["schema"]!=1 or packet["frame_id"]!="camera_optical" or [packet["width"],packet["height"]]!=self.cal["resolution"]:
                raise ValueError("detection contract mismatch")
            if not self.detector_guard.feed(source,now):
                return
            self.inputs.detector_stamp=source
            self.texture_ok=(packet.get("texture_points",0)>=self.cfg["detector"]["texture_min_points"] and
                             packet.get("texture_cells",0)>=self.cfg["detector"]["texture_min_cells"])
            pose=self.buffer.at(source+self.cal["timeshift_cam_imu"])
            if pose is None or now-self.inputs.height_stamp > .2:
                return
            ground=(self.ground_z if self.ground_z is not None else pose.p[2]-self.inputs.height)
            plane_z=ground+self.cfg["panel_height_m"]
            candidates=[]
            w,h=self.cal["resolution"]
            for item in packet["panels"][:8]:
                u,v=map(float,item["center"])
                score=float(item["score"])
                corners=np.asarray(item["corners"],dtype=float)
                if (not np.isfinite([u,v,score]).all() or not 0<u<w or not 0<v<h or
                    score<.75 or corners.shape!=(4,2) or not np.isfinite(corners).all()):
                    continue
                xyz=panel_intersection(optical_ray([u,v],self.cal),pose,self.cal["T_body_camera"],plane_z)
                candidates.append((xyz,(u-w/2)/w,(v-h/2)/h,score))
            results=self.map.observe(source,candidates)
            if self.map.selected is not None:
                choices=[x for x in results if x[0].id==self.map.selected]
            else:
                goal=self.mission.goal if self.mission.goal is not None else pose.p
                choices=sorted(results,key=lambda x:np.linalg.norm(x[4][:2]-goal[:2]))
            if choices:
                panel,ex,ey,_,xyz=choices[0]
                self.inputs.observation=Observation(source,panel.id,xyz.copy(),ex,ey,panel.hits)
            # No detections leave the last observation's ORIGINAL stamp intact.
        except (ValueError,KeyError,TypeError) as e:
            self.get_logger().warning("rejected detector packet: "+str(e),throttle_duration_sec=2.)

    def check_environment(self):
        now=self.now_s()
        current=self.requests.get("frame")
        if (not current or current[0].done() or now-current[1]>1.5) and self.frame_client.service_is_ready():
            if current and not current[0].done():
                current[0].cancel()
            request=GetParameters.Request()
            request.names=["mav_frame"]
            future=self.frame_client.call_async(request)
            self.requests["frame"]=(future,now)
            future.add_done_callback(self.checked_frame)
        topics=("setpoint_velocity/cmd_vel","setpoint_velocity/cmd_vel_unstamped",
                "setpoint_position/local","setpoint_raw/local","setpoint_raw/attitude",
                "setpoint_attitude/attitude","setpoint_accel/accel")
        others=[]
        for name in topics:
            count=self.count_publishers(self.ns+"/"+name)
            permitted=1 if name=="setpoint_velocity/cmd_vel" and self.command_pub else 0
            if count>permitted:
                others.append(name)
        self.owner_ok=not others
        self.owner_reason="other setpoint publisher: "+",".join(others) if others else ""
        future_info=self.requests.get("params")
        if future_info and not future_info[0].done():
            if now-future_info[1] > 1.5:
                future_info[0].cancel()
                self.params_ok=False
            return
        if now-self.last_requests.get("params",0) < 2 or not self.param_client.service_is_ready():
            return
        req=GetParameters.Request()
        req.names=list(self.cfg["expected_fc_parameters"])
        future=self.param_client.call_async(req)
        self.requests["params"]=(future,now)
        self.last_requests["params"]=now
        future.add_done_callback(self.checked_parameters)

    def checked_frame(self,future):
        try:
            values=future.result().values
            self.command_frame_ok=(len(values)==1 and values[0].type==ParameterType.PARAMETER_STRING and values[0].string_value=="LOCAL_NED")
            self.command_frame_stamp=self.now_s()
        except (AttributeError,TypeError,RuntimeError):
            self.command_frame_ok=False

    def checked_parameters(self,future):
        try:
            response=future.result()
            actual={}
            for name,value in zip(self.cfg["expected_fc_parameters"],response.values):
                if value.type==ParameterType.PARAMETER_INTEGER:
                    actual[name]=value.integer_value
                elif value.type==ParameterType.PARAMETER_DOUBLE:
                    actual[name]=value.double_value
            errors=[name for name,expected in self.cfg["expected_fc_parameters"].items()
                    if name not in actual or abs(actual[name]-expected) > 1e-5]
            self.params_ok=not errors
            self.param_reason="FC parameter mismatch/absent: "+",".join(errors) if errors else ""
            self.params_stamp=self.now_s()
        except (AttributeError,TypeError,RuntimeError):
            self.params_ok=False
            self.param_reason="FC parameter query failed"

    def readiness(self,now):
        issues=[]
        if not self.clock.check(now,time.monotonic()):
            issues.append("ROS clock jump")
        if not self.bridge.get("ready",False) or now-self.bridge_received > .2:
            issues.append("VIO bridge: "+self.bridge.get("reason","missing"))
        if self.bridge.get("dry_run",True) and not self.dry:
            issues.append("FC vision bridge disabled")
        if not self.range_guard.ready(now):
            issues.append("LiDAR rate/age/gap")
        if not self.detector_guard.ready(now):
            issues.append("detector rate/age/gap")
        if not self.texture_ok:
            issues.append("camera texture insufficient for this VIO profile")
        if not self.params_ok or now-self.params_stamp > 3.5:
            issues.append(self.param_reason or "FC parameter cache stale")
        if not self.owner_ok:
            issues.append(self.owner_reason)
        if not self.command_frame_ok or now-self.command_frame_stamp>1.5:
            issues.append("MAVROS setpoint_velocity mav_frame must be LOCAL_NED")
        e=self.estimator
        if (e is None or now-self.estimator_stamp > 1 or not e.attitude_status_flag or
            not e.velocity_horiz_status_flag or not e.velocity_vert_status_flag or
            not e.pos_horiz_rel_status_flag or not e.pos_vert_abs_status_flag or
            e.const_pos_mode_status_flag or e.accel_error_status_flag):
            issues.append("FC estimator not valid")
        if not 0 <= now-self.fc_v_stamp <= .3:
            issues.append("FC velocity stale")
        verified=self.cfg["verification"]
        if not verified.get("evidence"):
            issues.append("verification evidence not recorded")
        for name in ("fc_fusion_verified","rc_recovery_verified","flight_settings_verified"):
            if verified.get(name) is not True:
                issues.append(name+" is false")
        if self.recorder.failed:
            issues.append(self.recorder.failed)
        self.inputs.ready=not issues
        self.inputs.reason="; ".join(issues)

    def start(self,request,response):
        self.readiness(self.now_s())
        if self.dry:
            response.success=False
            response.message="dry_run: read-only mission; use offline tests for full state sequence"
            return response
        if self.mode_client is None or not self.mode_client.service_is_ready() or not self.arm_client.service_is_ready():
            response.success=False
            response.message="FC mode/arming service absent"
            return response
        success,reason=self.mission.start(self.now_s(),self.inputs)
        if success:
            self.map=WorkMap()
            self.inputs.observation=None
            self.ground_z=self.mission.ground_z
        response.success,response.message=success,reason
        return response

    def abort(self,request,response):
        self.mission.abort(self.now_s())
        response.success=True
        response.message="abort received"
        return response

    def send_request(self,name,client,request):
        now=self.now_s()
        current=self.requests.get(name)
        if current and not current[0].done():
            if now-current[1] > 1.5:
                current[0].cancel()
            else:
                return
        if now-self.last_requests.get(name,0) < 1 or not client.service_is_ready():
            return
        self.last_requests[name]=now
        future=client.call_async(request)
        self.requests[name]=(future,now)
        # Requests are retries; only observed State/ExtendedState confirm actions.

    def tick(self):
        now=self.now_s()
        self.readiness(now)
        s=self.inputs
        # Keep raw map panel coords; create a temporary body-centre goal that places
        # the calibrated optical centre over that same panel at current attitude.
        raw=s.observation
        if raw is not None and s.pose is not None and self.ground_z is not None:
            try:
                w,h=self.cal["resolution"]
                centre=panel_intersection(optical_ray([w/2,h/2],self.cal),s.pose,self.cal["T_body_camera"],self.ground_z+self.cfg["panel_height_m"])
                s.observation=Observation(raw.stamp,raw.panel_id,raw.xyz-(centre-s.pose.p),raw.ex,raw.ey,raw.hits)
            except ValueError:
                s.observation=None
        out=self.mission.step(now,s)
        s.observation=raw
        if self.mission.target is not None and self.map.selected is None:
            self.map.select(self.mission.target)
        if out.completed_hold and not self.last_map_completed:
            self.map.complete()
            self.last_map_completed=True
        if self.command_pub and out.publish:
            msg=TwistStamped()
            set_stamp(msg.header,now)
            msg.header.frame_id="map"
            write_vector(msg.twist.linear,out.velocity)
            msg.twist.angular.z=out.yaw_rate
            self.command_pub.publish(msg)
        if not self.dry and out.request_mode:
            request=SetMode.Request()
            request.custom_mode=out.request_mode
            self.send_request("mode",self.mode_client,request)
        if not self.dry and out.request_arm:
            request=CommandBool.Request()
            request.value=True
            self.send_request("arm",self.arm_client,request)
        if out.state!=self.last_state:
            self.get_logger().info(out.state+": "+out.reason)
            self.last_state=out.state
        data={"stamp":now,"state":out.state,"reason":out.reason,"ready":s.ready,
              "readiness":s.reason,"command":out.velocity.tolist(),"yaw_rate":out.yaw_rate,
              "publish":out.publish,"dry_run":self.dry,"mode":s.mode,"armed":s.armed,
              "height":s.height,"height_stamp":s.height_stamp,"target":self.mission.target,
              "hold_completed":out.completed_hold}
        if s.pose:
            data.update(vio_position=s.pose.p.tolist(),vio_velocity=s.pose.v.tolist(),vio_stamp=s.pose.stamp,
                        pose_variance=np.diag(s.pose.pose_cov).tolist(),velocity_variance=np.diag(s.pose.twist_cov).tolist())
        if s.fc_pose:
            data.update(fc_position=s.fc_pose.p.tolist(),fc_velocity=s.fc_pose.v.tolist())
        if raw:
            data.update(panel_stamp=raw.stamp,panel_error=[raw.ex,raw.ey],panel_xyz=raw.xyz.tolist())
        self.recorder.record(data)
        if int(now*40)%8==0:
            msg=String()
            msg.data=json.dumps(data)
            self.status_pub.publish(msg)

    def save_map(self):
        msg=String()
        msg.data=json.dumps(self.map.dump())
        self.map_pub.publish(msg)
        # Enqueue rather than performing disk writes in control executor.
        self.recorder.record({"stamp":self.now_s(),"work_map":self.map.dump()})

    def destroy_node(self):
        self.recorder.close()
        return super().destroy_node()


def main():
    run(Flight)
