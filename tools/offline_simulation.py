#!/usr/bin/env python3
"""ROS/FC-free coarse controller regression, not a flight dynamics/SITL validation."""
import json
import math
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"ros2_ws/src/we_meet_vio"))
from we_meet_vio.geometry import Pose,quaternion,yaw_rotation
from we_meet_vio.mission import Mission,Inputs,Observation


def sample(now,position=None,velocity=None,heading=0.,mode="OFFBOARD",armed=True):
    position=np.array([0.,0.,.32] if position is None else position,dtype=float)
    velocity=np.zeros(3) if velocity is None else np.array(velocity,dtype=float)
    pose=Pose(now,position,quaternion(yaw_rotation(heading)),velocity,np.zeros(3),np.eye(6)*.0025,np.eye(6)*.0025)
    return Inputs(pose=pose,fc_pose=pose,height=position[2],height_stamp=now,
                  update_stamp=now,image_stamp=now,detector_stamp=now,ready=True,reason="",
                  connected=True,armed=armed,landed=position[2]<=.33,mode=mode,
                  state_stamp=now,extended_state_stamp=now)


def simulate(wind=.025,heading=.45,max_s=110):
    mission=Mission()
    now=100.
    dt=.025
    position=np.array([0.,0.,.32])
    velocity=np.zeros(3)
    mode="POSCTL"
    armed=False
    s=sample(now,position,velocity,heading,mode,armed)
    ok,message=mission.start(now,s)
    if not ok:
        raise RuntimeError(message)
    states=["PRESTREAM"]
    panel=mission.goal.copy()
    panel[2]=0
    observations=None
    hits=0
    max_command_step=0.
    previous=np.zeros(3)
    hold_first=None
    hold_elapsed=None
    for tick in range(int(max_s/dt)):
        now+=dt
        s=sample(now,position.copy(),velocity.copy(),heading,mode,armed)
        body_error=yaw_rotation(heading).T@(panel-position)
        # Synthetic downward camera: right=-body-left, down=-body-forward.
        ex=-body_error[1]*960/(max(.2,position[2])*640)
        ey=-body_error[0]*960/(max(.2,position[2])*480)
        if tick%5==0 and position[2]>2.7 and abs(ex)<.45 and abs(ey)<.45:
            hits+=1
            observations=Observation(now,1,panel.copy(),ex,ey,hits)
        s.observation=observations
        out=mission.step(now,s)
        max_command_step=max(max_command_step,float(np.linalg.norm(out.velocity[:2]-previous[:2])))
        previous=out.velocity.copy()
        if out.state!=states[-1]:
            states.append(out.state)
        if out.state=="HOLD" and hold_first is None:
            hold_first=now
        if out.completed_hold and hold_elapsed is None:
            hold_elapsed=now-hold_first
        if out.request_mode:
            mode=out.request_mode
        if out.request_arm:
            armed=True
        if mode=="AUTO.LAND":
            velocity=np.array([0.,0.,-.35])
            if position[2]<=.32:
                armed=False
                velocity[:]=0
        elif armed:
            disturbance=np.array([wind,-wind*.3,0.]) if position[2]>.5 else np.zeros(3)
            velocity+=(out.velocity+disturbance-velocity)*(dt/.18)
            heading+=out.yaw_rate*dt
        else:
            velocity[:]=0
        position+=velocity*dt
        position[2]=max(.32,position[2])
        if out.state in ("DONE","FAILED"):
            return {"success":mission.success,"reason":out.reason,"states":states,
                    "elapsed_s":round(now-100,3),"hold_elapsed_s":None if hold_elapsed is None else round(hold_elapsed,3),
                    "final_xy_error_m":round(float(np.linalg.norm(position[:2]-panel[:2])),3),
                    "max_horizontal_command_step_mps":round(max_command_step,6)}
    return {"success":False,"reason":"simulation limit","states":states}


if __name__=="__main__":
    result=simulate()
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result["success"] else 1)
