"""Optional camera publisher. Does not open or command a flight controller."""
import threading
import time
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraNode(Node):
    def __init__(self):
        super().__init__('trial_camera_source')
        for name, value in {'backend': 'picamera2', 'device': 0, 'width': 640,
                            'height': 480, 'fps': 15.0, 'rotation_degrees': 180,
                            'image_topic': '/camera/image_raw'}.items():
            self.declare_parameter(name, value)
        self.rotation = self.get_parameter('rotation_degrees').value
        if self.rotation not in (0, 90, 180, 270):
            raise ValueError('rotation_degrees must be 0/90/180/270')
        self.backend = self.get_parameter('backend').value
        w, h = self.get_parameter('width').value, self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        self.publisher = self.create_publisher(Image, self.get_parameter('image_topic').value, qos_profile_sensor_data)
        if self.backend == 'picamera2':
            from picamera2 import Picamera2
            self.camera = Picamera2(self.get_parameter('device').value)
            config = self.camera.create_video_configuration(main={'size': (w, h), 'format': 'YUV420'},
                                                            controls={'FrameRate': fps}, buffer_count=2)
            self.camera.configure(config)
            self.camera.start()
        elif self.backend == 'v4l2':
            self.camera = cv2.VideoCapture(self.get_parameter('device').value, cv2.CAP_V4L2)
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            self.camera.set(cv2.CAP_PROP_FPS, fps)
            self.camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not self.camera.isOpened():
                raise RuntimeError('V4L2 camera could not open')
        else:
            raise ValueError('backend must be picamera2 or v4l2')
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self.capture, daemon=True)
        self.thread.start()

    def capture(self):
        while not self.stop_event.is_set():
            try:
                source_time = time.monotonic()
                if self.backend == 'picamera2':
                    request = self.camera.capture_request()
                    try:
                        frame = cv2.cvtColor(request.make_array('main'), cv2.COLOR_YUV2BGR_I420)
                        source_time = request.get_metadata()['SensorTimestamp'] * 1e-9
                    finally:
                        request.release()
                else:
                    ok, frame = self.camera.read()
                    if not ok:
                        raise RuntimeError('camera read failed')
                rotations = {90: cv2.ROTATE_90_CLOCKWISE, 180: cv2.ROTATE_180, 270: cv2.ROTATE_90_COUNTERCLOCKWISE}
                if self.rotation:
                    frame = cv2.rotate(frame, rotations[self.rotation])
                age = time.monotonic()-source_time
                if not 0 <= age <= 0.3:
                    continue
                msg = Image()
                timestamp = self.get_clock().now().nanoseconds-int(age*1e9)
                msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(timestamp, 1_000_000_000)
                msg.header.frame_id = 'camera_optical_frame'
                msg.height, msg.width = frame.shape[:2]
                msg.encoding, msg.is_bigendian, msg.step = 'bgr8', 0, msg.width*3
                msg.data = frame.tobytes()
                self.publisher.publish(msg)
            except Exception as exc:
                self.get_logger().error(f'camera stopped: {exc}')
                self.stop_event.set()  # Missing fresh frames triggers the mission's camera fault.

    def destroy_node(self):
        self.stop_event.set()
        if self.backend == 'picamera2':
            self.camera.stop()
        else:
            self.camera.release()
        self.thread.join(timeout=1)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
