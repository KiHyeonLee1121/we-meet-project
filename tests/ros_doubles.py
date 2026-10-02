"""Isolated message/Node doubles for offline tests; never connect to ROS/FC."""
import importlib.util
from pathlib import Path
import sys
import types

ROOT = Path(__file__).resolve().parents[1]
for p in ('ros2_ws/src/da_daka_control', 'ros2_ws/src/we_meet_prototype141', 'laptop_ai'):
    sys.path.insert(0, str(ROOT / p))


class Message:
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)

    def __getattr__(self, name):
        value = 0.0 if name in {'x', 'y', 'z', 'w'} else Message()
        setattr(self, name, value)
        return value


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeNode:
    overrides = {}

    def __init__(self, name):
        self.parameters = {}
        self.subscriptions = {}
        self.publishers = {}

    def declare_parameter(self, name, value):
        self.parameters[name] = Message(value=self.overrides.get(name, value))

    def get_parameter(self, name):
        return self.parameters[name]

    def list_parameters(self, prefixes, depth):
        return Message(names=list(self.parameters))

    def create_publisher(self, kind, topic, qos):
        publisher = Publisher()
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, kind, topic, callback, qos):
        self.subscriptions[topic] = callback

    def create_service(self, *args):
        return Message()

    def create_client(self, *args):
        return Message(service_is_ready=lambda: False)

    def create_timer(self, *args):
        return Message()

    def count_publishers(self, topic):
        return 1

    def get_logger(self):
        return Message(info=lambda *a: None, warning=lambda *a: None,
                       error=lambda *a: None, debug=lambda *a: None)

    def get_clock(self):
        return Message(now=lambda: Message(to_msg=lambda: Message(sec=0, nanosec=0)))

    def destroy_node(self):
        return True


def install_ros_doubles():
    def module(name, **attrs):
        value = types.ModuleType(name)
        value.__dict__.update(attrs)
        sys.modules[name] = value
        return value

    module('rclpy')
    module('rclpy.node', Node=FakeNode)
    module('rclpy.executors', ExternalShutdownException=RuntimeError)
    module('rclpy.qos', QoSProfile=Message, qos_profile_sensor_data=Message(),
           DurabilityPolicy=Message(TRANSIENT_LOCAL=1),
           ReliabilityPolicy=Message(RELIABLE=1))
    for package, names in {
        'geometry_msgs': ('PoseStamped', 'TwistStamped'),
        'mavros_msgs': ('Altitude', 'EstimatorStatus', 'ExtendedState', 'State',
                        'SysStatus', 'HomePosition'),
        'sensor_msgs': ('BatteryState', 'Range', 'NavSatFix'),
        'std_msgs': ('Bool', 'Float32', 'Int32', 'String'),
        'da_daka_interfaces': ('PanelMap', 'PanelTarget', 'PerceptionResult', 'PanelDetection'),
    }.items():
        module(package)
        module(package+'.msg', **{name: Message for name in names})
    for package, names in {'mavros_msgs': ('CommandBool', 'SetMode'),
                           'std_srvs': ('SetBool', 'Trigger')}.items():
        if package not in sys.modules:
            module(package)
        module(package+'.srv', **{name: type(name, (), {'Request': Message, 'Response': Message}) for name in names})


install_ros_doubles()
