"""Evaluate both launch files with isolated launch objects and actual YAML.

This checks package/executable and parameter wiring, not a ROS launch runtime.
"""
import ast
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch
import yaml

ROOT = Path(__file__).resolve().parents[1]


class Configuration:
    def __init__(self, name):
        self.name = name


class Argument:
    def __init__(self, name, default_value, **kwargs):
        self.name, self.default = name, default_value


class NodeSpec:
    def __init__(self, **kwargs):
        self.spec = kwargs


class Description:
    def __init__(self, actions):
        self.actions = actions


class Include:
    def __init__(self, source, launch_arguments):
        self.arguments = dict(launch_arguments)


class Value:
    def __init__(self, value, value_type):
        self.value, self.type = value, value_type


def evaluate(value, context):
    if isinstance(value, Configuration):
        return context[value.name]
    if isinstance(value, Value):
        raw = evaluate(value.value, context)
        return str(raw).lower() == 'true' if value.type is bool else value.type(raw)
    if isinstance(value, dict):
        return {key: evaluate(v, context) for key, v in value.items()}
    return value


def build_launch(overrides=None):
    fake = {}
    exports = {
        'ament_index_python.packages': {'get_package_share_directory': lambda name: str(ROOT / 'ros2_ws/src' / name)},
        'launch': {'LaunchDescription': Description},
        'launch.actions': {'DeclareLaunchArgument': Argument, 'IncludeLaunchDescription': Include},
        'launch.launch_description_sources': {'PythonLaunchDescriptionSource': lambda filename: filename},
        'launch.substitutions': {'LaunchConfiguration': Configuration},
        'launch_ros.actions': {'Node': NodeSpec},
        'launch_ros.parameter_descriptions': {'ParameterValue': Value},
    }
    for name in ('ament_index_python', 'launch_ros'):
        fake[name] = types.ModuleType(name)
    for name, entries in exports.items():
        module = types.ModuleType(name)
        module.__dict__.update(entries)
        fake[name] = module
    def load(filename):
        spec = importlib.util.spec_from_file_location(filename, ROOT / 'ros2_ws/src/we_meet_prototype141/launch' / filename)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module.generate_launch_description().actions
    with patch.dict(sys.modules, fake):
        wrapper = load('flight141.launch.py')
        context = {a.name: a.default for a in wrapper if isinstance(a, Argument)}
        context.update(overrides or {})
        include = next(a for a in wrapper if isinstance(a, Include))
        context.update(evaluate(include.arguments, context))
        stack = load('_stack141.launch.py')
        for argument in stack:
            if isinstance(argument, Argument):
                context.setdefault(argument.name, argument.default)
        nodes = []
        for action in stack:
            if not isinstance(action, NodeSpec):
                continue
            node = dict(action.spec)
            name = evaluate(node['name'], context)
            parameters = {}
            for source in node['parameters']:
                source = evaluate(source, context)
                if isinstance(source, dict):
                    parameters.update(source)
                else:
                    document = yaml.safe_load(Path(source).read_text())
                    parameters.update(document.get('/**', {}).get('ros__parameters', {}))
                    parameters.update(document.get(name, {}).get('ros__parameters', {}))
            node.update(name=name, parameters=parameters)
            nodes.append(node)
        return nodes


class Launch141Tests(unittest.TestCase):
    def test_live_launch_loads_141_profile_under_renamed_mission(self):
        nodes = build_launch({'lidar_port': '/fake/tf-luna'})
        mission = next(n for n in nodes if n['package'] == 'we_meet_prototype141')
        p = mission['parameters']
        self.assertEqual(mission['executable'], 'flight141_mission')
        self.assertEqual(mission['name'], 'panel_localization_test')
        self.assertEqual(p['historical_panel_order_ids'], [3, 1, 2, 6])
        self.assertTrue(p['historical_home_follow_enabled'])
        self.assertTrue(p['localization_scan_yaw_aligned'])
        self.assertEqual(p['localization_scan_forward_offset_m'], 0.)
        self.assertEqual(p['cruise_speed_mps'], .50)
        self.assertEqual(p['survey_timeout_s'], 65.)
        self.assertFalse(p['configuration_approved'])
        self.assertFalse(p['calibration_approved'])
        self.assertFalse(p['require_live_spray'])
        self.assertEqual(next(n for n in nodes if n['name'] == 'tf_luna_serial')['parameters']['port'], '/fake/tf-luna')
        vertical = next(n for n in nodes if n['name'] == 'distance_controller')['parameters']
        self.assertEqual(vertical['command_topic'], '/distance_control/cmd_vel_internal')
        self.assertEqual(vertical['lidar_takeoff_target_distance_m'], 3.)
        self.assertEqual(vertical['target_distance_m'], 1.)
        self.assertEqual(vertical['local_takeoff_max_speed_mps'], 1.5)
        self.assertTrue(vertical['hold_yaw_enabled'])
        self.assertEqual(next(n for n in nodes if n['name'] == 'altitude_guard')['parameters']['maximum_climb_m'], 4.)

    def test_every_launched_executable_is_packaged(self):
        for node in build_launch():
            setup = ast.parse((ROOT / 'ros2_ws/src' / node['package'] / 'setup.py').read_text())
            strings = [n.value for n in ast.walk(setup) if isinstance(n, ast.Constant) and isinstance(n.value, str)]
            self.assertTrue(any(s.startswith(node['executable'] + ' = ') for s in strings), node['executable'])
