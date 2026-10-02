"""Camera-free adapter for the shared flight controller."""
from we_meet_flight_core.node import FlightNode as CoreFlightNode, run_node


class FlightNode(CoreFlightNode):
    package_name = 'we_meet_flight_v1'
    api_prefix = '/test_flying_v1'
    default_profile = 'velocity_trial.yaml'


def main(args=None):
    run_node(FlightNode, args)
