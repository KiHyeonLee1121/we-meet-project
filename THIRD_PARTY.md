# Dependencies

This repository's new, independent Python nodes and tools use the MIT license in LICENSE.
OpenVINS is a separate ROS process downloaded at the commit in config/openvins.lock.json.
Its original GPL-3.0-or-later license and notices remain in that checkout; patches do not replace them.
The parameter schema in config/openvins/estimator_config.yaml is adapted from that pinned
upstream's config/rs_d455/estimator_config.yaml. It contains no copied RealSense calibration.

MAVROS, ROS 2, OpenCV, Picamera2, numpy, PyYAML and pyserial retain their respective licenses.
MAVROS sources were read to establish transport/frame semantics; no MAVROS plugin source is copied here.
No historical v1/v2 controller, laptop detector, or archived 141 flight script is imported.
