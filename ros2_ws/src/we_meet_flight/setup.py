from glob import glob
from setuptools import setup

setup(name='we_meet_flight', version='0.2.0', packages=['we_meet_flight'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/we_meet_flight']),
                  ('share/we_meet_flight', ['package.xml']),
                  ('share/we_meet_flight/config', glob('config/*.yaml')),
                  ('share/we_meet_flight/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='we-meet team', maintainer_email='maintainers@example.com',
      description='Velocity and yaw trial with OpenCV panel alignment', license='UNLICENSED',
      entry_points={'console_scripts': ['flight_trial = we_meet_flight.node:main',
                                       'camera_source = we_meet_flight.camera_node:main']})
