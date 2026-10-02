from glob import glob
from setuptools import setup

setup(name='we_meet_flight_core', version='0.2.0', packages=['we_meet_flight_core'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/we_meet_flight_core']),
                  ('share/we_meet_flight_core', ['package.xml']),
                  ('share/we_meet_flight_core/config', glob('config/*.yaml'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='we-meet team', maintainer_email='maintainers@example.com',
      description='Shared velocity, lidar and yaw trial controller', license='UNLICENSED')
