from glob import glob
from setuptools import setup

setup(name='we_meet_flight_v1', version='0.3.0', packages=['we_meet_flight_v1'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/we_meet_flight_v1']),
                  ('share/we_meet_flight_v1', ['package.xml']),
                  ('share/we_meet_flight_v1/config', glob('config/*.yaml')),
                  ('share/we_meet_flight_v1/launch', glob('launch/*.launch.py'))],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='we-meet team', maintainer_email='maintainers@example.com',
      description='Camera-free commanded-distance flight trial', license='UNLICENSED',
      entry_points={'console_scripts': ['flight_trial = we_meet_flight_v1.node:main']})
