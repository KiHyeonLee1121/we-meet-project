from glob import glob
from setuptools import find_packages, setup

setup(
    name='we_meet_prototype141', version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/we_meet_prototype141']),
        ('share/we_meet_prototype141', ['package.xml']),
        ('share/we_meet_prototype141/config', glob('config/*.yaml')),
        ('share/we_meet_prototype141/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='WE-MEET', maintainer_email='prototype@example.invalid',
    description='Archive-based reconstruction of DA-DAKA flight 141.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'flight141_mission = we_meet_prototype141.node:main',
    ]},
)
