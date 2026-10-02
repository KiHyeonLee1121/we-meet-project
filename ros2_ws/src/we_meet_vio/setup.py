from setuptools import setup
from glob import glob

package_name="we_meet_vio"
setup(name=package_name,version="0.1.0",packages=[package_name],
      data_files=[("share/ament_index/resource_index/packages",["resource/"+package_name]),
                  ("share/"+package_name,["package.xml"]),
                  ("share/"+package_name+"/launch",glob("launch/*.launch.py"))],
      install_requires=["setuptools","numpy","PyYAML"],zip_safe=True,
      maintainer="WE-MEET",maintainer_email="maintainer@example.invalid",
      description="OpenVINS + calibrated panel servo, single-panel 5m flight test",
      license="MIT",entry_points={"console_scripts":[
          "vio_bridge = we_meet_vio.vio_bridge:main",
          "imu_gate = we_meet_vio.imu_gate:main",
          "vio_flight = we_meet_vio.mission_node:main",
          "pi_camera = we_meet_vio.camera_node:main",
          "panel_detector = we_meet_vio.panel_node:main",
          "tf_luna = we_meet_vio.lidar_node:main"]})
