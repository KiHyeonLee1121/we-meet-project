import json
import tempfile
from pathlib import Path
import unittest
import numpy as np
import yaml
import cv2
from we_meet_vio.settings import calibration,load
from configure_calibration import generate


def synthetic_calibration():
    """ISOLATED TEST fixture; not a measurement of the user's drone."""
    return {"verified":True,"timeshift_cam_imu":0.0,"resolution":[640,480],"intrinsics":[500.,500.,320.,240.],
            "distortion":[0.,0.,0.,0.],"distortion_model":"radtan","T_body_imu":np.eye(4).tolist(),
            "T_cam_imu":[[0.,-1,0,0],[-1,0,0,0],[0,0,-1,0],[0,0,0,1]],
            "camera_controls":{"ExposureTime":4000,"AnalogueGain":2.,"LensPosition":.33,
                               "FrameDurationLimits":[40000,40000],"AeEnable":False,"AfMode":"Manual"},
            "scaler_crop":[0,0,640,480],"sensor_clock":"boottime","imu_topic":"/test_imu",
            "imu_frame":"base_link","lidar_position_body":[0.,0.,0.],
            "lidar_geometry_verified":True,"raw_imu_source_verified":True,
            "imu_intrinsics_already_corrected":True,
            "calibration_evidence":"synthetic isolated unit-test fixture"}


class SettingsTests(unittest.TestCase):
    def test_unmeasured_mount_blocks(self):
        with tempfile.TemporaryDirectory() as directory:
            file=Path(directory)/"calibration.json"
            c=synthetic_calibration()
            c["T_body_imu"]=None
            file.write_text(json.dumps(c))
            with self.assertRaises(ValueError): calibration(file)

    def test_unverified_or_forward_camera_blocks(self):
        for modification in ({"verified":False},{"T_cam_imu":np.eye(4).tolist()},
                             {"raw_imu_source_verified":False},{"calibration_evidence":""}):
            with tempfile.TemporaryDirectory() as directory:
                file=Path(directory)/"calibration.json"
                c=synthetic_calibration()
                c.update(modification)
                file.write_text(json.dumps(c))
                with self.assertRaises(ValueError): calibration(file)

    def test_kalibr_converter_consistent_camera_and_topic(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            c=synthetic_calibration()
            camera={"cam0":{"camera_model":"pinhole","distortion_model":"radtan",
                            "T_cam_imu":c["T_cam_imu"],"intrinsics":c["intrinsics"],
                            "distortion_coeffs":c["distortion"],"resolution":c["resolution"],
                            "timeshift_cam_imu":.012}}
            imu={"imu0":{"accelerometer_noise_density":.01,"accelerometer_random_walk":.001,
                         "gyroscope_noise_density":.001,"gyroscope_random_walk":.0001,"update_rate":200}}
            for name,data in (("camera.yaml",camera),("imu.yaml",imu),("mount.yaml",c)):
                (root/name).write_text(yaml.safe_dump(data))
            result=generate(root/"camera.yaml",root/"imu.yaml",root/"mount.yaml",root/"out")
            restored=calibration(root/"out/calibration.json")
            self.assertEqual(restored["intrinsics"],camera["cam0"]["intrinsics"])
            self.assertEqual(result["timeshift_cam_imu"],.012)
            self.assertIn("/camera/image_raw",(root/"out/openvins/kalibr_imucam_chain.yaml").read_text())
            for file,node_name in (("kalibr_imucam_chain.yaml","cam0"),("kalibr_imu_chain.yaml","imu0")):
                storage=cv2.FileStorage(str(root/"out/openvins"/file),cv2.FILE_STORAGE_READ)
                self.assertTrue(storage.isOpened())
                self.assertFalse(storage.getNode(node_name).empty())
                storage.release()
            with self.assertRaises(ValueError):
                generate(root/"camera.yaml",root/"imu.yaml",root/"mount.yaml",root/"out")
