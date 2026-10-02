import socket
import tempfile
from pathlib import Path
import unittest
from we_meet_vio.camera_wire import pack,unpack,capture_time,MAX_PACKET


class CameraIPCTests(unittest.TestCase):
    def test_full_resolution_real_unix_socket(self):
        try:
            probe=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
            probe.close()
        except PermissionError:
            self.skipTest("runtime blocks socket syscalls; actual local IPC is tested in Jazzy CI")
        with tempfile.TemporaryDirectory() as directory:
            path=str(Path(directory)/"camera.sock")
            with socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as reader, socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM) as writer:
                reader.bind(path)
                reader.settimeout(1.)
                writer.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,2*1024*1024)
                meta={"schema":1,"encoding":"mono8","width":640,"height":480,"sensor_ns":1000000000}
                pixels=b'\x70'*(640*480)
                packet=pack(meta,pixels)
                writer.sendto(packet,path)
                restored,data=unpack(reader.recv(MAX_PACKET))
                self.assertEqual(restored,meta)
                self.assertEqual(data,pixels)

    def test_corrupt_or_truncated_packet_rejected(self):
        meta={"schema":1,"encoding":"mono8","width":640,"height":480}
        packet=pack(meta,b'\x00'*(640*480))
        for broken in (b'',packet[:10],packet[:-1],b'\xff\xff\xff\xff'+packet[4:]):
            with self.assertRaises((ValueError,KeyError)):
                unpack(broken)

    def test_capture_stamp_preserved_despite_receive_delay(self):
        self.assertAlmostEqual(capture_time(1000000000,100050000000,1050000000),100.)
        with self.assertRaises(ValueError):
            capture_time(1000000000,100200000000,1200000000)
