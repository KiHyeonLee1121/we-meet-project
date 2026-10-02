import cv2
import numpy as np


def image_to_bgr(msg):
    channels = {'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4, 'mono8': 1}.get(msg.encoding)
    if channels is None or msg.height < 1 or msg.width < 1 or msg.step < msg.width*channels:
        raise ValueError('unsupported image encoding or invalid dimensions/stride')
    if len(msg.data) != msg.height*msg.step:
        raise ValueError('invalid image buffer length')
    array = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.step)
    array = array[:, :msg.width*channels].reshape(msg.height, msg.width, channels).copy()
    codes = {'rgb8': cv2.COLOR_RGB2BGR, 'bgra8': cv2.COLOR_BGRA2BGR,
             'rgba8': cv2.COLOR_RGBA2BGR, 'mono8': cv2.COLOR_GRAY2BGR}
    return cv2.cvtColor(array, codes[msg.encoding]) if msg.encoding in codes else array
