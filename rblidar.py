import ctypes
import logging
import numpy as np

class point_data_t(ctypes.Structure):
    _pack_ = 1  # Set alignment to 1 byte
    _fields_ = [
        ("azimuth", ctypes.c_uint16),
        ("dist", ctypes.c_uint16),
        ("rssi", ctypes.c_uint16),
        ("timestamp", ctypes.c_uint32)
    ]

# Create a corresponding numpy dtype for the structure
point_dtype = np.dtype([
    ('azimuth', np.uint16),
    ('dist', np.uint16),
    ('rssi', np.uint16),
    ('timestamp', np.uint32)
])

rblidar_lib = ctypes.CDLL('/home/hgf/richbeam/rblidar/lidar/rb_lidar.so')  

CALLBACK_TYPE = ctypes.CFUNCTYPE(None, ctypes.c_void_p, ctypes.c_int)

class RBLidar:
    def __init__(self, ip: str, port: int, frame_callback=None):
        logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s: %(message)s')

        self.frame_callback = frame_callback
        self.callback = CALLBACK_TYPE(self._callback_wrapper)

        # Create the lidar instance
        self.lidar = rblidar_lib.rblidar_create(ip.encode('utf-8'), port, self.callback)

    def _callback_wrapper(self, data, length):
        # Convert the received data into a byte buffer
        byte_data = ctypes.string_at(data, length)

        # Create a NumPy array from the byte data
        point_data_array = np.frombuffer(byte_data, dtype=point_dtype)

        # If a NumPy callback is provided, call it
        if self.frame_callback is not None:
            self.frame_callback(point_data_array)

    def __del__(self):
        rblidar_lib.rblidar_destroy(self.lidar)

if __name__ == "__main__":
    def my_frame_callback(point_data_array):
        for point in point_data_array:
            logging.info(f"Azimuth: {point['azimuth']}, Distance: {point['dist']}, RSSI: {point['rssi']}, Timestamp: {point['timestamp']}")

    lidar = RBLidar("192.168.198.1", 2368, frame_callback=my_frame_callback) 
    try:
        while True:
            pass 
    except KeyboardInterrupt:
        logging.info("Stopping...")