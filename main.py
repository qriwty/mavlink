import math
import time
from mavlink import MAVLinkHandler, DataAcquisitionThread
from mavlink.data import QueuePipe, Attitude, GlobalPosition, LocalPosition
from mavlink.processor import DataProcessor, GimbalProcessor, GlobalPositionProcessor, AttitudeProcessor
from pymavlink import mavutil
from mavlink.data import MAVLinkDataType, Gimbal, RCChannels, ServoChannels


mavlink_connection = MAVLinkHandler("udp:0.0.0.0:14550")

attitude_processor = AttitudeProcessor()
gimbal_processor = GimbalProcessor()

acquisition_thread = DataAcquisitionThread(mavlink_connection, [attitude_processor, gimbal_processor])
acquisition_thread.start()

for _ in range(3):
    print(attitude_processor.queue.get_latest())
    print(gimbal_processor.queue.get_latest())

    time.sleep(1)

