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
attitude_thread = DataAcquisitionThread(mavlink_connection, [attitude_processor, gimbal_processor])
attitude_thread.start()

for _ in range(3):
    print(attitude_processor.queue.get_latest())
    print(gimbal_processor.queue.get_latest())
    time.sleep(1)


# while True:
#     message = mavlink_connection.receive_packet(MAVLinkDataType.GIMBAL.value)
#
#     gimbal = Gimbal.from_mavlink(message)
#     print("GIMBAL", [math.degrees(rad) for rad in gimbal.quaternion.to_euler()])
