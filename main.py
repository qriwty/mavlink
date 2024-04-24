import math
import time
from mavlink import MAVLinkHandler, DataAcquisitionThread
from mavlink.data import QueuePipe, Attitude, GlobalPosition, LocalPosition
from mavlink.processor import DataProcessor, LocalPositionProcessor, GlobalPositionProcessor, AttitudeProcessor


mavlink_connection = MAVLinkHandler("udp:0.0.0.0:14550")

attitude_processor = AttitudeProcessor()
attitude_thread = DataAcquisitionThread(mavlink_connection, attitude_processor)
attitude_thread.start()

for _ in range(3):
    data = attitude_processor.simple_extrapolation()
    if not data:
        time.sleep(1)
        continue

    print(data)

attitude_thread.join()


from pymavlink import mavutil
from mavlink.data import MAVLinkDataType, Gimbal, RCChannels, ServoChannels

try:
    mavlink_connection = mavutil.mavlink_connection("udp:0.0.0.0:14550")

    while True:
        message = mavlink_connection.recv_match(blocking=True)
        if message.msgname == MAVLinkDataType.GIMBAL.value:
            gimbal = Gimbal.from_mavlink(message)
            print("GIMBAL", [math.degrees(rad) for rad in gimbal.quaternion.to_euler()])

except KeyboardInterrupt:
    print("Stopping message reception.")

