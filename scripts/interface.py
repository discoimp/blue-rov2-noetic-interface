#!/usr/bin/env python3
from pymavlink import mavutil
from sensor_msgs.msg import Imu
import rospy
import numpy as np
# quaternion_from_euler
import tf.transformations as tf
# Create the connection
# From topside computer
# UDPport = 14552
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

def request_message_interval(message_id: int, frequency_hz: float):
    """
    Request MAVLink message in a desired frequency,
    documentation for SET_MESSAGE_INTERVAL:
        https://mavlink.io/en/messages/common.html#MAV_CMD_SET_MESSAGE_INTERVAL

    Args:
        message_id (int): MAVLink message ID
        frequency_hz (float): Desired frequency in Hz
    """
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0, # Unused parameters
        0, # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )

# A function to publish IMU data
def get_imu_pixhawk_message(msg):
        ## acc, gyro and orientation in m/s^2, rad/s
        ## 
        
        accx = msg["xacc"]/100
        accy = msg["yacc"]/100
        accz = msg["zacc"]/100
        
        gyrox = np.math.radians(msg["xgyro"])
        gyroy = np.math.radians(msg["ygyro"])
        gyroz = np.math.radians(msg["zgyro"])
        

        #IMU_time = time.time()
        IMU_time = msg["time_usec"]*1e-6
        
        
        imu_msg = Imu()
        imu_msg.header.frame_id = "imu_pixhawk"
        # imu_msg.header.stamp = rospy.Time.from_sec(IMU_time)
        imu_msg.header.stamp = rospy.get_rostime()
        
        # in case of time offset between ROS and IMU
        # timeOffset = imu_msg.header.stamp.to_sec() - IMU_time
        
        ## acceleration
        imu_msg.linear_acceleration.x = accx
        imu_msg.linear_acceleration.y = accy
        imu_msg.linear_acceleration.z = accz

        ## gyro
        imu_msg.angular_velocity.x = gyrox
        imu_msg.angular_velocity.y = gyroy
        imu_msg.angular_velocity.z = gyroz
        
        ## orientation not collected
        imu_msg.orientation.x = 0
        imu_msg.orientation.y = 0
        imu_msg.orientation.z = 0
        imu_msg.orientation.w = 1
        
        ## covariance not collected
        not_defined = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        imu_msg.orientation_covariance = not_defined
        imu_msg.angular_velocity_covariance = not_defined
        imu_msg.linear_acceleration_covariance = not_defined
        imuPublisher.publish(imu_msg)

request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2, 1000) # 100 is the frequency in Hz

rospy.init_node('imu_pixhawk')  # initialize node

# save the publisher in a variable
imuPublisher = rospy.Publisher('imu_pixhawk', Imu, queue_size=1) # que_size is used to limit the number of messages to be stored if the subscriber is not receiving them fast enough
# Request parameter
master.mav.param_request_read_send(
    master.target_system, master.target_component,
    b'SCALED_IMU2',
    -1
)

# Print old parameter value
message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
print('name: %s\tvalue: %d' %
    (message['param_id'].decode("utf-8"), message['param_value']))

time.sleep(1)

while True:
    msg = master.recv_match()
    if not msg:
        continue
    if msg.get_type() == 'SCALED_IMU2':
        # get_imu_pixhawk_message(msg.to_dict())
        imu_data = msg.to_dict()
        print("Accel: ({:.2f}, {:.2f}, {:.2f}) m/s^2 | Gyro: ({:.2f}, {:.2f}, {:.2f}) rad/s".format(
            imu_data["xacc"]/100, imu_data["yacc"]/100, imu_data["zacc"]/100,
            np.math.radians(imu_data["xgyro"]), np.math.radians(imu_data["ygyro"]), np.math.radians(imu_data["zgyro"])))
