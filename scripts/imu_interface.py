#!/usr/bin/env python3
from pymavlink import mavutil
from sensor_msgs.msg import Imu
import rospy
import numpy as np
# quaternion_from_euler
import tf.transformations as tf
# Create the connection at port:
UDP_PORT = 14552 # 14550 for QGC, 14552 for mavros (if QGC is already using 14550)

# Switch between different imu data feeds
# RAW_IMU = mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU
# SCALED_IMU2 = mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2
# The Event camera has IMU data data as well, but it is not used here as it is (eventually) published by the camera driver as ROS messages
# MAVLINK_TYPE = mavutil.mavlink.MAVLINK_MSG_ID_SCALED_IMU2
MAVLINK_TYPE = mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU
ROS_TOPIC = 'imu_pixhawk'
NODE_NAME = 'imu_node'
FRAME_ID = 'imu_frame'
MSG_TYPE = Imu
master = mavutil.mavlink_connection(f'udpin:0.0.0.0:{UDP_PORT}')

# Give the connection some time to initialize before sending commands
master.wait_heartbeat()

def request_message_interval(message_id: int, frequency_hz: float):
    print("Requesting message interval for message ID {} at {} Hz".format(message_id, frequency_hz))
    
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        message_id, # The MAVLink message ID
        1e6 / frequency_hz, # The interval between two messages in microseconds. Set to -1 to disable and 0 to request default rate.
        0, 0, 0, 0, # Unused parameters
        0 # Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.
    )

# A function to publish IMU data
def get_imu_message(msg):
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
        imu_msg.header.frame_id = FRAME_ID
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

request_message_interval(MAVLINK_TYPE, 100) # 100 is the frequency in Hz

rospy.init_node(NODE_NAME)  # initialize node

# save the publisher in a variable
imuPublisher = rospy.Publisher(ROS_TOPIC, MSG_TYPE, queue_size=10) # que_size is used to limit the number of messages to be stored if the subscriber is not receiving them fast enough

print(f"Relaying {ROS_TOPIC} topic...")

while not rospy.is_shutdown():
    msg = master.recv_match(timeout=1)
    if not msg:
        continue
    if msg.get_type() == 'RAW_IMU':
        get_imu_message(msg.to_dict())
        
        # Uncomment to see the rate at which messages are received:
        
        # if not hasattr(get_imu_message, "last_received_time"):
        #     get_imu_message.last_received_time = rospy.get_rostime()
        # else:
        #     current_time = rospy.get_rostime()
        #     delta_time = (current_time - get_imu_message.last_received_time).to_sec()
        #     get_imu_message.last_received_time = current_time
        #     print(f"Received message {MAVLINK_TYPE} on {ROS_TOPIC} topic. Rate: {1 / delta_time:.2f} Hz")

