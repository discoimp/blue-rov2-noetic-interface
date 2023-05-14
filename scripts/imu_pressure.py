#!/usr/bin/env python3
from pymavlink import mavutil
from sensor_msgs.msg import Imu, FluidPressure
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped

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

# AHRS2 AHRS = Attitude and Heading Reference System
# MAVLINK_AHRS2_TYPE = mavutil.mavlink.MAVLINK_MSG_ID_AHRS2
# DVL_ROS_TOPIC = 'dvl_data'
# DVL_NODE_NAME = 'dvl_node'
# DVL_FRAME_ID = 'dvl_frame'
# DVL_MSG_TYPE = PoseStamped

# DEPTH data
MAVLINK_DEPTH_TYPE = mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2
FLUIDPRESSURE_ROS_TOPIC = 'pressure_data'
FLUIDPRESSURE_NODE_NAME = 'pressure_node'
FLUIDPRESSURE_FRAME_ID = 'pressure_frame'
FLUIDPRESSURE_MSG_TYPE = FluidPressure

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
        
        
        
        imu_msg = Imu()
        imu_msg.header.frame_id = FRAME_ID
        # IMU_time = msg["time_usec"]*1e-6
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

def get_dvl_message(msg):
    dvl_msg = PoseStamped()
    dvl_msg.header.frame_id = DVL_FRAME_ID
    dvl_msg.header.stamp = rospy.get_rostime()

    dvl_msg.pose.position.x = msg["lng"]
    dvl_msg.pose.position.y = msg["lat"]
    dvl_msg.pose.position.z = msg["altitude"]

    quaternion = tf.quaternion_from_euler(
        np.math.radians(msg["roll"]),
        np.math.radians(msg["pitch"]),
        np.math.radians(msg["yaw"])
    )
    dvl_msg.pose.orientation.x = quaternion[0]
    dvl_msg.pose.orientation.y = quaternion[1]
    dvl_msg.pose.orientation.z = quaternion[2]
    dvl_msg.pose.orientation.w = quaternion[3]
    
    dvlPublisher.publish(dvl_msg)

def get_pressure_message(msg):
    pressure_msg = FLUIDPRESSURE_MSG_TYPE()
    pressure_msg.header.frame_id = FLUIDPRESSURE_FRAME_ID
    pressure_msg.header.stamp = rospy.get_rostime()

    pressure_msg.fluid_pressure = msg["press_abs"]

    pressurePublisher.publish(pressure_msg)

request_message_interval(MAVLINK_TYPE, 100) # 100 is the frequency in Hz
# request_message_interval(MAVLINK_AHRS2_TYPE, 100) # 100 is the frequency in Hz
request_message_interval(MAVLINK_DEPTH_TYPE, 100) # 100 is the frequency in Hz

rospy.init_node(NODE_NAME)  # initialize node



# Initialize the publishers
# que_size = limit stored messages if subscriber not receiving them fast enough
# dvlPublisher = rospy.Publisher(DVL_ROS_TOPIC, DVL_MSG_TYPE, queue_size=10)
imuPublisher = rospy.Publisher(ROS_TOPIC, MSG_TYPE, queue_size=10)
pressurePublisher = rospy.Publisher(FLUIDPRESSURE_ROS_TOPIC, FLUIDPRESSURE_MSG_TYPE, queue_size=10)


print(f"Relaying {ROS_TOPIC} and {DVL_ROS_TOPIC} topic...")

while not rospy.is_shutdown():
    msg = master.recv_match(timeout=1)
    if not msg:
        continue
    if msg.get_type() == 'RAW_IMU':
        get_imu_message(msg.to_dict())
    # elif msg.get_type() == 'AHRS2':
    #    get_dvl_message(msg.to_dict())
    elif msg.get_type() == 'SCALED_PRESSURE2':
        get_pressure_message(msg.to_dict())
    



