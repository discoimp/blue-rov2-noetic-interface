#!/usr/bin/env python3

import rospy
import numpy as np
from pymavlink import mavutil
from sensor_msgs.msg import Imu, FluidPressure, Image
from std_msgs.msg import Float32, UInt32
import tf.transformations as tf

# Your imports for working with Gstreamer and camera frames
# ...

# Your camera frame callback
def camera_frame_callback(frame, meta_data):
    # Process your frame and meta_data here, then create and publish an Image message

    image_msg = Image()
    image_msg.header.stamp = rospy.get_rostime()
    # Fill other image_msg fields with frame and meta_data information
    # ...
    
    cameraPublisher.publish(image_msg)

# Add your Gstreamer and camera setup code here
# ...

UDP_PORT = 14552  # 14550 for QGC, 14552 for mavros (if QGC is already using 14550)
MAVLINK_TYPE = mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU
ROS_TOPIC_IMU = 'imu_pixhawk'
ROS_TOPIC_PRESSURE = 'scaled_pressure2'
ROS_TOPIC_CAM_TILT = 'named_value_float'
ROS_TOPIC_SYSTEM_TIME = 'system_time'
ROS_TOPIC_CAMERA = 'camera_frame'
NODE_NAME = 'mavlink_node'
FRAME_ID = 'imu_frame'
MSG_TYPE_IMU = Imu
MSG_TYPE_PRESSURE = FluidPressure
MSG_TYPE_CAM_TILT = Float32
MSG_TYPE_SYSTEM_TIME = UInt32
MSG_TYPE_CAMERA = Image
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
# Update get_imu_message function to include the timestamp in the returned dictionary
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

        return {
            "timestamp": imu_msg.header.stamp,
            "msg": imu_msg,
        }

def get_pressure_message(msg):
    pressure_msg = FluidPressure()
    pressure_msg.header.stamp = rospy.get_rostime()
    pressure_msg.fluid_pressure = msg["press_abs"]
    
    return {
        "timestamp": pressure_msg.header.stamp,
        "msg": pressure_msg,
    }

def get_cam_tilt_message(msg):
    cam_tilt_msg = Float32()
    if msg["name"] == "CamTilt":
        cam_tilt_msg.data = msg["value"]

    return {
        "timestamp": rospy.get_rostime(),
        "msg": cam_tilt_msg,
    }

def get_system_time_message(msg):
    system_time_msg = UInt32()
    system_time_msg.data = msg["time_boot_ms"]

    return {
        "timestamp": rospy.get_rostime(),
        "msg": system_time_msg,
    }

# Initialize publishers
imuPublisher = rospy.Publisher(ROS_TOPIC_IMU, MSG_TYPE_IMU, queue_size=10)
pressurePublisher = rospy.Publisher(ROS_TOPIC_PRESSURE, MSG_TYPE_PRESSURE, queue_size=10)
camTiltPublisher = rospy.Publisher(ROS_TOPIC_CAM_TILT, MSG_TYPE_CAM_TILT, queue_size=10)
systemTimePublisher = rospy.Publisher(ROS_TOPIC_SYSTEM_TIME, MSG_TYPE_SYSTEM_TIME, queue_size=10)
cameraPublisher = rospy.Publisher(ROS_TOPIC_CAMERA, MSG_TYPE_CAMERA, queue_size=10)

rospy.init_node(NODE_NAME)

request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_RAW_IMU, 100)
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SCALED_PRESSURE2, 30)
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_NAMED_VALUE_FLOAT, 1)
request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_SYSTEM_TIME, 1000)

# Add your Gstreamer and camera start code here, including the camera_frame_callback registration
# ...

print(f"Waiting for messages...")

while not rospy.is_shutdown():
    msg = master.recv_match(timeout=1)
    if not msg:
        continue

    msg_type = msg.get_type()
    msg_dict = msg.to_dict()

    if msg_type == "RAW_IMU":
        imu_msg = get_imu_message(msg_dict)
        imuPublisher.publish(imu_msg["msg"])

    elif msg_type == "SCALED_PRESSURE2":
        pressure_msg = get_pressure_message(msg_dict)
        pressurePublisher.publish(pressure_msg["msg"])

    elif msg_type == "NAMED_VALUE_FLOAT":
        cam_tilt_msg = get_cam_tilt_message(msg_dict)
        camTiltPublisher.publish(cam_tilt_msg["msg"])

    elif msg_type == "SYSTEM_TIME":
        system_time_msg = get_system_time_message(msg_dict)
        systemTimePublisher.publish(system_time_msg["msg"])

    # Add your camera frame processing and synchronization code here
    # ...