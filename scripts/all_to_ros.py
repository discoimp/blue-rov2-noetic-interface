#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from pymavlink import mavutil
from mavros_msgs.msg import Mavlink

# Set the UDP port to connect to the autopilot
UDP_PORT = 14552

# Set the topic name for publishing MAVLink messages
MAVLINK_TOPIC_PREFIX = 'mavlink'

# Create a MAVLink connection to the autopilot
mavlink_conn = mavutil.mavlink_connection(f'udp:0.0.0.0:{UDP_PORT}')

# Create a dictionary to store the publishers for each message type
mavlink_pubs = {}

# Define a callback function to publish MAVLink messages on ROS topics
def mavlink_to_ros_msg(mavlink_msg):
    # Create a ROS message from the MAVLink message
    ros_msg = Mavlink()
    ros_msg.header.stamp = rospy.get_rostime()
    ros_msg.header.frame_id = 'mavlink'

    ros_msg.framing_status = Mavlink.FRAMING_OK
    ros_msg.magic = mavlink_msg.get_magic()
    ros_msg.len = mavlink_msg.get_msgId()
    ros_msg.seq = mavlink_msg.get_seq()
    ros_msg.sysid = mavlink_msg.get_srcSystem()
    ros_msg.compid = mavlink_msg.get_srcComponent()
    ros_msg.msgid = mavlink_msg.get_msgId()
    ros_msg.payload64 = mavlink_msg.pack64()

    # Get the message type name and create a publisher for the corresponding topic
    msg_type = mavutil.mavlink.enums["MAV_CMD"][mavlink_msg.get_msgId()]
    if msg_type:
        topic_name = f"{MAVLINK_TOPIC_PREFIX}/{msg_type.name}"
        if topic_name not in mavlink_pubs:
            mavlink_pubs[topic_name] = rospy.Publisher(topic_name, Mavlink, queue_size=10)
        mavlink_pubs[topic_name].publish(ros_msg)
    else:
        rospy.logwarn(f"No message type found for message ID {mavlink_msg.get_msgId()}")

# Initialize the ROS node
rospy.init_node('mavlink_publisher')

# Start the MAVLink connection and wait for messages
mavlink_conn.wait_heartbeat()

# Process incoming MAVLink messages in a loop
while not rospy.is_shutdown():
    mavlink_msg = mavlink_conn.recv_match(blocking=True)
    if mavlink_msg:
        mavlink_to_ros_msg(mavlink_msg)
