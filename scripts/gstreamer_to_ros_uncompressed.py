#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import json
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

# Set the camera parameters
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
CAMERA_FPS = 30

# Initialize GStreamer
Gst.init(None)

# Set the GStreamer pipelines for receiving the video stream and metadata
video_gst_pipeline = "udpsrc port=5600 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert ! video/x-raw,format=I420 ! appsink name=video_sink"
metadata_gst_pipeline = "udpsrc port=5601 ! text/plain ! appsink name=metadata_sink"

# Create the GStreamer pipelines
video_pipeline = Gst.parse_launch(video_gst_pipeline)
metadata_pipeline = Gst.parse_launch(metadata_gst_pipeline)

# Get the appsink elements from the pipelines
video_sink = video_pipeline.get_by_name("video_sink")
metadata_sink = metadata_pipeline.get_by_name("metadata_sink")

# Set appsink properties
video_sink.set_property("emit-signals", True)
video_sink.set_property("sync", False)

metadata_sink.set_property("emit-signals", True)
metadata_sink.set_property("sync", False)

# Initialize the ROS node, publishers, and CvBridge
rospy.init_node('camera_publisher')
camera_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
camera_info_pub = rospy.Publisher('/camera/camera_info', CameraInfo, queue_size=10)
bridge = CvBridge()

def on_video_new_sample(appsink):
    sample = appsink.emit("pull-sample")
    buf = sample.get_buffer()
    result, mapinfo = buf.map(Gst.MapFlags.READ)

    if result:
        frame = np.frombuffer(mapinfo.data, dtype=np.uint8)
        y = frame[:1920 * 1080].reshape((CAMERA_HEIGHT, CAMERA_WIDTH))
        u = frame[1920 * 1080:1920 * 1080 * 5 // 4].reshape((CAMERA_HEIGHT // 2, CAMERA_WIDTH // 2))
        v = frame[1920 * 1080 * 5 // 4:].reshape((CAMERA_HEIGHT // 2, CAMERA_WIDTH // 2))
        
        u_resized = cv2.resize(u, (CAMERA_WIDTH, CAMERA_HEIGHT), interpolation=cv2.INTER_LINEAR)
        v_resized = cv2.resize(v, (CAMERA_WIDTH, CAMERA_HEIGHT), interpolation=cv2.INTER_LINEAR)
        
        yuv = cv2.merge([y, u_resized, v_resized])
        bgr_frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR)
        publish_frame(bgr_frame)

    buf.unmap(mapinfo)
    return Gst.FlowReturn.OK


def on_metadata_new_sample(appsink):
    sample = appsink.emit("pull-sample")
    buf = sample.get_buffer()
    result, mapinfo = buf.map(Gst.MapFlags.READ)

    if result:
        metadata_str = mapinfo.data.decode("utf-8")
        metadata = json.loads(metadata_str)
        process_metadata(metadata)

    buf.unmap(mapinfo)
    return Gst.FlowReturn.OK

video_sink.connect("new-sample", on_video_new_sample)
metadata_sink.connect("new-sample", on_metadata_new_sample)

def process_metadata(metadata):
    # Process the received metadata, e.g., update camera parameters
    pass

def publish_frame(frame):
    # Convert the OpenCV frame to a ROS message
    img_msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')

    # Set the ROS message header
    img_msg.header.stamp = rospy.Time.now()

    # Publish the ROS message
    camera_pub.publish(img_msg)

    # Create a CameraInfo message for the metadata
    cam_info_msg = CameraInfo()
    cam_info_msg.header.stamp = rospy.Time.now()
    cam_info_msg.height = frame.shape[0]
    cam_info_msg.width = frame.shape[1]

    # Set the camera matrix
    fx = fy = 1000
    cx = frame.shape[1] / 2
    cy = frame.shape[0] / 2
    cam_info_msg.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]

    # Set the distortion coefficients
    cam_info_msg.D = [0, 0, 0, 0, 0]

    # Set the rectification matrix
    cam_info_msg.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]

    # Set the projection matrix
    cam_info_msg.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]

    # Publish the CameraInfo message
    camera_info_pub.publish(cam_info_msg)

# Start the GStreamer pipelines
video_pipeline.set_state(Gst.State.PLAYING)
metadata_pipeline.set_state(Gst.State.PLAYING)

# Run the ROS loop
rospy.spin()

# Stop the GStreamer pipelines
video_pipeline.set_state(Gst.State.NULL)
metadata_pipeline.set_state(Gst.State.NULL)

rospy.loginfo("Camera publisher node stopped.")
