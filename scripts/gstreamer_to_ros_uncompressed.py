#!/usr/bin/env python3

import rospy
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
from sensor_msgs.msg import CompressedImage

# Set the camera parameters
CAMERA_WIDTH = 1920
CAMERA_HEIGHT = 1080
CAMERA_FPS = 30

# Initialize GStreamer
Gst.init(None)

# Set the GStreamer pipelines for receiving the video stream
video_gst_pipeline = "udpsrc port=5600 ! application/x-rtp, payload=96 ! rtpjitterbuffer ! rtph264depay ! appsink name=video_sink"

# Create the GStreamer pipelines
video_pipeline = Gst.parse_launch(video_gst_pipeline)

# Get the appsink elements from the pipelines
video_sink = video_pipeline.get_by_name("video_sink")

# Set appsink properties
video_sink.set_property("emit-signals", True)
video_sink.set_property("sync", False)

# Initialize the ROS node and publisher
rospy.init_node('camera_publisher')
camera_pub = rospy.Publisher('/camera/image_raw/compressed', CompressedImage, queue_size=10)

def on_video_new_sample(appsink):
    sample = appsink.emit("pull-sample")
    buf = sample.get_buffer()
    result, mapinfo = buf.map(Gst.MapFlags.READ)

    if result:
        img_msg = CompressedImage()
        img_msg.header.stamp = rospy.Time.now()
        img_msg.format = "h264"
        img_msg.data = bytearray(mapinfo.data)
        camera_pub.publish(img_msg)

    buf.unmap(mapinfo)
    return Gst.FlowReturn.OK

video_sink.connect("new-sample", on_video_new_sample)

# Start the GStreamer pipelines
video_pipeline.set_state(Gst.State.PLAYING)

# Run the ROS loop
rospy.spin()

# Stop the GStreamer pipelines
video_pipeline.set_state(Gst.State.NULL)

rospy.loginfo("Camera publisher node stopped.")
