#!/usr/bin/env python3

"""
BlueRov video capture class
"""

import cv2
import sys
import gi
import numpy as np
import time
import rospy
import sensor_msgs.msg
from cv_bridge import CvBridge

import threading


gi.require_version('Gst', '1.0')
from gi.repository import Gst


class Video():
    """BlueRov video capture class constructor

    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
        latest_frame (np.ndarray): Latest retrieved video frame
    """

    def __init__(self, port=5600):
        """Summary

        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self.latest_frame = self._new_frame = None
        self.latest_time = self._new_time = 0

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # self.video_source = '! multiudpsink clients=192.168.2.1:5600'
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
        self.video_codec = '! queue ! application/x-rtp, payload=96 ! queue ! rtph264depay ! h264parse ! queue ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)

        self.video_decode = \
            '! queue ! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'

        # Create a sink to get data
        self.video_sink_conf = \
            '! queue ! appsink emit-signals=true sync=true max-buffers=50 drop=false'

        self.video_pipe = None
        self.video_sink = None

        self.gstreamer_thread = threading.Thread(target=self.run, name = "gstreamer thread", args=())
        self.gstreamer_thread.daemon = True
        self.gstreamer_thread.start()

        # self.run()

    def stop_thread(self):
        self.gstreamer_thread.join()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        print(command)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps_structure = sample.get_caps().get_structure(0)
        array = np.ndarray(
            (
                caps_structure.get_value('height'),
                caps_structure.get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array

    def frame(self):
        """ Get Frame

        Returns:
            np.ndarray: latest retrieved image frame
        """
        if self.frame_available:
            self.latest_frame = self._new_frame
            self.latest_time = self._new_time
            # reset to indicate latest frame has been 'consumed'
            self._new_frame = None
            

        return self.latest_frame, self.latest_time

    def frame_available(self):
        """Check if a new frame is available

        Returns:
            bool: true if a new frame is available
        """
        return self._new_frame is not None

    def run(self):
        """ Get frame to update _new_frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])

        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        self._new_frame = self.gst_to_opencv(sample)
        self._new_time = time.time()

        return Gst.FlowReturn.OK



if __name__ == '__main__':
    # Create the video object
    # Add port= if is necessary to use a different one

    frame_rate = 30
    video = Video(port=5600)
    bridge = CvBridge()
    pub = rospy.Publisher("/camera/image_raw", sensor_msgs.msg.Image, queue_size = frame_rate)
    out_msg = sensor_msgs.msg.Image()

    rospy.init_node("bluerov_stream", anonymous=False)

    rate = rospy.Rate(200)

    print('Initialising stream...')
    waited = 0
    
    while not video.frame_available():
        waited += 1
        print('\r  Frame not available (x{})'.format(waited), end='')
        cv2.waitKey(30)
        if waited > 500:
            print("No connection available")
            rospy.signal_shutdown("No video")
            sys.exit(0)
            


    time_0 = time.time()
    fps_list = []
    counter = 0
    last_error = time.time()
    while True:
        # Wait for the next frame to become available
        if video.frame_available():
            # Only retrieve and display a frame if it's new
            frame, timestamp = video.frame()
            frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # detect if the frame is missing. 


            ## Detect missing frames
            s0 = time.time()
            grey_detector = frame_gray.std()
            s1 = time.time()

            if grey_detector < 10:
                last_error = time.time()
                print(f"Time: {s1} - std: {grey_detector} - calc_time: {s1-s0}")
            

            ## publish 
            out_msg = bridge.cv2_to_imgmsg(frame_gray, "mono8")
            out_msg.header.stamp = rospy.Time.from_sec(timestamp)


            # scale_percent = 60 # percent of original size

            # dim = (int(1280/2), int(720/2))
  
            # resize image
            # resized = cv2.resize(frame_gray, dim)
            # cv2.imshow("frame", resized)
            pub.publish(out_msg)

            # print(frame_gray.shape)

            ## fps calculation
            time_1 = time.time()
            time_delta = time_1 - time_0
            fps = 1 / (time_delta + 0.000001)
            fps_list.append(fps)
            fps_list = fps_list[-20:]
            time_0 = time_1

            counter += 1
            if counter > 300:
                counter = 0
                print(f"Time: {s1} - std: {grey_detector} - time_since_error: {time.time()-last_error:.3f}s - fps: {sum(fps_list) / len(fps_list)}")
                # print("fps: ", sum(fps_list) / len(fps_list))

            
            

        # # Allow frame to display, and check if user wants to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        if rospy.is_shutdown():
            break

        # rate.sleep()
    # time.sleep(0.00001)

    video.stop_thread()