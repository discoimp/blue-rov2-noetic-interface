#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError, CvBridge
import cv2

def image_callback(msg):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    cv2.imshow("Video Subscriber", cv_image)
    cv2.waitKey(1)

def video_subscriber_node():
    rospy.init_node("video_subscriber_node", anonymous=True)
    rospy.Subscriber("/camera/image_raw", Image, image_callback)

    rospy.loginfo("Video subscriber node started. Press Ctrl+C to stop.")
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    try:
        video_subscriber_node()
    except rospy.ROSInterruptException:
        pass