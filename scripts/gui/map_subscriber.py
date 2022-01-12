#!/usr/bin/env python3

import sys
import os
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


current_map = os.path.join(os.path.dirname(__file__), "assets/current_map.png")


class MapConverter:
    """
    Saves a ROS image file as a local png file so that the GUI can use it

    Subscribes: /map_image
    """

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/map_image", Image, self.callback)

    def callback(self, data):
        """
        Updates map once per second

        :param data: sensor_msgs.Image
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imwrite(current_map, cv_image)
        rospy.sleep(1)


def main(args):
    map_converter = MapConverter()
    rospy.init_node('map_converter', anonymous=True)
    rospy.loginfo("Map Subscriber Initialized")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == '__main__':
    main(sys.argv)
