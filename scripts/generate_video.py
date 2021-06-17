#!/usr/bin/env python
"""
Subscribes to an image topic and generates a video file from the
resulting Image messages.


"""
import argparse
import rospy

from sensor_msgs.msg import Image, CameraInfo, CompressedImage
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import sys
from datetime import datetime

FRAME_RATE = 30.0
FRAME_TIME = 1.0 / FRAME_RATE

class ExtractNode(object):
    """
    """

    def __init__(self, image_topic, image_info_topic, filename):
        rospy.init_node('extract_video')
        self.cv_bridge = CvBridge()

        rospy.loginfo("waiting for camera info...")
        self.info_msg = rospy.wait_for_message(image_info_topic, CameraInfo)
        rospy.loginfo("camera info received.")
        
        rospy.loginfo("subscribing to {}".format(image_topic))
        
        fourcc = cv2.VideoWriter_fourcc(*'DIVX')
        self.writer = cv2.VideoWriter(filename, fourcc, FRAME_RATE, 
                                     (self.info_msg.width,
                                      self.info_msg.height))

        #rospy.Subscriber(image_topic, Image, self.image_callback)
        rospy.Subscriber(image_topic, CompressedImage,
                         self.compressed_image_callback)
        
                
        self.num_frames = 0
        self.start_time = None
        rospy.spin()

    def handle_image(self, img_msg, cv_img):
        cur_time = img_msg.header.stamp.to_sec()
        
        time_fmt = '%Y-%m-%d %H:%M:%S.%f'
        time_str = datetime.utcfromtimestamp(cur_time).strftime(time_fmt)[:-4]

        if self.start_time is None:
            self.start_time = cur_time

        #rospy.loginfo("image callback")
        while (self.start_time + self.num_frames * FRAME_TIME < cur_time):
            #rospy.loginfo("frame added" + time_str)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(cv_img, time_str, (10,self.info_msg.height -10),
                        font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            self.writer.write(cv_img)
            self.num_frames += 1
               
        
    def compressed_image_callback(self, img_msg):
        np_arr = np.frombuffer(img_msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.handle_image(img_msg, cv_img)
        
    def image_callback(self, img_msg):
        cur_time = img_msg.header.stamp.to_sec()
        cv_img = self.cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
        self.handle_image(img_msg, cv_img)

    def done(self):
        self.writer.release()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--image_topic', default='/usb_cam/image_raw/compressed')
    parser.add_argument('--image_info_topic', default='/usb_cam/camera_info')
    parser.add_argument('--name', default='out.mkv', help="output file name.")
    args = parser.parse_args()
    node = ExtractNode(args.image_topic, args.image_info_topic, args.name)
    node.done()
