#!/usr/bin/env python
"""
Subscribes to an image topic and generates a video file from the
resulting Image messages.


"""
import threading
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

    def __init__(self, image_topic, image_info_topic, pip_image_topic,
                 pip_image_info_topic, pip_size, filename):
        rospy.init_node('extract_video')
        self.cv_bridge = CvBridge()

        self.img_lock = threading.Lock()

        rospy.loginfo("waiting for camera info...")
        self.info_msg = rospy.wait_for_message(image_info_topic, CameraInfo)
        rospy.loginfo("camera info received.")

        if pip_image_info_topic is not None:
            rospy.loginfo("waiting for pip camera info...")
            self.pip_info_msg = rospy.wait_for_message(pip_image_info_topic, CameraInfo)
            rospy.loginfo("camera info received.")

        
        rospy.loginfo("subscribing to {}".format(image_topic))
        
        fourcc = cv2.VideoWriter_fourcc(*'DIVX')
        self.writer = cv2.VideoWriter(filename, fourcc, FRAME_RATE, 
                                     (self.info_msg.width,
                                      self.info_msg.height))

        #rospy.Subscriber(image_topic, Image, self.image_callback)
        rospy.Subscriber(image_topic, CompressedImage,
                         self.compressed_image_callback)

        self.pip_img = None
        if pip_image_topic is not None:
            rospy.Subscriber(pip_image_topic, CompressedImage,
                             self.pip_compressed_image_callback)
            pip_width = int(self.info_msg.width * pip_size)
            pip_height = int((pip_width / self.pip_info_msg.width) *
                             self.pip_info_msg.height)
            self.pip_size = (pip_width, pip_height)
        
                
        self.num_frames = 0
        self.start_time = None
        rospy.spin()

    def handle_image(self, img_msg, cv_img):
        cur_time = img_msg.header.stamp.to_sec()
        
        time_fmt = '%Y-%m-%d %H:%M:%S.%f'
        time_str = datetime.fromtimestamp(cur_time).strftime(time_fmt)[:-4]

        if self.start_time is None:
            self.start_time = cur_time


        if self.pip_img is not None:
            self.img_lock.acquire()
            pip_img = self.pip_img
            self.img_lock.release()
            resized = cv2.resize(pip_img, self.pip_size, interpolation = cv2.INTER_AREA)
            resized = cv2.copyMakeBorder(resized, 3, 3, 3, 3,
                                         cv2.BORDER_CONSTANT,
                                         value=(255,255,255))
            cv_img[0:self.pip_size[1]+6, 0:self.pip_size[0]+6] = resized


        #rospy.loginfo("image callback")
        while (self.start_time + self.num_frames * FRAME_TIME < cur_time):
            #rospy.loginfo("frame added" + time_str)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(cv_img, time_str, (10, self.info_msg.height - 10),
                        font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            self.writer.write(cv_img)
            self.num_frames += 1
               
        
    def compressed_image_callback(self, img_msg):
        np_arr = np.frombuffer(img_msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.handle_image(img_msg, cv_img)

    def pip_compressed_image_callback(self, img_msg):
        np_arr = np.frombuffer(img_msg.data, np.uint8)
        cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.img_lock.acquire()
        self.pip_img = cv_img
        self.img_lock.release()
        
    # def image_callback(self, img_msg):
    #     cur_time = img_msg.header.stamp.to_sec()
    #     cv_img = self.cv_bridge.imgmsg_to_cv2(img_msg, "bgr8")
    #     self.handle_image(img_msg, cv_img)

    def done(self):
        self.writer.release()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('--image_topic', default='/zed/zed_node/right/image_rect_color/compressed')
    parser.add_argument('--image_info_topic', default='/zed/zed_node/right/camera_info')
    parser.add_argument('--pip_image_topic')
    parser.add_argument('--pip_image_info_topic')
    parser.add_argument('--pip_size', default=".2", type=float)
    parser.add_argument('--name', default='out.mkv', help="output file name.")
    args = parser.parse_args()
    node = ExtractNode(args.image_topic, args.image_info_topic,
                       args.pip_image_topic,
                       args.pip_image_info_topic, args.pip_size,
                       args.name)
    node.done()
