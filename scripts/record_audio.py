#!/usr/bin/env python
"""
Pick the card and device using 

# arecord -l

in the terminal.

You can see properties of the device by looking in /proc/asound. E.g. for hw:1,0 

cat /proc/asound/card1/stream0

"""

import rospy

from sensor_msgs.msg import CameraInfo
import subprocess
import sys
from datetime import datetime
import argparse

TIME_FORMAT = '%Y-%m-%d-%H-%M-%S'

class RecordNode(object):
    """
    """

    def __init__(self):
        rospy.init_node('record_audio')
        
        rate = rospy.get_param('~rate')
        channels = rospy.get_param('~channels')
        device = rospy.get_param('~device')
        format = rospy.get_param('~format')
        image_info_topic = rospy.get_param('~image_info_topic')
        prefix = rospy.get_param('~prefix')


        # Don't start recording until camera starts
        rospy.loginfo("waiting for camera info: {}".format(image_info_topic))
        info_msg = rospy.wait_for_message(image_info_topic, CameraInfo)
        rospy.loginfo("camera info received.")
        self.start_time = rospy.get_rostime().to_sec()

        utc_time = datetime.utcfromtimestamp(self.start_time)
        self.start_str = prefix + "_" + utc_time.strftime(TIME_FORMAT)

        self.record_process = subprocess.Popen(['arecord','--format={}'.format(format),
                                                '--rate={}'.format(rate),
                                                '--device={}'.format(device),
                                                '--channels={}'.format(channels),
                                                self.start_str + '.wav'])

        rospy.spin()

    def done(self):
        end_time = rospy.get_rostime().to_sec()
        milli_time_format = '%Y-%m-%d-%H-%M-%S.%f'

        with open(self.start_str + '.info', 'w') as f:   
            start_str = datetime.utcfromtimestamp(self.start_time).strftime(milli_time_format)
            end_str = datetime.utcfromtimestamp(end_time).strftime(milli_time_format)
            f.write("started: {}\n".format(start_str))
            f.write("ended:   {}\n".format(end_str))
            
        self.record_process.terminate()


if __name__ == "__main__":
    # parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    # parser.add_argument('--rate', default=32000, type=int,
    #                     help='audio sample rate')
    # parser.add_argument('--channels', default=2, type=int,
    #                     help='number of audio channels')
    # parser.add_argument('--device', default='hw:1,0',
    #                     help='device string in format expected by arecord')
    # parser.add_argument('--image_info_topic', default='/usb_cam/camera_info',
    #                     help='image info topic (just for time stamp)')
    # parser.add_argument('--prefix', default='',
    #                     help='prefix for file name')
    # args = parser.parse_args()
    
    node = RecordNode()
    node.done()    

