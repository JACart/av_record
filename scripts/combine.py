import subprocess
import argparse
import time
import tempfile
import shutil
import os
import rosbag
import cv2
import sys
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from datetime import datetime

FRAME_RATE = 30.0
FRAME_TIME = 1.0 / FRAME_RATE


def select_topic(bag, msg_types, prompt="Select a topic: "):
    topics = bag.get_type_and_topic_info()[1]

    info_topics = {i: t for i, t in enumerate(
        topics) if topics[t][0] in msg_types}
    print(prompt + "\n")

    default = 0
    for i in info_topics:
        default = i
        print("{} {}".format(i, info_topics[i]))
    print

    topic_num = int(
        raw_input("Enter topic num (default {}): ".format(default)) or str(default))
    return info_topics[topic_num]


def get_info_topic(bag, image_topic):
    topic_dict = bag.get_type_and_topic_info()[1]
    topic_base = "/".join(image_topic.split("/")[:-2])
    for topic in topic_dict:
        if topic_dict[topic][0] == 'sensor_msgs/CameraInfo' and topic_base in topic:
            return topic


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument('bagfile', help="the bag file to process")
    parser.add_argument('audiofile', help="the wav file to process")
    # parser.add_argument('image_topic')
    # parser.add_argument('image_info_topic')

    args = parser.parse_args()
    dir_name = os.path.dirname(args.bagfile)
    base_name = os.path.splitext(os.path.basename(args.bagfile))[0]

    tmp_dir = tempfile.mkdtemp()

    print(tmp_dir)

    bag = rosbag.Bag(args.bagfile)
    image_topic = select_topic(bag, set(
        ['sensor_msgs/CompressedImage', 'sensor_msgs/Image']), "Select an image topic:")
    pip_image_topic = select_topic(bag, set(
        ['sensor_msgs/CompressedImage', 'sensor_msgs/Image']), "Select a pip image topic:")
    topic_base = "/".join(image_topic.split("/")[:-2])

    print(topic_base)
    out_name = base_name + "_" + topic_base.replace("/", "_") + '.mp4'

    image_info_topic = get_info_topic(bag, image_topic)
    pip_image_info_topic = get_info_topic(bag, pip_image_topic)
    print(image_info_topic)

    fourcc = cv2.VideoWriter_fourcc(*'DIVX')

    writer = None
    # '/usb_cam/image_raw/compressed'
    # /zed/zed_node/right/image_rect_color/compressed 1280,720
    img_storage = []
    pip_img_storage = []
    cur_time = None
    for topic, msg, t in bag.read_messages(topics=['/zed/zed_node/right/image_rect_color/compressed', '/usb_cam/image_raw/compressed']):
        _cv_bridge = CvBridge()
        cur_time = msg.header.stamp.to_sec()

        if writer is None:
            writer = cv2.VideoWriter(os.path.join(tmp_dir, 'out.avi'), fourcc, FRAME_RATE,
                                     (1280,
                                      720))
        if topic in image_topic:
            img_storage.append(msg.data)

        if topic in pip_image_topic:
            pip_img_storage.append(msg.data)

    scale = float(len(pip_img_storage)) / len(img_storage)

    print("SCALE: " + str(scale))

    for i in range(len(img_storage)):
        sys.stdout.write('\r'+ 'Compiling ' + str(i) + " / " + str(len(img_storage)))
        sys.stdout.flush()
        img_data = img_storage[i]
        pip_img_data = pip_img_storage[int(i * scale)]

        img_arr = np.frombuffer(img_data, np.uint8)
        pip_arr = np.frombuffer(pip_img_data, np.uint8)

        img = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)
        pip_img = cv2.imdecode(pip_arr, cv2.IMREAD_COLOR)

        time_fmt = '%Y-%m-%d %H:%M:%S.%f'
        time_str = datetime.fromtimestamp(
            cur_time).strftime(time_fmt)[:-4]
        cv2.putText(img, time_str, (10, 720 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

        pip_size = (384, 216)

        resized = cv2.resize(pip_img, pip_size,
                             interpolation=cv2.INTER_AREA)
        resized = cv2.copyMakeBorder(resized, 3, 3, 3, 3,
                                     cv2.BORDER_CONSTANT,
                                     value=(255, 255, 255))
        img[0:pip_size[1]+6, 0:pip_size[0]+6] = resized

        writer.write(img)
        cur_time += FRAME_TIME
    writer.release()
    print('done')
    print("Temp: " + os.path.join(tmp_dir, 'out.avi'))
    print("Path: " + os.path.join(dir_name, out_name))
    p = subprocess.Popen(['ffmpeg', '-i', os.path.join(tmp_dir, 'out.avi'),
                          '-i', args.audiofile,
                         '-c:v', 'libx264', '-crf', '22', '-c:a', 'aac',
                          os.path.join(dir_name, out_name)])
    p.wait()
    shutil.rmtree(tmp_dir)


if __name__ == "__main__":
    main()
