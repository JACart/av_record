from __future__ import print_function
import subprocess
import argparse
import tempfile
import shutil
import os
import rosbag
import cv2
import sys
import numpy as np
from datetime import datetime

def select_topic(bag, msg_types, prompt="Select a topic: "):
    topics = bag.get_type_and_topic_info()[1]

    info_topics = {i: t for i, t in enumerate(
        topics) if topics[t][0] in msg_types}
    print(prompt + "\n")

    default = 0
    for i in info_topics:
        default = i
        print("{} {}".format(i, info_topics[i]))
    print()

    # So that this will work in Python 2 or 3...
    if sys.version_info[0] == 3:
        raw_input = input

    topic_num = int(raw_input("Enter topic num (default {}): ".format(default)) or str(default))
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
    parser.add_argument('--image-topic')
    parser.add_argument('--pip-image-topic')
    parser.add_argument('--pip-size', default=".2", type=float)
    parser.add_argument('--frame-rate', default="30", type=int)
    args = parser.parse_args()
    frame_rate = args.frame_rate
    frame_time = 1.0 / frame_rate
    dir_name = os.path.dirname(args.bagfile)
    base_name = os.path.splitext(os.path.basename(args.bagfile))[0]

    tmp_dir = tempfile.mkdtemp()

    print(tmp_dir)

    bag = rosbag.Bag(args.bagfile)
    image_topic = args.image_topic
    pip_image_topic = args.pip_image_topic
    if image_topic is None:
        image_topic = select_topic(bag, set(
            ['sensor_msgs/CompressedImage', 'sensor_msgs/Image']), "Select an image topic:")
    if pip_image_topic is None:
        pip_image_topic = select_topic(bag, set(
            ['sensor_msgs/CompressedImage', 'sensor_msgs/Image']), "Select a pip image topic:")

    topic_base = "/".join(image_topic.split("/")[:-2])

    out_name = base_name + "_" + topic_base.replace("/", "_") + '.mp4'

    image_info_topic = get_info_topic(bag, image_topic)
    pip_image_info_topic = get_info_topic(bag, pip_image_topic)

    # Set up the avi file writer
    image_info_gen = bag.read_messages(topics=[image_info_topic])
    _, img_info_msg, t = next(image_info_gen)
    fourcc = cv2.VideoWriter_fourcc(*'DIVX')
    writer = cv2.VideoWriter(os.path.join(tmp_dir, 'out.avi'), fourcc,
                             frame_rate, (img_info_msg.width,
                                          img_info_msg.height))

    # Figur out the PIP image size
    pip_info_gen = bag.read_messages(topics=[pip_image_info_topic])
    _, pip_info_msg, t = next(pip_info_gen)
    pip_width = int(img_info_msg.width * args.pip_size)
    pip_height = int((pip_width / float(pip_info_msg.width)) *
                     pip_info_msg.height)
    pip_size = (pip_width, pip_height)

    print("Processing bag file...")   
    image_gen = bag.read_messages(topics=[image_topic])
    pip_gen = bag.read_messages(topics=[pip_image_topic])

    _, prev_img, t = next(image_gen)
    _, prev_pip, t = next(pip_gen)
    if prev_img.header.stamp.to_sec() > prev_pip.header.stamp.to_sec():
        start_time = prev_img.header.stamp.to_sec()
    else:
        start_time = prev_pip.header.stamp.to_sec()
    end_time = bag.get_end_time()

    time_fmt = '%Y-%m-%d %H:%M:%S.%f'
    end_time_str = datetime.fromtimestamp(end_time).strftime(time_fmt)[:-4]
    
    cur_time = start_time + frame_time
    try:
        _, cur_img, t = next(image_gen)
        _, cur_pip, t = next(pip_gen)
        while True:

            # Read frames until we find one after the current frame
            # time. We'll use the last one BEFORE the current frame
            # time for this frame.
            while cur_img.header.stamp.to_sec() < cur_time:
                prev_img = cur_img
                _, cur_img, t = next(image_gen)

            while cur_pip.header.stamp.to_sec() < cur_time:
                prev_pip = cur_pip
                _, cur_pip, t = next(pip_gen)

            img_arr = np.frombuffer(prev_img.data, np.uint8)
            pip_arr = np.frombuffer(prev_pip.data, np.uint8)

            img = cv2.imdecode(img_arr, cv2.IMREAD_COLOR)
            pip_img = cv2.imdecode(pip_arr, cv2.IMREAD_COLOR)

            # Time stamp the main image
            img_time = prev_img.header.stamp.to_sec()
            time_str = datetime.fromtimestamp(img_time).strftime(time_fmt)[:-4]
            cv2.putText(img, time_str, (10, img_info_msg.height - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2,
                        cv2.LINE_AA)
            print(time_str + " / " + end_time_str, end='\r')

            # Time stamp the PIP image
            pip_time = prev_pip.header.stamp.to_sec()
            pip_time_str = datetime.fromtimestamp(pip_time).strftime(time_fmt)[:-4]
            cv2.putText(pip_img, pip_time_str, (10,
                                                pip_info_msg.height -10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1, (0, 255, 0), 2,
                        cv2.LINE_AA)

            resized = cv2.resize(pip_img, pip_size,
                                 interpolation=cv2.INTER_AREA)
            resized = cv2.copyMakeBorder(resized, 3, 3, 3, 3,
                                         cv2.BORDER_CONSTANT,
                                         value=(255, 255, 255))
            img[0:pip_size[1]+6, 0:pip_size[0]+6] = resized

            writer.write(img)
            cur_time += frame_time

    except StopIteration:
        print("Done processing images")
        writer.release()

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
