import subprocess
import argparse
import time
import tempfile
import shutil
import os
import rosbag

def select_topic(bag, msg_types, prompt="Select a topic: "):
    topics = bag.get_type_and_topic_info()[1]

    info_topics = {i: t for i, t in enumerate(topics) if topics[t][0] in msg_types}
    print(prompt + "\n")

    default = 0
    for i in info_topics:
        default = i
        print("{} {}".format(i, info_topics[i]))
    print
    
    topic_num = int(raw_input("Enter topic num (default {}): ".format(default)) or str(default))
    return info_topics[topic_num]

def main():
    parser = argparse.ArgumentParser()

    parser.add_argument('bagfile', help="the bag file to process")
    parser.add_argument('audiofile', help="the wav file to process")
    #parser.add_argument('image_topic')
    #parser.add_argument('image_info_topic')

    args = parser.parse_args()
    dir_name = os.path.dirname(args.bagfile)
    base_name = os.path.splitext(os.path.basename(args.bagfile))[0]
    

    tmp_dir = tempfile.mkdtemp()

    print(tmp_dir)

    bag = rosbag.Bag(args.bagfile)
    #image_info_topic = select_topic(bag, set(['sensor_msgs/CameraInfo']), "Select an image info topic:")
    image_topic = select_topic(bag, set(['sensor_msgs/CompressedImage', 'sensor_msgs/Image']), "Select an image topic:")
    topic_base = "/".join(image_topic.split("/")[:-2])
    print(topic_base)
    out_name = base_name + "_" + topic_base.replace("/", "_") + '.mp4'

    topic_dict = bag.get_type_and_topic_info()[1]
    for topic in topic_dict:
        if topic_dict[topic][0] == 'sensor_msgs/CameraInfo' and topic_base in topic:
            image_info_topic = topic
            break
    
    print(image_info_topic)
    
    p0 = subprocess.Popen(['roscore'])
    p1 = subprocess.Popen(['rosrun', 'av_record', 'generate_video.py',
                           '--image_topic', image_topic,
                           '--image_info_topic', image_info_topic,
                           '--name', os.path.join(tmp_dir, 'out.avi')])
    time.sleep(5)
    p2 = subprocess.Popen(['rosbag', 'play', '--rate', '2', args.bagfile])
    p2.wait()
    time.sleep(.5)
    p1.terminate()

    p = subprocess.Popen(['ffmpeg', '-i', os.path.join(tmp_dir, 'out.avi'),
                          '-i', args.audiofile,
                          '-c:v', 'copy', '-c:a', 'aac',
                          os.path.join(dir_name,out_name)])
    p.wait()
    shutil.rmtree(tmp_dir)

if __name__ == "__main__":
    main()
