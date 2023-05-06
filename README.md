# av_record
ROS package for creating time-stamped videos with audio.

## Setup
Update the default arguments and parameters in `av_record.launch` to correspond to the image topics and microphone that you wish to record.  Use the following steps to select and configure a microphone:
```
arecord -l
```
This will list the available audio capture devices:
```
**** List of CAPTURE Hardware Devices ****
card 0: PCH [HDA Intel PCH], device 0: ALC892 Analog [ALC892 Analog]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 0: PCH [HDA Intel PCH], device 2: ALC892 Alt Analog [ALC892 Alt Analog]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
card 1: C920 [HD Pro Webcam C920], device 0: USB Audio [USB Audio]
  Subdevices: 1/1
  Subdevice #0: subdevice #0
```
Select the card number and device number that you want to use, and check on the capabilities by looking at the appropriate entry in `/proc/asound`:
```
$ more /proc/asound/card1/stream0 
HD Pro Webcam C920 at usb-0000:06:00.0-2, high speed : USB Audio

Capture:
  Status: Stop
  Interface 3
    Altset 1
    Format: S16_LE
    Channels: 2
    Endpoint: 2 IN (ASYNC)
    Rates: 16000
    Data packet interval: 1000 us
    Bits: 16
  Interface 3
    Altset 2
    Format: S16_LE
    Channels: 2
    Endpoint: 2 IN (ASYNC)
    Rates: 24000
    Data packet interval: 1000 us
    Bits: 16
  Interface 3
    Altset 3
    Format: S16_LE
    Channels: 2
    Endpoint: 2 IN (ASYNC)
    Rates: 32000
    Data packet interval: 1000 us
    Bits: 16
```
Update `av_record.launch` so that the audio device parameters are correct.  In this case, we might have something like:
```xml
   <param name="rate" value="32000" type="int"/>
   <param name="channels" value="2" type="int"/>
   <param name="device" value="hw:1,0"/>
   <param name="format" value="S16_LE"/>
 ```
 A problem arises if you already have a process accessing the capture device.  This can be addressed by creating an `/etc/asound.conf` file that replicates the device using `dsnoop`
 https://superuser.com/questions/903390/how-can-two-applications-read-from-the-same-digital-sound-input-in-linux-alsa
 See `misc/asound.conf` for an example. 

## Recording
Recording is performed by creating a bag file containing the required image messages along with a .wav file containing the audio.  Start recording by launching `av_record.launch`:
```bash
roslaunch av_record av_record.launch subject:=john_doe recording_path:=/scratch/data/
```
from folder **~/av_record/launch**. Parameters are option. Default subject name is "JanDoe" and the default path is "/scratch/trial_recordings/".

Audio and video recording can only be done while the local_admin ROS module is running. It is advised to launch the cart first and then this package.

## Generating Videos
Videos can only be generated after the camera publishers are shut down.  Running the `combine.py` script will play the appropriate bag file and launch processes to capture and convert the images into a single video file.  Type 
```
python combine.py --help 
```
for usage information.

Combining files cannot be performed while local-admin and ROS are running as the method requires playback within ROS.

## Automatically Generating and Combining Videos:

In this directory is a 'genvideo.py' script that automatically takes in the bag/info files to create the mp4 in one step. The command on small laptop is: 
```
genvideo <name-of-video> <absolute-path-to-recording>
```

If small laptop is not in use, the script can be added as an alias in .bashrc.

It saves the mp4 in `<absolute-path-to-recording>/saved_recordings`, and the bag/info files in `<absolute-path-to-recording>/saved_recordings/temp`. This will save headache finding the bag files and combining them.
