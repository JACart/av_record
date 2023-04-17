mkdir $2/saved_recordings
cd $2/saved_recordings
mkdir temp

RECORDPATH=$2/saved_recordings/temp

roslaunch av_record av_record.launch subject:=$1 recording_path:=$RECORDPATH

BAGFILE=./$(ls . | grep ".bag" | head -1)
WAVFILE=./$(ls . | grep ".wav" | head -1)
INFOFILE=./$(ls . | grep ".info" | head -1)

echo $BAGFILE
echo $WAVFILE
echo $INFOFILE

python /home/jacart/catkin_ws/src/av_record/scripts/combine.py $BAGFILE $WAVFILE

mv $BAGFILE ./temp
mv $WAVFILE ./temp
mv $INFOFILE ./temp