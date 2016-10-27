# ./rgbd_tum ../../Vocabulary/ORBvoc.bin TUM1.yaml ~/rgbd_dataset_freiburg1_desk/ associations/fr1_desk.txt 0 > log.txt
cat log.txt  | grep "Tracking Time" | grep -o '[0-9]*\.[0-9]*' > TrackingTimeLog.txt
cat log.txt  | grep "Track time" | grep -o '[0-9]*\.[0-9]*' > TrackTimelog.txt
cat log.txt  | grep "Frame time" | grep -o '[0-9]*\.[0-9]*' > FrameTimelog.txt
cat log.txt  | grep "ComputeKeyPoint" | grep -o '[0-9]*\.[0-9]*' > ComputeKeyPoint.txt
cat log.txt  | grep "computeDescriptors" | grep -o '[0-9]*\.[0-9]*' > computeDescriptors.txt
cat log.txt  | grep "Initial camera pose" | grep -o '[0-9]*\.[0-9]*' > InitialCameraPose.txt
cat log.txt  | grep "Track the local map" | grep -o '[0-9]*\.[0-9]*' > TrackLocalMap.txt
cat log.txt  | grep "LocalMap keypoints num" | grep -o '[0-9]*\.[0-9]*' > KeypointsNum.txt

gnuplot -e \
"plot 'TrackingTimeLog.txt' with linespoint ls 1,  \
'TrackTimelog.txt' with linespoint ls 2,  \
'FrameTimelog.txt' with linespoint ls 3,  \
'ComputeKeyPoint.txt' with linespoint ls 4,  \
'computeDescriptors.txt' with linespoint ls 5,  \
'InitialCameraPose.txt' with linespoint ls 6,  \
'KeypointsNum.txt' using 0:(\$1/100) with linespoint ls 7,  \
'TrackLocalMap.txt' with linespoint ls 8; pause -1"
