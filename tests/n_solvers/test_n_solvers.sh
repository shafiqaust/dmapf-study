#!/bin/bash
set -x

S=(4 8 12 16 20 24 28 32)
I=10

for s in ${S[@]}; do
  rosrun dmapf gendiv -d -f $HOME/catkin_ws/src/DMAPF/examples/lak303d/r200/lak303d_n14784_r200.lp -k 240 --bk-means -s ${s} --seed 2
  rosclean purge -y; sleep 10s
  for (( i=1; i<=$I; i++ )); do
    roslaunch dmapf lak303d_n14784_r200.launch &> lak303d_r200_s${s}_i${i}.txt
    rosclean purge -y; sleep 10s
  done
done

for s in ${S[@]}; do
  rosrun dmapf gendiv -d -f $HOME/catkin_ws/src/DMAPF/examples/lak303d/r400/lak303d_n14784_r400.lp -k 240 --bk-means -s ${s} --seed 2
  rosclean purge -y; sleep 10s
  for (( i=1; i<=$I; i++ )); do
    roslaunch dmapf lak303d_n14784_r400.launch &> lak303d_r400_s${s}_i${i}.txt
    rosclean purge -y; sleep 10s
  done
done

for s in ${S[@]}; do
  rosrun dmapf gendiv -d -f $HOME/catkin_ws/src/DMAPF/examples/lak303d/r600/lak303d_n14784_r600.lp -k 240 --bk-means -s ${s} --seed 2
  rosclean purge -y; sleep 10s
  for (( i=1; i<=$I; i++ )); do
    roslaunch dmapf lak303d_n14784_r600.launch &> lak303d_r600_s${s}_i${i}.txt
    rosclean purge -y; sleep 10s
  done
done
