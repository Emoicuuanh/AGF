#!/bin/bash

time=`date +%F_%H%M%S`
file="$HOME/log/ros/$time.log"
mkdir -p "$HOME/log/ros"
echo $file | tee $file


source $HOME/.bashrc
export ROS_MASTER_URI=http://192.168.11.30:11311
export ROS_HOSTNAME=192.168.11.30
export AMR_MODEL=amr_2
export ROSCONSOLE_FORMAT='[${severity}] [${walltime}] [${node}] [${line}]: ${message}'

source /opt/ros/melodic/setup.bash
source $HOME/catkin_ws/devel/setup.bash

T=20 # Time must be differ with other roslauch cmd
if [ $# -ne 0 ]
then
T=$1
fi

echo "Launching application after $T (s), please wait..."
#sleep Ts

#for i in {0..$T..1}
for((i=1;i<=$T;i++))
do
  echo -n '.'
  sleep 1
done

roslaunch control_system robot.launch