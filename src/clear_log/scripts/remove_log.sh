#!/bin/bash

# contab -e
# 0,30 * * * * command # min(each 30 min) hour day_of_month month day_of_week command
# @reboot sh /home/penguin/catkin_vs/src/receptionist_robot/tools/removeLogs.sh # run when reboot
# 0 10 * * * sh /home/penguin/catkin_vs/src/receptionist_robot/tools/removeLogs.sh # run at 10 o'clock

log_ros="$HOME/log/ros"
log_ai="$HOME/log/ai"
log_web="$HOME/log/web"
log_sys="$HOME/.ros/log"

echo $log_ros
echo $log_ai
echo $log_web
echo $log_sys

if [ -d $log_ai ]
then
    find $log_ai -type f -mtime +3 -name '*.log' -exec rm -rf {} \;
    echo Clear all old log in $log_ai
else
    echo $log_ai not exist
fi

if [ -d $log_ros ]
then
    find $log_ros -type f -mtime +3 -name '*.log' -exec rm -rf {} \;
    echo Clear all old log in $log_ros
else
    echo $log_ros not exist
fi

if [ -d $log_web ]
then
    find $log_web -type f -mtime +3 -name '*.log' -exec rm -rf {} \;
    echo Clear all old log in $log_web
else
    echo $log_web not exist
fi

if [ -d $log_sys ]
then
    find $log_sys -type f -mtime +3 -name '*.log' -exec rm -rf {} \;
    echo Clear all old log in $log_sys
else
    echo $log_sys not exist
fi