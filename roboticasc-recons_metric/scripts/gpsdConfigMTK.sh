#!/bin/bash

echo "***** gpsd configuration *****"

#sudo killall gpsd

echo "Setting up MTK3339 10Hz baud 57600"
rosrun nmea_navsat_driver mtk3339_config.py _port:=/dev/cp210x _baud:=9600

echo "Connecting GPS gpsd port 2949"
gpsd -N -S 2949 /dev/cp210x &

echo "Lauching gpsd_client "
roslaunch recons_metric gpsdMTK.launch

#ps -e | grep -w "gpsd"

