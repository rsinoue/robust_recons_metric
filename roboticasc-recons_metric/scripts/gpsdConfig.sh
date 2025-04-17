#!/bin/bash

echo "***** gpsd configuration *****"

#sudo killall gpsd

echo "Connecting GPS MTK at port 2949"
gpsd /dev/cp210x -S 2949
echo "Connecting GPS RTK at port 2948"
gpsd /dev/pl2303 -S 2948

ps -e | grep -w "gpsd"

