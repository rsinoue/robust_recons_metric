#!/bin/bash

echo "***** gpsd configuration *****"

#sudo killall gpsd

echo "Connecting GPS RTK at port 2948"
gpsd -N -S 2948 /dev/pl2303

#ps -e | grep -w "gpsd"

