#!/bin/bash

ver=$(uname -r)
if [[ $ver =~ "6.6"* ]]; then
    gpio=529
else
    gpio=17
fi
#Enable CTS
echo $gpio > /sys/class/gpio/export 2>/dev/null
sleep 0.5
echo out > /sys/class/gpio/gpio$gpio/direction 2>/dev/null
sleep 0.5
echo 0 > /sys/class/gpio/gpio$gpio/value 2>/dev/null
echo $gpio > /sys/class/gpio/unexport
