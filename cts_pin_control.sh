#!/bin/bash

#Enable CTS
echo 17 > /sys/class/gpio/export 2>/dev/null
sleep 0.5
echo out > /sys/class/gpio/gpio17/direction 2>/dev/null
sleep 0.5
echo 0 > /sys/class/gpio/gpio17/value 2>/dev/null
