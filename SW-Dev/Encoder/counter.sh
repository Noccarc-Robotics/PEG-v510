#!/bin/bash

echo "Setting up quadrature encoder on boot"
echo "quadrature x4" > /sys/bus/counter/devices/counter0/count0/function 
echo 65535 > /sys/bus/counter/devices/counter0/count0/ceiling 
echo 1 > /sys/bus/counter/devices/counter0/count0/enable
