#!/bin/bash

initial_c=$(cat /sys/bus/counter/devices/counter0/count0/count)

while :
do
        var=$(gpioget gpiochip5 10)

        if [[ $var -eq 0 ]]
	then
		echo "Button pressed , please release and rotate"
		
	else
		cat /sys/bus/counter/devices/counter0/count0/count
	fi

done
