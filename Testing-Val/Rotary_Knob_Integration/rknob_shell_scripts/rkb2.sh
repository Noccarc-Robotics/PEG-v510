#!/bin/bash

prev=$(cat /sys/bus/counter/devices/counter0/count0/count)
new=$(cat /sys/bus/counter/devices/counter0/count0/count)

while :
do
        var=$(gpioget gpiochip5 10)

        if [[ $var -eq 0 ]]
	then
		echo "Button pressed , please release and rotate"
		prev=$new
		new=$(cat /sys/bus/counter/devices/counter0/count0/count)
		
	else
		
		dir=($new-$prev)
		
		if [[ $dir -eq 0 ]]
		then
			echo "Direction:At rest"
			prev=$new
			new=$(cat /sys/bus/counter/devices/counter0/count0/count)

		elif [[ $dir -lt 0 ]]
		then
			echo "Direction: Backward"
			prev=$new
			new=$(cat /sys/bus/counter/devices/counter0/count0/count)

		else
			echo "Direction: Forward"
			prev=$new
			new=$(cat /sys/bus/counter/devices/counter0/count0/count)
		fi	
	fi

done
