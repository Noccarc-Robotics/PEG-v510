#!/bin/bash


while :
do
	var=$(gpioget gpiochip5 10)
	echo $var	
done
