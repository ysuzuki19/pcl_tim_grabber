#!/bin/bash

while :
do
	if [ $? ]; then
		make
	fi
	res=$(fswatch -1 --event Updated ..)
done
