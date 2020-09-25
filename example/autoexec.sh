#!/bin/bash

while :
do
	if [ $? ]; then
		make
		if make; then
			./projec
		fi
	fi
	res=$(fswatch -1 --event Updated ..)
done
