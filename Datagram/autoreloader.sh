#!/bin/bash

while :
do
	if make ; then
		./project
		echo "=================================================="
		echo "-- Program Finished ------------------------------"
		echo "=================================================="
	fi
	res=$(fswatch -1 --event Updated ..)
done
