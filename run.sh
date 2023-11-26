#!/bin/bash

# first spin the dependencies
# python3 src/check_sensor.py &

python3 src/dockSkeleton.py

# shutdown the dependencies when the main process is killed
# trap "trap - SIGTERM && kill -- -$$" SIGINT SIGTERM EXIT
