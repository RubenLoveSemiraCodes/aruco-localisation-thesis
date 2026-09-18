#!/bin/bash

WORKSPACE=~/aruco-localisation-thesis

source /opt/ros/humble/setup.bash
source "$WORKSPACE/install/setup.bash"

while true
do
    echo ""
    echo "======================================"
    echo "          READY FOR TEST"
    echo "======================================"
    echo ""

    read -p "Press ENTER to start detector..."

    echo ""
    echo "========== TEST STARTED =========="
    echo ""

    ros2 run aruco_localisation detector \
        --ros-args \
        -p image_topic:=/camera/camera/color/image_raw

    echo ""
    echo "========== TEST FINISHED =========="
    echo ""
done
