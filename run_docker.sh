#!/bin/bash

usage(){
    echo "Usage: $0 [-r <humble|iron|rolling>]"
    exit 1
}

ROS_DISTRO=${ROS_DISTRO:-"iron"}  # [humble, iron, rolling]
while getopts "r:" opt; do
    case $opt in
        r)
            if [ $OPTARG != "humble" ] && [ $OPTARG != "iron" ] && [ $OPTARG != "rolling" ]; then
                echo "Invalid ROS distro: $OPTARG" >&2
                usage
            fi
            ROS_DISTRO=$OPTARG
            ;;
        \?)
            echo "Invalid option: -$OPTARG" >&2
            usage
            ;;
    esac
done

XSOCK=/tmp/.X11-unix
XAUTH=$HOME/.Xauthority

docker pull georgno/sjtu_drone:ros2-${ROS_DISTRO}

if [ $? -ne 0 ]; then
    exit 1
fi


xhost +local:docker
docker run \
    -it --rm \
    $VOLUMES \
    -v ${XSOCK}:${XSOCK} \
    -v ${XAUTH}:${XAUTH} \
    -e DISPLAY=${DISPLAY} \
    -e XAUTHORITY=${XAUTH} \
    --env=QT_X11_NO_MITSHM=1 \
    --privileged \
    --net=host \
    --name="sjtu_drone" \
    georgno/sjtu_drone:ros2-${ROS_DISTRO}
xhost -local:docker
