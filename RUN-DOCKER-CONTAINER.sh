#!/bin/bash

IMAGE_NAME=roomba_hack
TAG_NAME=eloquent
CONTAINER_NAME="roomba_hack"
echo "$0: IMAGE=${IMAGE_NAME}:${TAG_NAME}"
echo "$0: CONTAINER=${CONTAINER_NAME}"

if [ ! -z $1 ]; then
    ROOMBA_NAME=$1
else
    ROOMBA_NAME=jetson-nano
fi

EXISTING_CONTAINER_ID=`docker ps -aq -f name=${CONTAINER_NAME}`
if [ ! -z "${EXISTING_CONTAINER_ID}" ]; then
    docker exec -it ${CONTAINER_NAME} bash
else
    if [ "$(uname -m)" == "aarch64" ]; then
        docker run -it --rm \
            --privileged \
            --runtime nvidia \
            --net host \
            --volume ${PWD}/ros2_ws/:/root/roomba_hack/ros2_ws/ \
            --volume /dev/:/dev/ \
            --name ${CONTAINER_NAME} \
            ${IMAGE_NAME}:jetson \
            bash
            # bash -c "source /root/.bashrc; roslaunch roomba_bringup bringup.launch"
    else
        xhost +
        ROOMBA_HOSTNAME=${ROOMBA_NAME}.local
        echo "Now resolving local host name '${ROOMBA_HOSTNAME}'..."
        ROOMBA_IP=`avahi-resolve -4 --name ${ROOMBA_HOSTNAME} | cut -f 2`
        if [ "$?" != "0" ]; then
            echo "Failed to execute 'avahi-resolve'. You may need to install 'avahi-utils'."
        elif [ ! -z "${ROOMBA_IP}" ]; then
            echo "Successfully resolved host name '${ROOMBA_HOSTNAME}' as '${ROOMBA_IP}'."
        fi
        docker run -it --rm \
            --privileged \
            --gpus all \
            --env DISPLAY=${DISPLAY} \
            --net host \
            --volume ${PWD}/ros2_ws/:/root/roomba_hack/ros2_ws/ \
            --volume /dev/:/dev/ \
            --volume /tmp/.X11-unix:/tmp/.X11-unix \
            --name ${CONTAINER_NAME} \
            ${IMAGE_NAME}:${TAG_NAME} \
            bash -c "sed -i 's/TMP_HOSTNAME/${ROOMBA_HOSTNAME}/' ~/.bashrc;
                     sed -i 's/TMP_IP/${ROOMBA_IP}/' ~/scripts/initialize-bash-shell.sh;
                     sed -n -e '/^[^#[:space:]]*[[:space:]]\+${ROOMBA_HOSTNAME}\$/!p' /etc/hosts > /etc/hosts.tmp;
                     echo '${ROOMBA_IP} ${ROOMBA_HOSTNAME}' >> /etc/hosts.tmp;
                     cp /etc/hosts.tmp /etc/hosts; 
                     sed -i 's/eloquent/galactic/' ~/scripts/initialize-catkin-workspace.sh;
                     bash"
    fi
fi
