#!/bin/bash
cd "$(dirname "$0")"

echo $HOME

docker run -it \
    -e DISPLAY=$DISPLAY \
    -v $HOME/.Xauthority:/root/.Xauthority:rw \
    -v /tmp/.X11-unix/:/tmp/.X11-unix/ \
    -v /dev/shm:/dev/shm \
    -v $PWD:/home/object_detection/ \
    --net=host \
    -w /home/object_detection/ \
    object_detection
