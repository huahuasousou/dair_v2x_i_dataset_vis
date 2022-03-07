# bin/bash
XSOCK=/tmp/.X11-unix
XAUTH=$(mktemp /tmp/.docker.xauth.XXXXXXXXX)
xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
chmod 777 $XAUTH
docker run -it --gpus all -e DISPLAY=$DISPLAY -v $XAUTH:$XAUTH -e XAUTHORITY=$XAUTH -v /work/data01/dair_v2x_i:/work/data01/dair_v2x_i -v /work/cliu/2022test/dair_v2x_i_dataset_vis:/code python:3.7.11 /bin/bash

