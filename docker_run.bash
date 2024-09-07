xhost local:root


XAUTH=/tmp/.docker.xauth


docker run -it \
    --name=r13d_container \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    r1_image_from_file \
    bash

echo "Done."