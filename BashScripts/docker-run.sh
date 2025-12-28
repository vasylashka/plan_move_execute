# !/bin/bash

#allow Docker use X-services
xhost +local:docker


CONTAINER_NAME="plan_move_execute"
IMAGE_NAME="plan_move_execute"

# Read and validate path
REPO_PATH=$(git rev-parse --show-toplevel 2>/dev/null)

CLION_PATH="/opt/Clion/clion-2024.2.1"
PYCHARM_PATH="/opt/PyCharm/pycharm-2024.2.1"

XSERVICES_PATH="$HOME/.Xauthority"

cd ..

docker run -it \
	--name $CONTAINER_NAME \
	-e DISPLAY=unix$DISPLAY  \
	-v $XSERVICES_PATH:/root/.Xauthority:rw \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v /tmp/.docker.xauth:/tmp/.docker.xauth \
	-v $CLION_PATH:/home/ros2_ws/Clion \
	-v $REPO_PATH:/home/ros2_ws/plan_move_execute  \
	-v $PYCHARM_PATH:/home/ros2_ws/PyCharm \
  --device=/dev/bus/usb:/dev/bus/usb \
  --device=/dev/video0:/dev/video0 \
  --device=/dev/video1:/dev/video1 \

  --volume /dev:/dev \
	--gpus all  \
	$IMAGE_NAME
	


