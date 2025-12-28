# !/bin/bash

CONTAINER_NAME="plan_move_execute"

docker start $CONTAINER_NAME
docker exec -it $CONTAINER_NAME /bin/bash

