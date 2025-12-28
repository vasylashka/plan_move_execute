# !/bin/bash

IMAGE_NAME="plan_move_execute"

cd ..

docker build -t $IMAGE_NAME .

if [ $? -eq 0 ]; then
    echo "Docker image '$IMAGE_NAME' built successfully."
else
    echo "Failed to build Docker image '$IMAGE_NAME'."
    exit 1
fi

