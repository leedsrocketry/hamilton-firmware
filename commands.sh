#!/bin/bash

# Function to run the Docker container
run() {
    sudo docker run -it --rm --privileged \
        -v "$(pwd)":/project \
        stm32-development
}

# Function to build the Docker image
build() {
    sudo docker build -t stm32-development .
}
