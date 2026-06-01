# Tests Dockerfile 

This dockerfile is used to build an image for testing.  It build on top of the (private) base [xcore_builder](https://github.com/xmos/xcore_builder) image. Access to the private base image repository is not required to build this image.  

## Build Image

To build the docker container locally, run the following command in the root of the repository:

    docker build -t ghcr.io/xmos/xcore_voice_tester:develop -f tools/docker/Dockerfile.tests .

## Run Container

To run the container:

    docker run -it ghcr.io/xmos/xcore_voice_tester:develop bash

