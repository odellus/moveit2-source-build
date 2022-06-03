#! /bin/bash -e

image_name="moveit2-docker"
image_tag=$IMAGE_TAG
full_image_name=${image_name}:${image_tag}

docker build -t ${full_image_name} -f Dockerfile .