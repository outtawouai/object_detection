# This is an auto generated Dockerfile for ros:desktop
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:humble-ros-base-jammy

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# xauth to forward GUI + vim + pip
RUN apt-get update && apt-get install -y \
xauth \
x11-apps \
vim \
pip \
wget

# object detection libraries (YOLO as implemented by Ultralytics : https://docs.ultralytics.com/modes/predict/)
RUN pip install \
opencv-python \
ultralytics

# resolve conflict : ultralytics installs a recent version of numpy that causes conflicts with cv2
RUN pip uninstall -y numpy
RUN pip install "numpy<2.0"

