### object_detection
Object + pose detection module running with ROS2

### Requirements

Docker, or a working ROS2 Humble Desktop installation

### Installation

0. Clone this repo, cd into root folder
1. Build docker image with `sudo docker buildx build -t object_detection .`
2. Run interactive container with `bash run_docker.sh`
3. Forward GUI with `bash forward_x11.sh` : check that xeyes can be seen. If not check where .Xauthority file is, and change its path in `run_docker.sh`.
4. Put a MP4 video of your choosing in `/input` of your choosing and rename it `sidewalk.mp4`. Download the one I used with `wget https://videos.pexels.com/video-files/30061113/12894145_2560_1440_30fps.mp4`
5. Build the package. `cd /ros2_ws` followed by `colcon build`

### Launching

I have not included a launch file yet so it's going to be a little tedious, sorry about that.

1. Open 4 terminals, `bash run_docker.sh` in each of them.
2. `source ros2_ws/install/setup.bash` in each of them.
3. Run one of the 4 nodes in each terminal. `ros2 run object_detection ` followed by `video_node`, `detector_node`, `display_node`, and `pose_detector_node`.

#### Todo

Flake8 the code. There are probably a few unused packages here and there

Apply black formatting

Make a launch file

Run models with GPU using onnxruntime's CUDA Execution Provider or TensorRT (instead of CPU).
Lots of performance gains to be made

Merge pose and object detection topics using bounding box IoU criterion in another node

Comment more, write docstrings

