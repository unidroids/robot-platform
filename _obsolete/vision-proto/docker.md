docker run -it --ipc=host --runtime nvidia --privileged -v /tmp/argus_socket:/tmp/argus_socket -v /var/run/nvidia-persistenced/socket:/var/run/nvidia-persistenced/socket --device /dev/video0 --device /dev/video1 -v /opt/projects/robotour:/workspace ultralytics/ultralytics:latest-jetson-jetpack6


docker run -it --ipc=host --runtime nvidia -v /tmp:/tmp -v /data:/data -v /var/run/nvidia-persistenced/socket:/var/run/nvidia-persistenced/socket -v /opt/projects/robotour:/workspace ultralytics/ultralytics:latest-jetson-jetpack6


docker run -it --ipc=host --net=host --runtime nvidia -v /tmp:/tmp -v /data:/data -v /opt/projects/robotour:/opt/projects/robotour -w /opt/projects/robotour/vision ultralytics/ultralytics:latest-jetson-jetpack6 bash
