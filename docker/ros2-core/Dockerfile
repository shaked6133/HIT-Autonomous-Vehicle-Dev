FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    ros-jazzy-turtlesim \
    ros-jazzy-teleop-twist-keyboard \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
RUN mkdir -p src

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

CMD ["bash"]
