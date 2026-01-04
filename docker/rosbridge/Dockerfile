FROM ros:jazzy-ros-base

RUN apt-get update && apt-get install -y \
    ros-jazzy-rosbridge-server \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

COPY rosbridge_params.yaml /ros2_ws/rosbridge_params.yaml

EXPOSE 9090

CMD bash -lc "source /opt/ros/jazzy/setup.bash && \
  ros2 run rosbridge_server rosbridge_websocket \
  --ros-args --params-file /ros2_ws/rosbridge_params.yaml"
