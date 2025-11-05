# rosbridge container
FROM ros:jazzy-ros-base

# Install rosbridge_server
RUN apt-get update && apt-get install -y \
    ros-jazzy-rosbridge-server \
 && rm -rf /var/lib/apt/lists/*

# Default WebSocket port
EXPOSE 9090

# Launch rosbridge (WebSocket server)
CMD bash -lc "source /opt/ros/jazzy/setup.bash \
  && if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi \
  && ros2 launch rosbridge_server rosbridge_websocket_launch.xml"
