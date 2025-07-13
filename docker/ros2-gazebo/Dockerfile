FROM osrf/ros:rolling-desktop

# install turtlebot3 gazebo and rosbridge
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-rolling-turtlebot3-gazebo \
        ros-rolling-rosbridge-server && \
    rm -rf /var/lib/apt/lists/*

ENV TURTLEBOT3_MODEL=burger

COPY launch_all.sh /launch_all.sh
RUN chmod +x /launch_all.sh

CMD ["/launch_all.sh"]
