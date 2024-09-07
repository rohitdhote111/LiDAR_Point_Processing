FROM osrf/ros:noetic-desktop-full

RUN apt-get update
RUN apt-get update && apt-get install -y \
    libpcl-dev && \
    pip3 install open3d

RUN mkdir -p ~/catkin_ws/src && \
    cd ~/catkin_ws/src/

RUN echo "ALL Done"
