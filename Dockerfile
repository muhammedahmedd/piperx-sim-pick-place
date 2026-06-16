FROM osrf/ros:humble-desktop

RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    ros-humble-xacro \
    ros-humble-joint-state-publisher-gui \
    ros-humble-tf-transformations \
    ros-humble-moveit-configs-utils \
    ros-humble-moveit \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-topic-based-ros2-control \
    ros-humble-ros-testing \
    x11-apps

WORKDIR /workspace

COPY ros_entrypoint.sh /

RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
