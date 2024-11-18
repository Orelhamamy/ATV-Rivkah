FROM ros:humble-ros-base-jammy

SHELL [ "/bin/bash", "-c"]


# RUN git clone -b humble-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git src

COPY ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

COPY src/ /root/ros2_ws/src
WORKDIR /root/ros2_ws


# Build ros workspace
RUN     source /opt/ros/${ROS_DISTRO}/setup.bash && \
        colcon build 




ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]