FROM nvcr.io/nvidia/isaac/ros:noble-ros2_jazzy_d3e84470d576702a380478a513fb3fc6-amd64@sha256:82cb5792395aaa0834dc0d02878b3e90d89e215ee0677539e06df0336086ad6e
USER root
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR /opt/isaac-ros-build
RUN git clone --depth 1 --branch v4.0-0 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git src/isaac_ros_common && \
    test "$(git -C src/isaac_ros_common rev-parse HEAD)" = "68d835816a906e1b5d643bffbfe72e940ae37ea9" && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build --merge-install --install-base /opt/isaac-ros-4.0.0 \
        --packages-select isaac_ros_common isaac_ros_apriltag_interfaces \
        --cmake-args -DBUILD_TESTING=OFF '-DCMAKE_CUDA_ARCHITECTURES=86;89;120' && \
    source /opt/isaac-ros-4.0.0/setup.bash && \
    ros2 pkg prefix isaac_ros_common && \
    ros2 interface show isaac_ros_apriltag_interfaces/msg/AprilTagDetectionArray > /dev/null && \
    rm -rf build log src
WORKDIR /workspaces/isaac_ros-dev
