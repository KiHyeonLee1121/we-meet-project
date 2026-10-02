FROM ros:jazzy-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
    git build-essential libeigen3-dev libboost-all-dev libceres-dev libopencv-dev geographiclib-tools wget \
    python3-colcon-common-extensions python3-opencv python3-numpy python3-yaml python3-serial \
    ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-mavros ros-jazzy-mavros-extras ros-jazzy-tf2-ros-py \
    && rm -rf /var/lib/apt/lists/*
RUN geographiclib-get-geoids egm96-5
WORKDIR /opt/we_meet
COPY . .
RUN python3 tools/fetch_openvins.py
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/jazzy/setup.bash && cd ros2_ws && MAKEFLAGS=-j2 colcon build \
    --packages-up-to ov_msckf we_meet_vio --executor sequential \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_ARUCO_TAGS=OFF -DDISABLE_MATPLOTLIB=ON
CMD ["bash"]
