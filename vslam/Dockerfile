FROM ros:noetic

ENV DEBIAN_FRONTEND noninteractivve

ENV ROS_VER noetic

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV CATKIN_WS=/root/catkin_ws
ENV N_PROC = 2

RUN apt-get update && apt-get install -y \
    cmake \
    libatlas-base-dev \
    libeigen3-dev \
    libgoogle-glog-dev \
    libsuitesparse-dev \
    git \
    python3-catkin-tools \    
    ros-${ROS_VER}-image-geometry \
    ros-${ROS_VER}-pcl-ros \
    ros-${ROS_VER}-image-proc \
    ros-${ROS_VER}-tf-conversions \
    ros-${ROS_VER}-sophus \
    ros-${ROS_VER}-cv-bridge \
    ros-${ROS_VER}-tf2-geometry-msgs && \
    # System dependencies
    apt-get install -y \
    build-essential unzip pkg-config autoconf \
    libboost-all-dev \
    libjpeg-dev libpng-dev libtiff-dev \
    libvtk6-dev libgtk-3-dev \
    libatlas-base-dev gfortran \
    libparmetis-dev \
    libgl1-mesa-dev libglew-dev \
    x11-xserver-utils \
    python3-wstool python3-catkin-tools && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y 

RUN cd /root/ && \
    git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    git fetch && \
    git checkout tags/v0.6 && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF .. && \
    make -j4 && \
    rm -rf Pangolin


# Setup catkin workspace
RUN mkdir -p $CATKIN_WS/src/vslam/ && \
    cd ${CATKIN_WS} && \
    catkin init && \
    catkin config \
    --extend /opt/ros/${ROS_VER} \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release && \
    catkin config --merge-devel

# COPY ./ ${CATKIN_WS}/src/vslam

RUN cd $CATKIN_WS/src && \ 
    git clone https://github.com/hellovuong/vslam.git

RUN rosdep update

# RUN source /opt/ros/${ROS_VER}/setup.bash && \
RUN cd $CATKIN_WS/src && \
    wstool init && \
    wstool merge vslam/vslam.rosinstall   && \
    wstool update
RUN cd $CATKIN_WS && \
    catkin build -j4 catkin_simple && \ 
    catkin build -j4 dbow2_catkin && \ 
    catkin build -j4 g2o_catkin 

RUN xhost +local:docker

RUN cd $CATKIN_WS/src/vslam/ORB_SLAM3  && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=OFF .. && \
    make -j2 && \
    cd ../Vocabulary && \
    tar -xf ORBvoc.txt.tar.gz && \
    rm -rf ORBvoc.txt.tar.gz

WORKDIR ${CATKIN_WS}
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8   
RUN cd ${CATKIN_WS} && \ 
    catkin build -j2 && \
    sed -i '/exec "$@"/i \
    source "/root/catkin_ws/devel/setup.bash"' /ros_entrypoint.sh

RUN echo "source /opt/ros/${ROS_VER}/setup.bash" >> ~/.bashrc && \
    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
