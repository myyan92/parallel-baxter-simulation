# A Dockerfile for the gym-gazebo environment
FROM nvidia/cuda:8.0-cudnn5-devel-ubuntu16.04

#--------------------
# General setup
#--------------------

# Get the dependencies
RUN apt-get update \
    && apt-get install -y xorg-dev \
    libgl1-mesa-dev \
    xvfb \
    libxinerama1 \
    libxcursor1 \
    unzip \
    libglu1-mesa \
    libav-tools \
    python-numpy \
    python-scipy \
    python-pyglet \
    python-setuptools \
    libpq-dev \
    libjpeg-dev \
    wget \
    curl \
    cmake \
    git \
    nano vim \
    locales \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/* \
 && easy_install pip

RUN apt-get update && \
    apt-get install -y python3-pip

#--------------------
# Install gym
#--------------------
WORKDIR /home

# Clone the official gym
RUN git clone https://github.com/openai/gym

# Install the gym's requirements
RUN pip install -r gym/requirements.txt

# Install the gym
RUN ls -l
RUN pip install -e gym/
RUN pip3 install -e gym/

# Checks
#RUN python --version
#RUN python -c "import gym"

#--------------------
# Install ROS
#--------------------
# setup environment
RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list

# setup key
RUN apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116

# install ROS kinectic
RUN apt-get update && apt-get install -y \
    ros-kinetic-desktop-full 

# install rosdep
RUN rosdep init \
    && rosdep update

RUN apt-get install -y \
    python-rosinstall ros-kinetic-qt-build \
    ros-kinetic-ros-control ros-kinetic-control-toolbox \
    ros-kinetic-realtime-tools ros-kinetic-ros-controllers \
    ros-kinetic-xacro python-wstool \
    ros-kinetic-tf-conversions ros-kinetic-kdl-parser

# install gazebo and dependencies
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable xenial main" > /etc/apt/sources.list.d/gazebo-stable.list
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN apt-get update && apt-get install -y gazebo7 libgazebo7-dev

# install gazebo-ros packages from source
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/kinetic/setup.bash && \
    mkdir -p /home/gazebo-ros/src && \
    cd /home/gazebo-ros/src && \
    catkin_init_workspace && \
    cd /home/gazebo-ros && \
    catkin_make

WORKDIR /home/gazebo-ros/src
RUN git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b kinetic-devel
RUN rosdep update && rosdep install --from-paths . --ignore-src --rosdistro kinetic -y
RUN apt-get install -y libignition-math2-dev
WORKDIR /home/gazebo-ros
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/kinetic/setup.bash && \
    catkin_make && catkin_make install

# install baxter SDK
RUN apt-get update && apt-get install -y \
    python-argparse python-wstool python-vcstools \
    python-rosdep ros-kinetic-control-msgs ros-kinetic-joystick-drivers

RUN mkdir -p /home/ros_ws/src && \
    cd /home/ros_ws/src && \
    wstool init . && \
    wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter/master/baxter_sdk.rosinstall && \
    wstool update

# source, build and install
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/kinetic/setup.bash && \
    cd /home/ros_ws && \
    catkin_make && \
    catkin_make install

# install baxter simulator
# RUN cd /home/ros_ws/src && \
#     wstool merge -y https://raw.githubusercontent.com/RethinkRobotics/baxter_simulator/kinetic-devel/baxter_simulator.rosinstall && \
#     wstool update
COPY ros_ws/src /home/ros_ws/src

# build and install 
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/kinetic/setup.bash && \
    source /home/gazebo-ros/devel/setup.bash && \
    cd /home/ros_ws && \
    catkin_make clean && \
    catkin_make && \
    catkin_make install && \
    cp src/baxter/baxter.sh .

# -------------------------------------------
# dependency for baxter bridge
# -------------------------------------------
RUN pip install zmq protobuf
RUN pip3 install zmq protobuf

# ---------------------------------------------
# installing tensorflow for python3
# ---------------------------------------------
RUN pip3 install --upgrade https://storage.googleapis.com/tensorflow/linux/gpu/tensorflow_gpu-1.2.1-cp35-cp35m-linux_x86_64.whl 

# --------------------------------------------
# clone openai baseline
# -------------------------------------------
WORKDIR /home

RUN git clone https://github.com/openai/baselines.git
RUN apt-get update && \
    apt-get install -y libblas-dev liblapack-dev libatlas-base-dev gfortran
RUN pip3 install scipy tqdm joblib dill progressbar2 mpi4py atari-py pillow
ENV PYTHONPATH /home/baselines:${PYTHONPATH}

# ------------------------------------------
# copy codes
# ------------------------------------------
COPY baxter_bridge /home/baxter_bridge
COPY test /home/test
COPY ros_ws/baxter.sh /home/ros_ws/baxter.sh
ENV PYTHONPATH /home/baxter_bridge:${PYTHONPATH}
# do not use gpu
ENV CUDA_VISIBLE_DEVICES=4 
