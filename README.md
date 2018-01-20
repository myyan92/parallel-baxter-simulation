# parallel-baxter-simulation
modified baxter simulation package to enable simulation of multiple baxter robot inside Gazebo

This repo is a docker image for Ubuntu16.04, ROS kinetic, Gazebo7 and baxter SDK / simulation package.
This repo also include a ZMQ based bridge to control baxter robot in python3 with minimal code. Motivation being to use this with RL/IL algorithms implemented in python3, such as OpenAI's baselines.
Gym, Tensorflow and baselines are also installed in the Dockerfile, you can remove them if you don't need these components.

## Usage
prerequisite: Docker CE. If you have a NVIDIA GPU and want to use vision in Gazebo, you would need also nvidia-docker 1.0.

clone this repository to $YOUR_ROOT
```
cd $YOUR_ROOT
docker build -t parallel-baxter .
docker run -it --rm parallel-baxter
```
if you want to use vision in Gazebo, change the last line to 
```
nvidia-docker run -it --rm -e="DISPLAY" -v="/tmp/.X11-unix:/tmp/.X11-unix:rw" parallel-baxter
```
and in another terminal on your host machine, type
```
xhost +local:root
```
For more information on using vision in Gazebo inside docker, refer to [this](http://wiki.ros.org/docker/Tutorials/GUI)

now that you are inside the docker container, run the following to start gazebo and add baxter robot to it:
```
cd /home/ros_ws
source /opt/ros/kinetic/setup.bash
source /home/gazebo-ros/devel/setup.bash
source devel/setup.bash
chmod +x baxter.sh
./baxter.sh sim
roslaunch baxter_gazebo baxter_world.launch &  # this start Gazebo7 with nothing in it
roslaunch baxter_gazebo add_baxter.launch robot_ns:=/alice robot_dx:= 0.0 &    # this spawns a baxter with namespace /alice into Gazebo
roslaunch baxter_gazebo add_baxter.launch robot_ns:=/bob robot_dx:=5.0 &       # this spawns a baxter with namespace /bob into Gazebo, at position (5,0).
```

## Using baxter bridge
in a separate terminal, attach to the docker container shell.
```
python3
>>> from baxterClient import BaxterClient
>>> alice = BaxterClient('/alice', 'left')   # this makes a controller of Alice's left arm
>>> bob = BaxterClient('/bob', 'right')      # this makes a controller of Bob's right arm
```

Now in the terminal where you have executed "./baxter.sh sim"
```
cd /home/baxter_bridge
python baxterServerManager.py
```

When you go back to the python3 terminal, try
```
>>> alice.observe()
>>> alice.command([0,10,0,0,0,0,0])
>>> alice.command([0,0,0,0,0,0,0], absolute=True)
```

## Test ROS graph
this is useful tool if you want to modify the simulator package and want to test that ROS nodes and topics are working.
```
cd /home/test
python generateGraphFiles.py
python compareGraph.py > log.txt
grep "WARNING" log.txt
```

## Known issues:
IKservices are still under absolute namespace, using IKservice may crash or give unpredictable results.   
TF frames are not tested, these is no guarantee they would work properly.   
baxter_examples and baxter_tools are not modified to work with multiple robots.

You are welcome to report bugs, issues, and PRs. However I cannot guarantee timely response.
