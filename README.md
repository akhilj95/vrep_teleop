## This code is currently unusable due to changes in V-REP 3.5. Will make the necessary changes soon. Sorry for the inconvenience.

# Vrep_RosInterface_teleop
V-rep and ROS used to create a teleoperation system with Baxter to learn from demonstrations. Here is a small [video clip](https://youtu.be/53QvpNsICyA) as an example.

## Prerequisites
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation)
* [V-REP](http://www.coppeliarobotics.com/downloads.html)
* V-REP RosInterface

Guide to install and setup these are given [here](https://github.com/akhilj95/vrep_teleop/wiki/RosInterface-Setup)

## To setup
* Clone this project in the same catkin workspace as used for the RosInterface.
* Open _meta/messages.txt_ of vrep_ros_interface package and add
>       vrep_teleop/Joints
* In _CMakeLists.txt_ of vrep_ros_interface package add 'vrep_teleop' to the list of package names in catkin_package().
* Open _package.xml_, and add these two lines:
>       <build_depend>vrep_teleop</build_depend>
>       <run_depend>vrep_teleop</run_depend>    
* Rebuild your catkin workspace using
>       $ catkin build
* Copy the library file _libv_repExtRosInterface.so_ in _devel/lib_ folder to your V-REP installation folder.
>       $ cp -iv devel/lib/libv_repExtRosInterface.so "$VREP_ROOT/"
* Copy the _matrix.lau_ file in _vrep/lua_ to the _lua_ folder of your V-REP installation folder.
>       $ cp -iv src/vrep_teleop/vrep/lua/matrix.lua "$VREP_ROOT/lua/"

## To use teleoperation and record
1) Start baxter and from your Baxter Workspace run
>       ./baxter.sh
2) Enable the baxter:
>       rosrun baxter_tools enable_robot.py -e
>       rosrun baxter_tools tuck_arms.py -u
3) Open another terminal repeat step 1. Then start V-REP and open _vrep/scenes/baxterTeleopRecording.ttt_
4) Open another terminal repeat step 1. Then run the python script _script/teleop_data_logging.py_ if you wish to record the data


## Other
* Some sample data are given in _vrep/SampleData/_.
* Scripts for using the saved data are there in _script/_ folder.
* Read the instruction at the beginning of the scripts.
