# Vrep_RosInterface_teleop
V-rep and ros used to create teleoperation system with Baxter.

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

## To use
* Start baxter and from your Baxter Workspace run
>       ./baxter.sh
* Enable the baxter:
>       rosrun baxter_tools enable_robot.py -e
>       rosrun baxter_tools tuck_arms.py -u
* start V-REP and open _vrep/scenes/baxterTeleopRecording.ttt_
* run the python script _script/teleop_data_logging.py_ if you wish to record the data
