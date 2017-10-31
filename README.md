# Vrep_RosInterface_teleop
V-rep and ros used to create teleoperation system with Baxter.

## Prerequisites
* [ROS Kinetic](http://wiki.ros.org/kinetic/Installation)
* [V-REP](http://www.coppeliarobotics.com/downloads.html)
* V-REP RosInterface
Guide to setup these are given [here](https://github.com/akhilj95/vrep_teleop/wiki/RosInterface-Setup)

## To setup
Clone this project in the same catkin workspace as used for the RosInterface. Open _meta/messages.txt_ of vrep_ros_interface package and add

    vrep_teleop/Joints
In _CMakeLists.txt_ of vrep_ros_interface package add 'vrep_teleop' to the list of package names in catkin_package(). Open _package.xml_, and add these two lines:

    <build_depend>vrep_teleop</build_depend>
    <run_depend>vrep_teleop</run_depend>
Rebuild your catkin workspace and copy the library file _libv_repExtRosInterface.so_ in _devel/lib_ folder to your V-REP installation folder.
