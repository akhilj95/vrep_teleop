#!/usr/bin/env python

# run the ReplaySavedTrajectory.ttt file on Vrep before running this

import numpy as np
import glob
import errno
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float64


class Trajectory:
    trajectory_number = 0

    def __init__(self, filename):
        self.__class__.trajectory_number = self.__class__.trajectory_number + 1
        self.input_file = filename
        self.read_count = 0
        self.phase = None
        self.joint_positions = None
        self.joint_velocities = None
        self.box_positions = None
        self.box_orientations = None
        self.context = None
        self.read()

    def read(self):
        with open(self.input_file) as f_in:
            content = f_in.readlines()
            new_content = [line.strip().split(' ') for line in content]
            data_dict = {}

            for line_pie in new_content:
                feature_name = line_pie.pop(0)
                this_v = [float(v) for v in line_pie]
                if feature_name not in data_dict:
                    data_dict[feature_name] = []
                data_dict[feature_name].append(this_v)

        ts = np.array(data_dict['simTime'])[:, 0]
        ts = (ts - ts[0])
        self.read_count = len(ts)
        self.phase = ts / ts[self.read_count - 1]
        self.joint_positions = np.array(data_dict['currentPos'])
        self.joint_velocities = np.array(data_dict['currentVel'])
        self.box_positions = np.array(data_dict['boxPosition'])
        self.box_orientations = np.array(data_dict['boxOrientation'])


# function to play a trajectory in vrep given the trajectory joint positions and initial box data
def baxter_play(traj, box_data):
    pub1 = rospy.Publisher('/replay/joint_states', JointState, queue_size=10)
    rate_value = 250
    rate1 = rospy.Rate(rate_value)
    pub2 = rospy.Publisher('/stopSim', Float64, queue_size=1)
    rate2 = rospy.Rate(160)

    prev_pos = traj[0]

    for pos in traj:
        replay = JointState()
        replay.header = Header()
        replay.header.stamp = rospy.Time.now()
        replay.name = ['ljoint1', 'ljoint2', 'ljoint3', 'ljoint4', 'ljoint5', 'ljoint6', 'ljoint7', 'rjoint1',
                       'rjoint2', 'rjoint3', 'rjoint4', 'rjoint5', 'rjoint6', 'rjoint7']
        replay.position = pos
        replay.velocity = (pos - prev_pos) * rate_value
        replay.effort = box_data

        prev_pos = pos

        pub1.publish(replay)
        rate1.sleep()

    # for vrep to know when a trajectory ends and to keep the joints from moving randomly
    for i in range(0, 5):
        replay = JointState()
        replay.header = Header()
        replay.header.stamp = rospy.Time.now()
        replay.name = ['ljoint1', 'ljoint2', 'ljoint3', 'ljoint4', 'ljoint5', 'ljoint6', 'ljoint7', 'rjoint1',
                       'rjoint2', 'rjoint3', 'rjoint4', 'rjoint5', 'rjoint6', 'rjoint7']
        replay.position = pos
        replay.velocity = (pos - pos)  # to send 0s so that the joints don't go haywire
        replay.effort = []

        pub1.publish(replay)
        pub2.publish(data=1)
        rate2.sleep()


def plot(trajectory_list, joints_list):
        for j in joints_list:
            plt.figure(j * 2)
            plt.title('left_arm joint'+str(j+1))
            for i in range(0, len(trajectory_list)):
                y = trajectory_list[i].joint_positions[:, j]
                x = trajectory_list[i].phase
                plt.plot(x, y)
            plt.figure(j * 2 + 1)
            plt.title('right_arm joint' + str(j + 1))
            for i in range(0, len(trajectory_list)):
                y = trajectory_list[i].joint_positions[:, j + 7]
                x = trajectory_list[i].phase
                plt.plot(x, y)
        plt.show()


if __name__ == '__main__':
    rospy.init_node('trajectory_replay', anonymous=True)
    input_files = glob.glob('/home/akhil/Data/box_40_15_15/turnDemoSlave_c_5_t_*_r.txt')
    trajectory_list = []
    for name in input_files:
        try:
            trajectory_list.append(Trajectory(name))
        except IOError as exc:  # Not sure what error this is
            if exc.errno != errno.EISDIR:
                raise

    for i in range(0, len(trajectory_list)):
        baxter_play(trajectory_list[i].joint_positions,
                    np.concatenate((trajectory_list[i].box_positions[0], trajectory_list[i].box_orientations[0]), axis=0))

    joints_to_plot = [0, 1, 2, 3, 4, 5, 6]
    plot(trajectory_list, joints_to_plot)

    print('end')
