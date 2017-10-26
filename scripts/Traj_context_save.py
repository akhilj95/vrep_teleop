#!/usr/bin/env python

import numpy as np
import glob
import errno
from scipy.linalg import block_diag
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
        self.box_positions = None
        self.box_orientations = None
        self.context = None
        self.read()
        self.basis_centres = None
        self.phi = None
        self.weights = None
        self.linear_basis()
        self.weights_func()

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
        self.box_positions = np.array(data_dict['boxPosition'])
        self.box_orientations = np.array(data_dict['boxOrientation'])
        context_start = np.array([self.box_positions[0, 0], self.box_positions[0, 1], self.box_orientations[0, 2]])
        context_end = np.array([self.box_positions[self.read_count - 1, 0], self.box_positions[self.read_count - 1, 1],
                                self.box_orientations[self.read_count - 1, 2]])
        self.context = np.concatenate((context_start, context_end), axis=0)

        # [np.savez('/home/akhil/Downloads/data/trajectory' + str(self.__class__.trajectory_number) + '.npz',
        #         phase=self.phase, joint_positions=self.joint_positions, box_positions=self.box_positions,
        #       box_orientations=self.box_orientations, context=self.context)

    def linear_basis(self):
        num_basis_inside = 11
        num_basis_outside = 4
        num_basis_tot = num_basis_inside + num_basis_outside
        basis_centre_gap = 1.0 / (num_basis_inside - 1)
        basis_width = 2 * basis_centre_gap ** 2
        self.basis_centres = np.linspace(-2*basis_centre_gap, 1+2*basis_centre_gap, num_basis_tot)
        self.phi = np.zeros((self.phase.shape[0], num_basis_tot))
        for z in range(0, self.phase.shape[0]):
            b = np.exp(-(1/basis_width)*(self.phase[z] - self.basis_centres)**2)
            sum_bz = np.sum(b)
            self.phi[z, :] = b/sum_bz

    def weights_func(self):
        self.weights = np.zeros((self.basis_centres.shape[0], 14))
        regularization_factor = 1e-10
        i_mat = np.identity(self.basis_centres.shape[0])
        self.weights = np.dot(np.linalg.inv(np.dot(self.phi.T, self.phi) + regularization_factor*i_mat),
                              np.dot(self.phi.T, self.joint_positions))


def calc_trajectory(weights, steps):
    num_basis_inside = 11
    num_basis_outside = 4
    num_basis_tot = num_basis_inside + num_basis_outside
    basis_centre_gap = 1.0 / (num_basis_inside - 1)
    basis_width = 2 * basis_centre_gap ** 2
    phase = np.linspace(0,1,steps)
    basis_centres = np.linspace(-2 * basis_centre_gap, 1 + 2 * basis_centre_gap, num_basis_tot)
    phi = np.zeros((steps, num_basis_tot))
    for z in range(0, steps):
        b = np.exp(-(1 / basis_width) * (phase[z] - basis_centres) ** 2)
        sum_bz = np.sum(b)
        phi[z, :] = b / sum_bz
    traj = np.dot(phi, weights)
    return traj


def baxter_play(traj, num, boxpos):
    pub1 = rospy.Publisher('/replay/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(100)

    for pos in traj:
        replay = JointState()
        replay.header = Header()
        replay.header.stamp = rospy.Time.now()
        replay.name = ['ljoint1', 'ljoint2', 'ljoint3', 'ljoint4', 'ljoint5', 'ljoint6', 'ljoint7', 'rjoint1',
                       'rjoint2', 'rjoint3', 'rjoint4', 'rjoint5', 'rjoint6', 'rjoint7']
        replay.position = pos
        replay.velocity = [num]
        replay.effort = boxpos

        pub1.publish(replay)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('trajectory_replay', anonymous=True)
    input_files = glob.glob('/home/akhil/Downloads/box_40_15_15/turnDemoSlave10.txt')
    trajectory_list = []
    for name in input_files:
        try:
            trajectory_list.append(Trajectory(name))
        except IOError as exc:  # Not sure what error this is
            if exc.errno != errno.EISDIR:
                raise

    for i in range(0, len(trajectory_list)):
        traj = calc_trajectory(trajectory_list[i].weights, 1500)
        baxter_play(traj, (i+1), np.concatenate((trajectory_list[i].box_positions[0],
                                                 trajectory_list[i].box_orientations[0]), axis=0))

    #print(block_diag(*[o.context for o in trajectory_list]))


