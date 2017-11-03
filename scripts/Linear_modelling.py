#!/usr/bin/env python

import numpy as np
import glob
import errno
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


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
        self.phase = ts / ts[self.read_count - 1]  # scaling the time to [0,1]
        self.joint_positions = np.array(data_dict['currentPos'])
        self.box_positions = np.array(data_dict['boxPosition'])
        self.box_orientations = np.array(data_dict['boxOrientation'])
        context_start = np.array([self.box_positions[0, 0], self.box_positions[0, 1], self.box_orientations[0, 1]])
        context_end = np.array([self.box_positions[self.read_count - 1, 0], self.box_positions[self.read_count - 1, 1],
                                self.box_orientations[self.read_count - 1, 1]])
        # context contains initial and final x,y,angle
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


# function to calculate a trajectory given the weight vectors
def calc_trajectory(weights, steps):
    num_basis_tot = weights.shape[0]
    num_basis_outside = 4
    num_basis_inside = num_basis_tot - num_basis_outside
    basis_centre_gap = 1.0 / (num_basis_inside - 1)
    basis_width = 2 * basis_centre_gap ** 2
    phase = np.linspace(0, 1, steps)
    basis_centres = np.linspace(-2 * basis_centre_gap, 1 + 2 * basis_centre_gap, num_basis_tot)
    phi = np.zeros((steps, num_basis_tot))
    for z in range(0, steps):
        b = np.exp(-(1 / basis_width) * (phase[z] - basis_centres) ** 2)
        sum_bz = np.sum(b)
        phi[z, :] = b / sum_bz
    traj = np.dot(phi, weights)
    return traj


# function to play a trajectory in vrep given the trajectory joint positions and initial box data
def baxter_play(traj,  box_data):
    pub1 = rospy.Publisher('/replay/joint_states', JointState, queue_size=10)
    rate_value = 200
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

        pub1.publish(replay)
        rate1.sleep()
        prev_pos = pos

    # for vrep to know when a trajectory ends and to keep the joints from moving randomly
    for i in range(0, 5):
        replay = JointState()
        replay.header = Header()
        replay.header.stamp = rospy.Time.now()
        replay.name = []
        replay.position = pos
        replay.velocity = (pos-pos)  # to send 0s so that the joints don't go haywire
        replay.effort = []

        pub1.publish(replay)
        pub2.publish(data=1)
        rate2.sleep()


class ContextResult:
    def __init__(self, trajectory_list, context_indices):
        self.trajectory_list = trajectory_list
        self.trajectories_tot = len(trajectory_list)
        # self.plot([4,5,6])
        self.context_indices = context_indices
        self.context_feature_matrix = None
        self.k = self.find_transformation_parameters()
        self.final_box_data = None
        rospy.Subscriber('boxData', Float64MultiArray, self.replay_box_data)

    def find_transformation_parameters(self):
        num_contexts = len(self.context_indices)
        num_basis = self.trajectory_list[0].basis_centres.shape[0]
        self.context_feature_matrix = np.ones((self.trajectories_tot, num_contexts + 1))
        concatenated_weights = np.zeros((self.trajectories_tot, num_basis * 14))

        for i in range(0, self.trajectories_tot):
            # saving the context feature vectors of a trajectory as rows
            for p in range(0, num_contexts):
                self.context_feature_matrix[i, 1 + p] = self.trajectory_list[i].context[self.context_indices[p]]
            # concatenating weights for each joint of a trajectory in a row
            for j in range(0, 14):
                concatenated_weights[i, num_basis * j:num_basis * (j + 1)] = self.trajectory_list[i].weights[:, j]

        regularization_factor = 1e-10
        i_mat = np.identity(self.context_feature_matrix.shape[1])

        k = np.dot(np.linalg.inv(np.dot(self.context_feature_matrix.T, self.context_feature_matrix)
                                 + regularization_factor * i_mat),
                   np.dot(self.context_feature_matrix.T, concatenated_weights))
        return k

    def plot(self, joints):
        for j in joints:
            plt.figure(j * 2)
            plt.title('left_arm joint' + str(j + 1))
            for i in range(0, self.trajectories_tot):
                y = self.trajectory_list[i].joint_positions[:, j]
                x = self.trajectory_list[i].phase
                plt.plot(x, y, label=str(i))
                plt.legend()
            plt.figure(j * 2 + 1)
            plt.title('right_arm joint' + str(j + 1))
            for i in range(0, self.trajectories_tot):
                y = self.trajectory_list[i].joint_positions[:, j + 7]
                x = self.trajectory_list[i].phase
                plt.plot(x, y, label=str(i))
                plt.legend()
        plt.show()

    def replay_box_data(self, msg):
        # print(msg.data)
        self.final_box_data = msg.data

    def find_linear_errors(self, num_divisions):
        num_contexts = len(self.context_indices)
        contexts_min = np.amin(self.context_feature_matrix, axis=0)
        contexts_max = np.amax(self.context_feature_matrix, axis=0)
        task_linear_features = np.ones((num_divisions, self.context_feature_matrix.shape[1]))
        errors = np.zeros((num_divisions, 3))

        for p in range(0, num_contexts):
            # making task context features from min to max
            task_linear_features[:, p + 1] = np.linspace(contexts_min[p + 1], contexts_max[p + 1], num_divisions)

        box_data_default = np.concatenate((self.trajectory_list[0].box_positions[0],
                                           self.trajectory_list[0].box_orientations[0]), axis=0)

        for i in range(0, num_divisions):
            temp_result = np.dot(self.k.T, task_linear_features[i])
            resulting_weights = np.reshape(temp_result, (14, 15)).T
            traj = calc_trajectory(resulting_weights, 1500)
            box_data = box_data_default

            # changing the initial box position if the context says so
            for p in range(0, num_contexts):
                if self.context_indices[p] < 2:
                    box_data[p] = task_linear_features[i, p+1]
                elif self.context_indices[p] == 2:
                    box_data[4] = task_linear_features[i, p+1]

            baxter_play(traj, box_data)

            # finding the errors in the final position
            for p in range(0, num_contexts):
                if self.context_indices[p] == 3 or self.context_indices[p] == 4:
                    errors[i, self.context_indices[p]-3] = self.final_box_data[self.context_indices[p]-3] \
                                                           - task_linear_features[i, p+1]
                elif self.context_indices[p] == 5:
                    errors[i, self.context_indices[p]-3] = self.final_box_data[4] - task_linear_features[i, p+1]

        plt.figure(100)
        if self.context_indices[0] == 5:
            plt.plot((task_linear_features[:, 1] - box_data[4])*180/3.14, errors[:, 2]*180/3.14, 'o')
            plt.xlabel('rotation angle(in deg)')
            plt.ylabel('error in angle(in deg)')
        if self.context_indices[0] == 4:
            plt.plot((task_linear_features[:, 1] - box_data[1])*100, errors[:, 1]*100, 'o')
            plt.xlabel('change in box position(in cms)')
            plt.ylabel('error in box position(in cms)')
        if self.context_indices[0] == 3:
            plt.plot((task_linear_features[:, 0] - box_data[0])*100, errors[:, 1]*100, 'o')
            plt.xlabel('change in box position(in cms)')
            plt.ylabel('error in box position(in cms)')
        plt.show()


if __name__ == '__main__':
    rospy.init_node('trajectory_replay', anonymous=True)
    # Enter the file path of the recorded data
    input_files = glob.glob('/home/akhil/SampleData/turnDemoSlave_*.txt')
    trajectory_list = []
    for name in input_files:
        try:
            trajectory_list.append(Trajectory(name))
        except IOError as exc:  # Not sure what error this is
            if exc.errno != errno.EISDIR:
                raise

    relevant_context_indices = [4]
    # the contexts that can be used are initial x,y,angle final x,y,angle in order 0-5
    calc = ContextResult(trajectory_list, relevant_context_indices)
    num_linear_goal_positions = 5
    calc.find_linear_errors(num_linear_goal_positions)
    # joints_to_plot = [0, 1, 2, 3, 4, 5, 6]
    # calc.plot(joints_to_plot)
    print('end')
