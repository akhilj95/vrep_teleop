#!/usr/bin/env python

import numpy as np
import glob
import errno


class Trajectory:
    trajectory_number = 0

    def __init__(self, filename):
        self.__class__.trajectory_number = self.__class__.trajectory_number + 1
        self.input_file = filename
        self.joint_positions = None
        self.z = None
        self.read_count = 0
        self.box_position = None
        self.box_orientation = None
        self.context = {'start': [], 'end': []}
        self.read()

    def read(self):
        with open(self.input_file) as f_in:
            cont = f_in.readlines()
            new_cont = [line.strip().split(' ') for line in cont]
            rst_d = {}

            for line_pie in new_cont:
                fea_n = line_pie.pop(0)
                this_v = [float(v) for v in line_pie]
                if fea_n not in rst_d:
                    rst_d[fea_n] = []
                rst_d[fea_n].append(this_v)

        ts = np.array(rst_d['simTime'])[:, 0]
        ts = (ts-ts[0])
        self.read_count = len(ts)-1
        self.z = ts/ts[self.read_count]
        self.joint_positions = np.array(rst_d['currentPos'])
        self.box_position = np.array(rst_d['boxPosition'])
        self.box_orientation = np.array(rst_d['boxOrientation'])
        self.context['start'] = [self.box_position[0, 0], self.box_position[0, 1], self.box_orientation[0, 2]]
        self.context['end'] = [self.box_position[self.read_count, 0], self.box_position[self.read_count, 1],
                               self.box_orientation[self.read_count, 2]]
        np.savez('/home/akhil/Downloads/data/trajectory' + str(self.__class__.trajectory_number) + '.npz', z=self.z,
                 joint_positions=self.joint_positions, box_position=self.box_position, box_orientation=self.box_orientation)

    def basis(self):
        num_basis_inside = 100
        num_basis_ends = 4
        num_basis_tot = num_basis_inside + num_basis_ends
        basis_centre_dist = 1 / num_basis_inside
        basis_width = 2 * basis_centre_dist ** 2
        basis_centres = np.linspace(-2*basis_width, 1+2*basis_width, num_basis_tot)


if __name__ == '__main__':
        input_files = glob.glob('/home/akhil/Downloads/box_40_15_15/turnDemoSlave10.txt')
        trajectory_list = []
        for name in input_files:
            try:
                trajectory_list.append(Trajectory(name))
                print(trajectory_list[0].read_count)
            except IOError as exc:  # Not sure what error this is
                if exc.errno != errno.EISDIR:
                    raise

