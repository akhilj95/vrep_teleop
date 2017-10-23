#!/usr/bin/env python

import rospy
from baxter_core_msgs.msg import DigitalIOState
from sensor_msgs.msg import JointState
from simple_check.msg import Joints
import numpy as np
import matplotlib.pyplot as plt
from pprint import pprint


class readData:
    def read(self):
        with open('/home/akhil/Downloads/box_40_15_15/turnDemoSlave39.txt') as f:
            for line in f:
                if self.z % 11 == 1:
                    nums = line.split()  # split the line into a list of strings by whitespace
                    nums = map(float, nums[1:])  # turn each string into a float
                    self.current.extend(nums)
                elif self.z % 11 == 2:
                    nums = line.split()  # split the line into a list of strings by whitespace
                    nums = map(float, nums[1:])  # turn each string into a float
                    self.target.extend(nums)

                elif self.z % 11 == 3:
                    nums = line.split()  # split the line into a list of strings by whitespace
                    nums = map(float, nums[1:])  # turn each string into a float
                    self.newTarget.extend(nums)

                elif self.z % 11 == 4:
                    nums = line.split()  # split the line into a list of strings by whitespace
                    nums = map(float, nums[1:])  # turn each string into a float
                    self.error.extend(nums)

                elif self.z % 11 == 5:
                    nums = line.split()  # split the line into a list of strings by whitespace
                    nums = map(float, nums[1:])  # turn each string into a float
                    self.inputVel.extend(nums)

                elif self.z % 11 == 6:
                    nums = line.split()  # split the line into a list of strings by whitespace
                    nums = map(float, nums[1:])  # turn each string into a float
                    self.compVel.extend(nums)

                elif self.z % 11 == 7:
                    nums = line.split()  # split the line into a list of strings by whitespace
                    nums = map(float, nums[1:])  # turn each string into a float
                    self.maxVel.extend(nums)

                elif self.z % 11 == 8:
                    nums = line.split()  # split the line into a list of strings by whitespace
                    nums = map(float, nums[1:])  # turn each string into a float
                    self.effort.extend(nums)

                elif self.z % 11 == 9:
                    nums = line.split()  # split the line into a list of strings by whitespace
                    nums = map(float, nums[1:])  # turn each string into a float
                    self.ts.extend(nums)

                elif self.z % 11 == 10:
                    nums = line.split()  # split the line into a list of strings by whitespace
                    nums = map(float, nums[1:])  # turn each string into a float
                    self.boxPos.extend(nums)

                elif self.z % 11 == 0:
                    nums = line.split()  # split the line into a list of strings by whitespace
                    nums = map(float, nums[1:])  # turn each string into a float
                    self.boxOrientation.extend(nums)

                self.z = self.z + 1

    def __init__(self):
        self.left_cuff_pressed = 0
        self.right_cuff_pressed = 0
        self.record = 0
        self.end = 0
        self.n = 14
        self.z = 1
        self.current = []
        self.target = []
        self.newTarget = []
        self.error = []
        self.inputVel = []
        self.compVel = []
        self.maxVel = []
        self.effort = []
        self.ts = []
        self.boxPos = []
        self.boxOrientation = []

if __name__ == '__main__':
    startRead = readData()
    startRead.read()

    totalRead = (startRead.z - 1) / 11
    currentPos = np.array(startRead.current).reshape((totalRead, startRead.n))
    targetPos = np.array(startRead.target).reshape((totalRead, startRead.n))
    newTargetPos = np.array(startRead.newTarget).reshape((totalRead, startRead.n))
    errorValue = np.array(startRead.error).reshape((totalRead, startRead.n))
    vs = np.array(startRead.inputVel).reshape((totalRead, startRead.n))
    vel = np.array(startRead.compVel).reshape((totalRead, startRead.n))
    maxVelocity = np.array(startRead.maxVel).reshape((totalRead, startRead.n))
    effort = np.array(startRead.effort).reshape((totalRead, startRead.n))
    ts = np.array(startRead.ts).reshape((totalRead, startRead.n))
    boxPos = np.array(startRead.boxPos).reshape((totalRead, 3))
    boxOrientation = np.array(startRead.boxOrientation).reshape((totalRead, 3))
    print(boxPos)

    plt.figure(1)
    plt.title('x-y plane')
    plt.xlim(-0.4, 0.4)
    plt.ylim(-0.2, 0.2)
    plt.plot(boxPos[:, 1]-boxPos[0, 1], boxPos[0, 0]-boxPos[:, 0], '.')
    plt.xlabel('Position(in m)')
    plt.ylabel('Position(in m)')
    plt.figure(2)
    for i in range(2, 3):
    #    plt.subplot(131+i)
        plt.plot(ts[:, i], boxOrientation[:, i], '.')
        plt.ylabel('Angle(in radians)')
        plt.xlabel('time(in sec)')
    plt.show()
