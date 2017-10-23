#!/usr/bin/env python

import rospy
from baxter_core_msgs.msg import DigitalIOState
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float64
from vrep_teleop.msg import Joints
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


start = 0


def vrepstart(data):
    global start
    start = 1


def baxter(pos, vel, eff, tot, boxdata):
    pub1 = rospy.Publisher('/replay/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(140)
    i = 0

    while not rospy.is_shutdown():
        replay = JointState()
        replay.header = Header()
        replay.header.stamp = rospy.Time.now()
        replay.name = ['ljoint1', 'ljoint2', 'ljoint3', 'ljoint4', 'ljoint5', 'ljoint6', 'ljoint7', 'rjoint1',
                       'rjoint2', 'rjoint3', 'rjoint4', ]
        replay.position = [pos[i][0], pos[i][1], pos[i][2], pos[i][3], pos[i][4], pos[i][5], pos[i][6], pos[i][7],
                           pos[i][8], pos[i][9], pos[i][10], pos[i][11], pos[i][12], pos[i][13]]
        replay.velocity = [vel[i][0], vel[i][1], vel[i][2], vel[i][3], vel[i][4], vel[i][5], vel[i][6], vel[i][7],
                           vel[i][8], vel[i][9], vel[i][10], vel[i][11], vel[i][12], vel[i][13]]
        replay.effort = boxdata
        if i < tot - 2 and start == 1:
            i = i + 1
        else:
            replay.velocity = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        pub1.publish(replay)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('trajectory_replay', anonymous=True)
    try:
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

        rospy.Subscriber('replay/start', Float64, vrepstart)

        baxter(currentPos, vel, effort, totalRead, np.concatenate((boxPos[0], boxOrientation[0]), axis=0))

    finally:
        pass
