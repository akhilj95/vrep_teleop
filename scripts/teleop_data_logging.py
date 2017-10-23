#!/usr/bin/env python

import rospy
from baxter_core_msgs.msg import DigitalIOState
from sensor_msgs.msg import JointState
from vrep_teleop.msg import Joints
import numpy as np
import matplotlib.pyplot as plt


class RecordData:
    def left_button(self, data):
        if data.state == 1:
            self.left_button_pressed = 1
            self.start = 1
        else:
            self.left_button_pressed = 0

        if self.start == 1 and self.record == 1:
            self.record = 0
            self.end = 1

    def left_cuff(self, data):
        if data.state == 1:
            self.left_cuff_pressed = 1
        else:
            self.left_cuff_pressed = 0

    def right_cuff(self, data):
        if data.state == 1:
            self.right_cuff_pressed = 1
        else:
            self.right_cuff_pressed = 0

        if (self.left_cuff_pressed == 1 or self.right_cuff_pressed == 1) and self.start == 1:
            self.record = 1
            self.start = 0

    def master_state(self, data):
        if self.record == 1:
            self.f1.write("%s \n" % data)
        elif self.end == 1:
            self.f1.close()

    def slave_state(self, data):

        if self.record == 1:

            self.current.extend(data.currentPos)
            self.target.extend(data.targetPos)
            self.newTarget.extend(data.newTargetPos)
            self.error.extend(data.errorValue)
            self.inputVel.extend(data.vs)
            self.compVel.extend(data.vel)
            self.maxVel.extend(data.maxVelocity)
            self.effort.extend(data.effort)
            self.ts.extend(data.simTime)
            self.z = self.z + 1

            rospy.loginfo("master %s", data.stamp)
            self.f2.write("currentPos ")
            for i in range(0, self.n):
                self.f2.write("%f " % (data.currentPos[i]))
            self.f2.write("\n")
            self.f2.write("targetPos ")
            for i in range(0, self.n):
                self.f2.write("%f " % (data.targetPos[i]))
            self.f2.write("\n")
            self.f2.write("newTargetPos ")
            for i in range(0, self.n):
                self.f2.write("%f " % (data.newTargetPos[i]))
            self.f2.write("\n")
            self.f2.write("errorValue ")
            for i in range(0, self.n):
                self.f2.write("%f " % (data.errorValue[i]))
            self.f2.write("\n")
            self.f2.write("vs ")
            for i in range(0, self.n):
                self.f2.write("%f " % (data.vs[i]))
            self.f2.write("\n")
            self.f2.write("vel ")
            for i in range(0, self.n):
                self.f2.write("%f " % (data.vel[i]))
            self.f2.write("\n")
            self.f2.write("requiredVel ")
            for i in range(0, self.n):
                self.f2.write("%f " % (data.maxVelocity[i]))
            self.f2.write("\n")
            self.f2.write("effort ")
            for i in range(0, self.n):
                self.f2.write("%f " % (data.effort[i]))
            self.f2.write("\n")
            self.f2.write("simTime ")
            for i in range(0, self.n):
                self.f2.write("%f " % (data.simTime[i]))
            self.f2.write("\n")
            self.f2.write("boxPosition ")
            for i in range(0, 3):
                self.f2.write("%f " % (data.boxPosition[i]))
            self.f2.write("\n")
            self.f2.write("boxOrientation ")
            for i in range(0, 3):
                self.f2.write("%f " % (data.boxOrientation[i]))
            self.f2.write("\n")

        elif self.end == 1:
            self.f2.close()

    def __init__(self):
        self.left_button_pressed = 0
        self.left_cuff_pressed = 0
        self.right_cuff_pressed = 0
        self.record = 0
        self.start = 0
        self.end = 0
        self.n = 14
        self.z = 0
        self.current = []
        self.target = []
        self.newTarget = []
        self.error = []
        self.inputVel = []
        self.compVel = []
        self.maxVel = []
        self.effort = []
        self.ts = []
        self.f1 = open("/home/akhil/Downloads/box_40_15_15/turnDemoMaster100txt", "w+")
        self.f2 = open("/home/akhil/Downloads/box_40_15_15/turnDemoSlave100.txt", "w+")

        rospy.Subscriber('robot/digital_io/left_lower_button/state', DigitalIOState, self.left_button)
        rospy.Subscriber('robot/digital_io/left_lower_cuff/state', DigitalIOState, self.left_cuff)
        rospy.Subscriber('robot/digital_io/right_lower_cuff/state', DigitalIOState, self.right_cuff)
        rospy.Subscriber("/robot/joint_states", JointState, self.master_state)
        rospy.Subscriber("/vrep/joints", Joints, self.slave_state)


if __name__ == '__main__':
    rospy.init_node('trajectory_listener', anonymous=True)
    try:
        startRecord = RecordData()
        while startRecord.end == 0:
            pass

        jn = 0
        totalRead = startRecord.z
        currentPos = np.array(startRecord.current).reshape((totalRead, startRecord.n))
        targetPos = np.array(startRecord.target).reshape((totalRead, startRecord.n))
        newTargetPos = np.array(startRecord.newTarget).reshape((totalRead, startRecord.n))
        errorValue = np.array(startRecord.error).reshape((totalRead, startRecord.n))
        vs = np.array(startRecord.inputVel).reshape((totalRead, startRecord.n))
        vel = np.array(startRecord.compVel).reshape((totalRead, startRecord.n))
        maxVelocity = np.array(startRecord.maxVel).reshape((totalRead, startRecord.n))
        effort = np.array(startRecord.effort).reshape((totalRead, startRecord.n))
        ts = np.array(startRecord.ts).reshape((totalRead, startRecord.n))

        for i in range(0, startRecord.n-14):
            plt.figure(i+1)
            plt.figure(i+1).suptitle('Simulator:(moving)  points=5 order=3 lambda=10^-6  PGain=250 VGain=60')
            plt.subplot(511)
            plt.plot(ts[:, i], targetPos[:, i], '.')

            plt.subplot(511)
            plt.plot(ts[:, i], newTargetPos[:, i], 'x r')

            plt.subplot(511)
            plt.plot(ts[:, i], currentPos[:, i], '. g')

            plt.xlabel('masterPos(b), newTarget(r), SlavePos(g)')

            plt.subplot(513)
            plt.plot(ts[:, i], errorValue[:, i], '.')
            plt.xlabel('errorValue')

            plt.subplot(515)
            plt.plot(ts[:, i], vs[:, i], '.')

            plt.subplot(515)
            plt.plot(ts[:, i], maxVelocity[:, i], 'x r')

            plt.subplot(515)
            plt.plot(ts[:, i], vel[:, i], '. g')

            plt.xlabel('masterVel(b), newTargetVel(r), slaveVel(g)')
            #plt.figure(i+1).savefig("sim_moving_5_3_10m6_250_60_jn"+str(i+1)+".png")
        plt.show()

    except rospy.ROSInterruptException:
        pass
