#!/usr/bin/env python

# run the baxterTeleopRecording.ttt file on Vrep before running this

import rospy
from baxter_core_msgs.msg import DigitalIOState
from sensor_msgs.msg import JointState
from vrep_teleop.msg import Joints
import numpy as np
import matplotlib.pyplot as plt


class RecordData:
    def __init__(self):
        self.left_button_pressed = 0
        self.left_cuff_pressed = 0
        self.right_cuff_pressed = 0
        self.record = 0
        self.start = 0
        self.end = 0
        self.n = 14
        self.z = 0
        self.currentPos = []
        self.targetPos = []
        self.newTargetPos = []
        self.errorValue = []
        self.targetVel = []
        self.currentVel = []
        self.newTargetVel = []
        self.effort = []
        self.ts = []
        self.f1 = open("/home/user/turnDemoMaster_1.txt", "w+")
        self.f2 = open("/home/user/turnDemoSlave_1.txt", "w+")

        rospy.Subscriber('robot/digital_io/left_lower_button/state', DigitalIOState, self.left_button)
        rospy.Subscriber('robot/digital_io/left_lower_cuff/state', DigitalIOState, self.left_cuff)
        rospy.Subscriber('robot/digital_io/right_lower_cuff/state', DigitalIOState, self.right_cuff)
        rospy.Subscriber("/robot/joint_states", JointState, self.master_state)
        rospy.Subscriber("/vrep/joints", Joints, self.slave_state)

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

            self.currentPos.extend(data.currentPos)
            self.targetPos.extend(data.targetPos)
            self.newTargetPos.extend(data.newTargetPos)
            self.errorValue.extend(data.errorValue)
            self.targetVel.extend(data.targetVel)
            self.currentVel.extend(data.currentVel)
            self.newTargetVel.extend(data.newTargetVel)
            self.effort.extend(data.effort)
            self.ts.extend(data.simTime)
            self.z = self.z + 1

            rospy.loginfo("master %s", data.seq)
            self.f2.write("seq ")
            self.f2.write("%d " % data.seq)
            self.f2.write("\n")
            self.f2.write("timeStamp ")
            self.f2.write("%f " % data.timeStamp)
            self.f2.write("\n")
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
            self.f2.write("targetVel ")
            for i in range(0, self.n):
                self.f2.write("%f " % (data.targetVel[i]))
            self.f2.write("\n")
            self.f2.write("currentVel ")
            for i in range(0, self.n):
                self.f2.write("%f " % (data.currentVel[i]))
            self.f2.write("\n")
            self.f2.write("newTargetVel ")
            for i in range(0, self.n):
                self.f2.write("%f " % (data.newTargetVel[i]))
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


if __name__ == '__main__':
    rospy.init_node('trajectory_listener', anonymous=True)
    try:
        startRecord = RecordData()
        while startRecord.end == 0:
            pass

        totalRead = startRecord.z
        currentPos = np.array(startRecord.currentPos).reshape((totalRead, startRecord.n))
        targetPos = np.array(startRecord.targetPos).reshape((totalRead, startRecord.n))
        newTargetPos = np.array(startRecord.newTargetPos).reshape((totalRead, startRecord.n))
        errorValue = np.array(startRecord.errorValue).reshape((totalRead, startRecord.n))
        targetVel = np.array(startRecord.targetVel).reshape((totalRead, startRecord.n))
        currentVel = np.array(startRecord.currentVel).reshape((totalRead, startRecord.n))
        newTargetVel = np.array(startRecord.newTargetVel).reshape((totalRead, startRecord.n))
        effort = np.array(startRecord.effort).reshape((totalRead, startRecord.n))
        ts = np.array(startRecord.ts).reshape((totalRead, startRecord.n))

        for i in range(0, startRecord.n):
            plt.figure(i+1)
            plt.figure(i+1).suptitle('Joint'+str(i+1))
            plt.subplot(311)
            plt.plot(ts[:, i]-ts[0, 0], targetPos[:, i], '.', label='Master')

            plt.subplot(311)
            plt.plot(ts[:, i]-ts[0, 0], newTargetPos[:, i], '. r', label='Master_Corrected')

            plt.subplot(311)
            plt.plot(ts[:, i]-ts[0, 0], currentPos[:, i], '. g', label='Slave')

            plt.legend()
            plt.xlabel('Time(in sec)')
            plt.ylabel('Joint Angles(in Radians)')

            plt.subplot(312)
            plt.plot(ts[:, i]-ts[0, 0], errorValue[:, i], '.')
            plt.xlabel('Time(in sec)')
            plt.ylabel('Position Error(in Radians)')

            plt.subplot(313)
            plt.plot(ts[:, i]-ts[0, 0], targetVel[:, i], '.', label='Master_Velocity')

            plt.subplot(313)
            plt.plot(ts[:, i]-ts[0, 0], newTargetVel[:, i], '. r', label='Master_Velocity_Corrected')

            plt.subplot(313)
            plt.plot(ts[:, i]-ts[0, 0], currentVel[:, i], '. g', label='Slave_Velocity')
            plt.legend()
            plt.xlabel('Time(in sec)')
            plt.ylabel('Joint Velocities(in Radians/sec)')
            # plt.figure(i+1).savefig("jn"+str(i+1)+".png")
        plt.show()

    except rospy.ROSInterruptException:
        pass
