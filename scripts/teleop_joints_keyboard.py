#! /usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (c) 2013 PAL Robotics SL.
# Released under the BSD License.
#
# Authors:
#   * Siegfried-A. Gevatter

import curses
import math

import rospy
#from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class TextWindow():

    _screen = None
    _window = None
    _num_lines = None

    def __init__(self, stdscr, lines=10):
        self._screen = stdscr
        self._screen.nodelay(True)
        curses.curs_set(0)

        self._num_lines = lines

    def read_key(self):
        keycode = self._screen.getch()
        return keycode if keycode != -1 else None

    def clear(self):
        self._screen.clear()

    def write_line(self, lineno, message):
        if lineno < 0 or lineno >= self._num_lines:
            raise ValueError, 'lineno out of bounds'
        height, width = self._screen.getmaxyx()
        y = (height / self._num_lines) * lineno
        x = 10
        for text in message.split('\n'):
            text = text.ljust(width)
            self._screen.addstr(y, x, text)
            y += 1

    def refresh(self):
        self._screen.refresh()

    def beep(self):
        curses.flash()



class SimpleKeyTeleop():
    def __init__(self, interface):
        self._interface = interface
        self._pub_cmd = rospy.Publisher('key_vel', Float64MultiArray)
	#queue_size=10
        self._hz = rospy.get_param('~hz',250)

        self._last_pressed = {}
        self._angles = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,100]
	

    movement_bindings = {

	101:		  (0.0,0.005,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0),
	114:		  (0.0,-0.005,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0),

	116:		  (0.0,0.0,0.005,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0),
	121:		  (0.0,0.0,-0.005,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0),

	117:		  (0.0,0.0,0.0,0.005,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0),
	105:		  (0.0,0.0,0.0,-0.005,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0),

	111:		  (0.0,0.0,0.0,0.0,0.005,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0),
	112:		  (0.0,0.0,0.0,0.0,-0.005,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0),

	97:		  (0.0,0.0,0.0,0.0,0.0,0.005,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0),
	115:		  (0.0,0.0,0.0,0.0,0.0,-0.005,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0),

	100:		  (0.0,0.0,0.0,0.0,0.0,0.0,0.005,0.0,0.0,0.0,0.0,0.0,0.0,0.0),
	102:		  (0.0,0.0,0.0,0.0,0.0,0.0,-0.005,0.0,0.0,0.0,0.0,0.0,0.0,0.0),

	103:		  (0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.005,0.0,0.0,0.0,0.0,0.0),
	104:		  (0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.005,0.0,0.0,0.0,0.0,0.0),

	109:		  (0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0),

        curses.KEY_UP:    (0.005,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0),
        curses.KEY_LEFT:  (0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.005,0.0,0.0,0.0,0.0,0.0,0.0),
        curses.KEY_RIGHT:  (0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.005,0.0,0.0,0.0,0.0,0.0,0.0),
        curses.KEY_DOWN: (-0.005,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0),
    }

    def run(self):
        rate = rospy.Rate(self._hz)
        self._running = True
        while self._running:
            while True:
                keycode = self._interface.read_key()
                if keycode is None:
                    break
                self._key_pressed(keycode)
            self._append()
            self._publish()
            rate.sleep()

    def _get_joints(self, angles):
        joints = Float64MultiArray()
        joints.data = angles
        return joints

    def _append(self):
        now = rospy.get_time()
        keys = []
	angles = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
	tempangles = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        for a in self._last_pressed:
            if now - self._last_pressed[a] < 0.001:
                keys.append(a)
        for k in keys:
		tempangles=self.movement_bindings[k];
		for i in range(0,13):
			angles[i] =angles[i]+tempangles[i]
	for i in range(0,13):
        	self._angles[i] = self._angles[i] + angles[i]

    def _key_pressed(self, keycode):
        if keycode == ord('q'):
            self._running = False
            rospy.signal_shutdown('Bye')
        elif keycode in self.movement_bindings:
            self._last_pressed[keycode] = rospy.get_time()

    def _publish(self):
        self._interface.clear()
        self._interface.write_line(2, 'Check text')
        self._interface.write_line(5, 'Use arrow keys to move, q to exit.')
        self._interface.refresh()
	self._angles[14]=rospy.get_time()
        joints = self._get_joints(self._angles)
        self._pub_cmd.publish(joints)


def main(stdscr):
    rospy.init_node('key_teleop')
    app = SimpleKeyTeleop(TextWindow(stdscr))
    app.run()

if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except rospy.ROSInterruptException:
        pass
