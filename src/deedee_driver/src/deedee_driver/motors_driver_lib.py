#! /usr/bin/env python

import rospy
import serial

class MotorsDriver:
    def __init__(self, robot_name, config):
        self.serial_port = config["serial_port"]
        self.max_wheel_speed = config["max_wheel_speed"]

        self.ser = serial.Serial(self.serial_port, timeout=1)

        rospy.loginfo("\MotorsDriver initialized for {}, awaiting path".format(robot_name))

    def map_speeds(self, speed_left_raw, speed_right_raw):
        speed_left = speed_left_raw / self.max_wheel_speed * 100
        speed_right = speed_right_raw / self.max_wheel_speed * 100

        speed_left = min(speed_left, 100)
        speed_right = min(speed_right, 100)

        return speed_left, speed_right

    def build_packet(self, speed_left, speed_right):
        packet = "$0,%i,%i\n" % (speed_left, speed_right)
        return packet

    def send_speeds(self, speed_left_raw, speed_right_raw):
        speed_left, speed_right = self.map_speeds(speed_left_raw,
                                                  speed_right_raw)
        packet = self.build_packet(speed_left, speed_right)
        self.ser.write(packet)

