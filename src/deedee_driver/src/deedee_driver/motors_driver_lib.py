#! /usr/bin/env python

import rospy

class MotorsDriver:
    def __init__(self, robot_name, config, serial_socket):
        self.max_wheel_speed = config["max_wheel_speed"]

        self.ser = serial_socket

        rospy.loginfo("MotorsDriver initialized for {}, awaiting path".format(robot_name))

    def map_speed(self, speed_raw):
        speed = speed_raw / self.max_wheel_speed * 100

        if speed == 0:
            speed = 0
        else:
            speed = speed / abs(speed) * min(abs(speed), 100)

        return speed

    def build_packet(self, speed_left, speed_right):
        packet = "$0,%i,%i\n" % (speed_left, speed_right)
        return packet

    def send_speeds(self, speed_left_raw, speed_right_raw):
        speed_left = self.map_speed(speed_left_raw)
        speed_right = self.map_speed(speed_right_raw)
        packet = self.build_packet(speed_left, speed_right)
        self.ser.write(packet)

