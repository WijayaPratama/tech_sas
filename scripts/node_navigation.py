#!/usr/bin/env python3

import rospy
from std_msgs.msg import Bool, String
from robotic_sas_auv_ros.msg import Sensor, SetPoint, Error

class Subscriber():
    def __init__(self):
        self.error = Error()
        self.set_point = SetPoint()

        self.is_stable_roll = False
        self.is_stable_pitch = False
        self.is_stable_yaw = False
        self.is_stable_depth = False

        # Publisher
        self.pub_error = rospy.Publisher('error', Error, queue_size=10)
        self.pub_movement = rospy.Publisher('movement', String, queue_size=10)

        # Subscriber
        # subscribe x object
        # subscribe y object

        rospy.Subscriber('set_point', SetPoint, self.callback_set_point)
        rospy.Subscriber('sensor', Sensor, self.callback_sensor)
        rospy.Subscriber('is_start', Bool, self.callback_is_start)

    def reset_error(self):
        self.error.roll = 0
        self.error.pitch = 0
        self.error.yaw = 0
        self.error.depth = 0

    # callback x hitung error x 

    def calculate_compass_error(self, current, target):
        return (target - current + 180) % 360 - 180

    def callback_set_point(self, data):
        self.set_point = data

    def callback_sensor(self, data):
        error_roll = self.set_point.roll + data.roll
        error_pitch = self.set_point.pitch - data.pitch
        error_yaw = self.calculate_compass_error(data.yaw, self.set_point.yaw)
        error_depth = self.set_point.depth - data.depth

        self.is_stable_roll = -2 <= error_roll <= 2
        self.is_stable_pitch = -8 <= error_pitch <= 8
        self.is_stable_yaw = -3 <= error_yaw <= 3
        self.is_stable_depth = -0.03 <= error_depth <= 0.03

        #RIFQI
        # self.is_stable_roll = -5 <= error_roll <= 5
        # self.is_stable_pitch = -5 <= error_pitch <= 5
        # self.is_stable_yaw = -3 <= error_yaw <= 3
        # self.is_stable_depth = -0.03 <= error_depth <= 0.03

        # self.error.roll = 0 if -3 <= error_roll <= 3 else error_roll
        # self.error.pitch = 0 if -3 <= error_pitch <= 3 else error_pitch
        # self.error.yaw = 0 if -3 <= error_yaw <= 3 else error_yaw
        # self.error.depth = error_depth

        #RIFQI
        self.error.roll = 0 if -3 <= error_roll <= 3 else error_roll
        self.error.pitch = 0 if -3 <= error_pitch <= 3 else error_pitch
        self.error.yaw = 0 if -3 <= error_yaw <= 3 else error_yaw
        self.error.depth = error_depth

    def callback_is_start(self, data):  
        if data.data:
            self.pub_error.publish(self.error)
            if self.is_stable_depth:
                self.pub_movement.publish('SURGE')

    def spin(self):
        rospy.spin()


def main():
    rospy.init_node('node_navigation', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()