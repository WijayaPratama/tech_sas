#!/usr/bin/env python3

import rospy
from robotic_sas_auv_ros.msg import Sensor, Orientation

class Subscriber():
    def __init__(self):
        # Publisher
        self.pub_sensor = rospy.Publisher('sensor', Sensor, queue_size=10)

        # Subscriber
        # rospy.Subscriber('arduino/sensor', Sensor, self.callback_sensor)
        rospy.Subscriber('/orientation', Orientation, self.callback_sensor)

    def callback_sensor(self, data):
        rospy.loginfo("Received a message!")
        self.pub_sensor.publish(data)
        rospy.loginfo(data)

    def spin(self):
        rospy.spin()

def main():
    rospy.init_node('node_accumulator', anonymous=True)

    subscriber = Subscriber()

    subscriber.spin()

if __name__ == '__main__':
    main()