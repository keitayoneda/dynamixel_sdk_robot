#!/usr/bin/env python

import rospy
from dynamixel_sdk_robot.msg import servo_position, servo_position_array
from math import sin, pi

def main():
    publisher = rospy.Publisher('set_position_array', servo_position_array, queue_size=1)
    rospy.init_node("move_test_node", anonymous=True)
    ids = [1, 2, 3, 4, 5, 6]
    t = 0
    dt = 2*pi/2/20
    rate = rospy.Rate(10) #5hz
    while not rospy.is_shutdown():
        pos = int(1000*sin(t)) + 1000
        pos_array = []
        for _id in ids:
            pos_array.append(servo_position(_id, pos))
        publisher.publish(servo_position_array(data=pos_array))
        t += dt
        rate.sleep()

if __name__ == "__main__":
    main()

