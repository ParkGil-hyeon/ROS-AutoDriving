#!/usr/bin/env python

import rospy, time

from linedetector import LineDetector
from obstacledetector import ObstacleDetector
from motordriver import MotorDriver

class AutoDrive:

    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.obstacle_detector = ObstacleDetector('/ultrasonic')
        self.driver = MotorDriver('/xycar_motor_msg')

    def trace(self):
        obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
        left, right = self.line_detector.direction_info
        angle = self.steer(left, right)
        speed = self.accelerate(angle, obs_l, obs_m, obs_r)
        self.driver.drive(angle + 90, speed + 90)

    def steer(self, left, right):
        mid = (left + right) // 2
        if mid < 280:
            angle = -30
        elif mid > 360:
            angle = 30
        else:
            angle = 0
        return angle

    def accelerate(self, angle, left, mid, right):
        # if min(left, mid, right) < 50:
        #     speed = 0
        if angle < -20 or angle > 20:
            speed = 20
        else:
            speed = 30
        return speed

    def exit(self):
        print('finished')

if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        car.trace()
        rate.sleep()
    rospy.on_shutdown(car.exit)
