#!/usr/bin/env python

import rospy, time

from linedetector import LineDetector
from obstacledetector import ObstacleDetector
from motordriver import MotorDriver
import time

class AutoDrive:

    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.obstacle_detector = ObstacleDetector('/ultrasonic')
        self.driver = MotorDriver('/xycar_motor_msg')

    def trace(self):
       	distance = self.obstacle_detector.get_distance()
        left, right = self.line_detector.direction_info
        self.line_detector.show_image()
        angle = self.steer(left, right)
        speed = self.accelerate(angle, distance)
        self.driver.drive(angle + 90, speed + 90)

    def steer(self, left, right):
        mid = (left + right) // 2
	angle = float(mid-320)/1.7
        print("angle:",angle)
        return angle

    def accelerate(self, angle, distance):
        if distance < 140 and angle >= -14 and angle <= 14 and time.time()-start_time >= 50:
	     	print("stop!")
             	speed = 0
        elif angle <= -12 or angle >=12 :
            	speed = 38
        else:
            	speed = 45
        print("speed : ",speed)
        return speed

    def exit(self):
        print('finished')

if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(4)
    rate = rospy.Rate(10)
    start_time = time.time()
    while not rospy.is_shutdown():
        car.trace()
    rospy.on_shutdown(car.exit)
