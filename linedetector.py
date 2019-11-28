import rospy
import cv2
import numpy as np
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

image_width = 640
scan_width, scan_height = 300, 80
roi_vertical_pos = 250
area_width = 5
area_height = 5
row_begin = (scan_height - area_height) // 2 - 10
row_end = row_begin + area_height
lmid, rmid = scan_width, image_width - scan_width
pixel_cnt_threshold = 0.6 * area_width * area_height

class LineDetector:

    def __init__(self, topic):
        self.step_1 = np.zeros(shape=(scan_height, image_width, 3), dtype=np.uint8)
        self.step_2 = np.zeros(shape=(scan_height, image_width, 3), dtype=np.uint8)
        self.step_3 = np.zeros(shape=(scan_height, image_width, 3), dtype=np.uint8)
        self.step_4 = np.zeros(shape=(scan_height, image_width, 3), dtype=np.uint8)
        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.result = np.zeros(shape=(scan_height, image_width, 3), dtype=np.uint8)
        self.bridge = CvBridge()
        self.direction_info = [-40, 680]
        rospy.Subscriber(topic, Image, self.conv_image)

    def conv_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        v = roi_vertical_pos
        roi = self.cam_img[v:v + scan_height, :]

        roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        roi = cv2.GaussianBlur(roi, (5, 5), 0)
        roi = cv2.Canny(roi, 50, 80)
        self.step_1 = roi
        lines = cv2.HoughLines(roi, 1, np.pi / 180, 80, None, 0, 0)

        left = [0, 0, 0, 0]
        left_count = 0
        right = [0, 0, 0, 0]
        right_count = 0

        result = np.zeros((scan_height, image_width, 3), dtype=np.uint8)
        test = np.zeros((scan_height, image_width, 3), dtype=np.uint8)
        if lines is not None:
            for line in lines:
                for rho, theta in line:
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a * rho
                    y0 = b * rho

                    x1 = int(x0 + 1000 * (-b))
                    y1 = int(y0 + 1000 * (a))
                    x2 = int(x0 - 1000 * (-b))
                    y2 = int(y0 - 1000 * (a))

                    if float(y2 - y1) / float(x2 - x1) <= -0.2:
                        left[0] += x1
                        left[1] += y1
                        left[2] += x2
                        left[3] += y2
                        left_count += 1
                        cv2.line(test, (x1, y1), (x2, y2), (255, 0, 0), 3, cv2.LINE_AA)
                    elif float(y2 - y1) / float(x2 - x1) >= 0.2:
                        right[0] += x1
                        right[1] += y1
                        right[2] += x2
                        right[3] += y2
                        right_count += 1
                        cv2.line(test, (x1, y1), (x2, y2), (0, 255, 0), 3, cv2.LINE_AA)
                    else:
                        cv2.line(test, (x1, y1), (x2, y2), (0, 0, 255), 3, cv2.LINE_AA)
        self.step_2 = test

        if left_count > 0:
            for i, value in enumerate(left):
                left[i] = value // left_count
        if right_count > 0:
            for i, value in enumerate(right):
                right[i] = value // right_count

        cv2.line(result, (left[0], left[1]), (left[2], left[3]), (255, 0, 0), 3, cv2.LINE_AA)
        cv2.line(result, (right[0], right[1]), (right[2], right[3]), (255, 0, 0), 3, cv2.LINE_AA)
        self.step_3 = result

        hsv = cv2.cvtColor(result, cv2.COLOR_BGR2HSV)

        lbound = np.array([0, 0, 60], dtype=np.uint8)
        ubound = np.array([131, 255, 255], dtype=np.uint8)

        bin = cv2.inRange(hsv, lbound, ubound)
        view = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)

        left, right = -40, 680

        for l in range(area_width, lmid):
            area = bin[row_begin:row_end, l - area_width:l]
            if cv2.countNonZero(area) > pixel_cnt_threshold:
                left = l
                break

        for r in range(image_width - area_width, rmid, -1):
            area = bin[row_begin:row_end, r:r + area_width]
            if cv2.countNonZero(area) > pixel_cnt_threshold:
                right = r
                break

        if left != -40:
            lsquare = cv2.rectangle(view,
                                    (left - area_width, row_begin),
                                    (left, row_end),
                                    (0, 255, 0), 3)
        else:
            pass
            # print("Lost left line")

        if right != 680:
            rsquare = cv2.rectangle(view,
                                    (right, row_begin),
                                    (right + area_width, row_end),
                                    (0, 255, 0), 3)
        else:
            pass
            # print("Lost right line")
        self.direction_info = [left, right]
        self.step_4 = view
        # print(left, right)

    def show_image(self):
        # cv2.imshow('step_1', self.step_1)
        # cv2.imshow('step_2', self.step_2)
        # cv2.imshow('step_3', self.step_3)
        # cv2.imshow('step_4', self.step_4)
        cv2.waitKey(1)
