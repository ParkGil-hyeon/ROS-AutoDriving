#!/usr/bin/env python

import cv2, time
import numpy as np

cap = cv2.VideoCapture('2.avi')

image_width = 640
scan_height = 80
roi_vertical_pos = 270

while True:
    ret, frame = cap.read()
    if not ret:
        break
    if cv2.waitKey(1) & 0xFF == 27:
        break

    roi = frame[roi_vertical_pos:roi_vertical_pos + scan_height, :]
    roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    roi = cv2.GaussianBlur(roi, (5, 5), 0)
    roi = cv2.Canny(roi, 50, 100)
    lines = cv2.HoughLines(roi, 1, np.pi/180, 80)

    semi = np.zeros(shape=(scan_height, image_width, 3), dtype=np.uint8)
    result = np.zeros(shape=(scan_height, image_width, 3), dtype=np.uint8)
    left = [0, 0, 0, 0]
    left_count = 0
    right = [0, 0, 0, 0]
    right_count = 0
    if lines is not None:
        for line in lines:
            for rho, theta in line:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho

                p1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
                p2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
                cv2.line(semi, p1, p2, (0, 0, 255), 3, cv2.LINE_AA)

                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))

                if (y2 - y1)/(x2 - x1) <= -0.2:
                    left[0] += x1
                    left[1] += y1
                    left[2] += x2
                    left[3] += y2
                    left_count += 1
                elif (y2 - y1)/(x2 - x1) >= 0.2:
                    right[0] += x1
                    right[1] += y1
                    right[2] += x2
                    right[3] += y2
                    right_count += 1

    if left_count > 0:
        for i, value in enumerate(left):
            left[i] = value//left_count
    if right_count > 0:
        for i, value in enumerate(right):
            right[i] = value//right_count

    cv2.line(result, (left[0], left[1]), (left[2], left[3]), (255, 0, 0), 3, cv2.LINE_AA)
    cv2.line(result, (right[0], right[1]), (right[2], right[3]), (255, 0, 0), 3, cv2.LINE_AA)

    cv2.imshow('origin', frame)
    cv2.imshow('canny', roi)
    cv2.imshow('semi', semi)
    cv2.imshow('result', result)

    time.sleep(0.005)

cap.release()
cv2.destroyAllWindows()
