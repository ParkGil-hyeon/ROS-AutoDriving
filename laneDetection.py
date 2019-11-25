#!/usr/bin/env python

import cv2, time
import numpy as np
import math

cap = cv2.VideoCapture('2.avi')

value_threshold = 190

image_width = 640
scan_width, scan_height = 200, 100
lmid, rmid = scan_width, image_width - scan_width
area_width, area_height = 20, 10
roi_vertical_pos = 280
row_begin = (scan_height - area_height) // 2
row_end = row_begin + area_height
pixel_cnt_threshold = 0.35 * area_width * area_height

while True:
   ret, frame = cap.read()
   if not ret:
       break
   if cv2.waitKey(1) & 0xFF == 27:
       break

   roi = frame[roi_vertical_pos:roi_vertical_pos + scan_height, :]
   # roi = frame
   hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
   blur = cv2.GaussianBlur(hsv, (5, 5), 0)

   lbound = np.array([0, 0, value_threshold], dtype=np.uint8)
   ubound = np.array([131, 255, 255], dtype=np.uint8)
   dst = cv2.Canny(blur, 50, 200, None, 3)

   cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
   cdstP = np.copy(cdst)

   lines = cv2.HoughLines(dst, 1, np.pi / 180, 15, None, 0, 0)

   if lines is not None:
       for i in range(0, len(lines)):
           rho = lines[i][0][0]
           theta = lines[i][0][1]
           a = math.cos(theta)
           b = math.sin(theta)
           x0 = a * rho
           y0 = b * rho
           pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
           pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
           cv2.line(cdst, pt1, pt2, (0, 0, 255), 3, cv2.LINE_AA)

   linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)

   if linesP is not None:
       for i in range(0, len(linesP)):
           l = linesP[i][0]
           cv2.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 3, cv2.LINE_AA)
   cv2.imshow('test', cdstP)

   cv2.imshow("origin", frame)

   time.sleep(0.01)

cap.release()
cv2.destroyAllWindows()
