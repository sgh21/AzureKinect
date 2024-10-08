# Function: This script is used to measure the HSV value of a specific color 
# in an image and real-time show the mask and result of the color 
# Author: Shen Guang-hui
# Create date: 2024-08-15
# Version: 1.0
# Environment: Windows 11, Python 3.8.0

import cv2  
import numpy as np  

path = r'..\\documents\\TestDoc\\HSVtest.png'  #图像的位置，使用时仅需修改这一属性
# 滑动条的回调函数，获取滑动条位置处的值  
def empty(a):  
    h_min = cv2.getTrackbarPos("Hue Min", "TrackBars")  
    h_max = cv2.getTrackbarPos("Hue Max", "TrackBars")  
    s_min = cv2.getTrackbarPos("Sat Min", "TrackBars")  
    s_max = cv2.getTrackbarPos("Sat Max", "TrackBars")  
    v_min = cv2.getTrackbarPos("Val Min", "TrackBars")  
    v_max = cv2.getTrackbarPos("Val Max", "TrackBars")  
    print(h_min, h_max, s_min, s_max, v_min, v_max)  
    return h_min, h_max, s_min, s_max, v_min, v_max  
    
# 创建一个窗口，放置6个滑动条  
cv2.namedWindow("TrackBars")  
cv2.resizeWindow("TrackBars", 640, 240)  
cv2.createTrackbar("Hue Min", "TrackBars", 0, 179, empty)  
cv2.createTrackbar("Hue Max", "TrackBars", 19, 179, empty)  
cv2.createTrackbar("Sat Min", "TrackBars", 110, 255, empty)  
cv2.createTrackbar("Sat Max", "TrackBars", 240, 255, empty)  
cv2.createTrackbar("Val Min", "TrackBars", 153, 255, empty)  
cv2.createTrackbar("Val Max", "TrackBars", 255, 255, empty)  
  
while True:  
    img = cv2.imread(path)  
    imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  
    # 调用回调函数，获取滑动条的值  
    h_min, h_max, s_min, s_max, v_min, v_max = empty(0)  
    lower = np.array([h_min, s_min, v_min])  
    upper = np.array([h_max, s_max, v_max])  
    # 获得指定颜色范围内的掩码  
    mask = cv2.inRange(imgHSV, lower, upper)  
    # 对原图图像进行按位与的操作，掩码区域保留  
    imgResult = cv2.bitwise_and(img, img, mask=mask)  
    cv2.imshow("Mask", mask)  
    cv2.imshow("Result", imgResult)  
    cv2.waitKey(1)