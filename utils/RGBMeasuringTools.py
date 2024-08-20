# Function: This script is used to measure the RGB value of a specific color 
# in an image and real-time show the mask and result of the color 
# Author: Shen Guang-hui
# Create date: 2024-08-15
# Version: 1.0
# Environment: Windows 11, Python 3.8.0

import cv2
import numpy as np

path = r'..\\documents\\TestDoc\\RGBtest.png'  # 图像的位置，使用时仅需修改这一属性

# 滑动条的回调函数，获取滑动条位置处的值
def empty(a):
    r_min = cv2.getTrackbarPos("Red Min", "TrackBars")
    r_max = cv2.getTrackbarPos("Red Max", "TrackBars")
    g_min = cv2.getTrackbarPos("Green Min", "TrackBars")
    g_max = cv2.getTrackbarPos("Green Max", "TrackBars")
    b_min = cv2.getTrackbarPos("Blue Min", "TrackBars")
    b_max = cv2.getTrackbarPos("Blue Max", "TrackBars")
    print(r_min, r_max, g_min, g_max, b_min, b_max)
    return r_min, r_max, g_min, g_max, b_min, b_max

# 创建一个窗口，放置6个滑动条
cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars", 640, 240)
cv2.createTrackbar("Red Min", "TrackBars", 0, 255, empty)
cv2.createTrackbar("Red Max", "TrackBars", 255, 255, empty)
cv2.createTrackbar("Green Min", "TrackBars", 0, 255, empty)
cv2.createTrackbar("Green Max", "TrackBars", 255, 255, empty)
cv2.createTrackbar("Blue Min", "TrackBars", 0, 255, empty)
cv2.createTrackbar("Blue Max", "TrackBars", 255, 255, empty)

while True:
    img = cv2.imread(path)
    # 调用回调函数，获取滑动条的值
    r_min, r_max, g_min, g_max, b_min, b_max = empty(0)
    lower = np.array([b_min, g_min, r_min])
    upper = np.array([b_max, g_max, r_max])
    # 获得指定颜色范围内的掩码
    mask = cv2.inRange(img, lower, upper)
    # 对原图图像进行按位与的操作，掩码区域保留
    imgResult = cv2.bitwise_and(img, img, mask=mask)
    # 将掩码应用到原始图像上
    mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    cv2.imshow("Mask", mask_rgb)
    cv2.imshow("Result", imgResult)
    cv2.waitKey(1)