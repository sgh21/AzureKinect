# import cv2
# import pykinect_azure as pykinect
 
 
# # 初始化库，可以库的路径作为参数
# pykinect.initialize_libraries()
 
# # 修改相机配置
# device_config = pykinect.default_configuration  # 默认初始配置
# device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_2160P  # 修改分辨率
 
# # 开启设备
# device = pykinect.start_device(config=device_config)
 
# while True:
# 	capture = device.update()  # 更新获取
# 	ret, rgb_img = capture.get_color_image()
# 	# rgb_img = cv2.resize(rgb_img, fx=0.5, fy=0.5)
# 	if not ret:
# 		continue
# 	cv2.imshow('rgb image', rgb_img)
# 	cv2.imwrite('216p.jpg', rgb_img)
# 	if cv2.waitKey(1) == ord('q'):
# 		break
	

# import cv2
# import pykinect_azure as pykinect
 
 
# pykinect.initialize_libraries()
# device_config = pykinect.default_configuration
# device_config.color_format = pykinect.K4A_IMAGE_FORMAT_COLOR_BGRA32
# device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_1536P
# device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED
 
# device = pykinect.start_device(config=device_config)
 
# while True:
#     capture = device.update()
#     ret_color, color_image = capture.get_color_image()
#     ret_depth, transformed_colored_depth_image = capture.get_transformed_colored_depth_image()
#     if not ret_color or not ret_depth:
#         continue
 
#     transformed_colored_depth_image = cv2.cvtColor(transformed_colored_depth_image, cv2.COLOR_BGR2GRAY)
 
#     alpha = 5  # 可调节对比度
#     beta = 100  # 可调节亮度
#     adjusted_image = cv2.convertScaleAbs(transformed_colored_depth_image, alpha=alpha, beta=beta)
#     cv2.imshow('Transformed Color Depth Image', adjusted_image)
#     cv2.imwrite('depth_img.jpg', adjusted_image)
 
#     if cv2.waitKey(1) == ord('q'):
#         break


# import cv2
# import numpy as np

# path = r'..\\documents\\TestDoc\\HSVtest.png'  # 图像的位置，使用时仅需修改这一属性

# # 滑动条的回调函数，获取滑动条位置处的值
# def empty(a):
#     r_min = cv2.getTrackbarPos("Red Min", "TrackBars")
#     r_max = cv2.getTrackbarPos("Red Max", "TrackBars")
#     g_min = cv2.getTrackbarPos("Green Min", "TrackBars")
#     g_max = cv2.getTrackbarPos("Green Max", "TrackBars")
#     b_min = cv2.getTrackbarPos("Blue Min", "TrackBars")
#     b_max = cv2.getTrackbarPos("Blue Max", "TrackBars")
#     print(r_min, r_max, g_min, g_max, b_min, b_max)
#     return r_min, r_max, g_min, g_max, b_min, b_max

# # 创建一个窗口，放置6个滑动条
# cv2.namedWindow("TrackBars")
# cv2.resizeWindow("TrackBars", 640, 240)
# cv2.createTrackbar("Red Min", "TrackBars", 0, 255, empty)
# cv2.createTrackbar("Red Max", "TrackBars", 255, 255, empty)
# cv2.createTrackbar("Green Min", "TrackBars", 0, 255, empty)
# cv2.createTrackbar("Green Max", "TrackBars", 255, 255, empty)
# cv2.createTrackbar("Blue Min", "TrackBars", 0, 255, empty)
# cv2.createTrackbar("Blue Max", "TrackBars", 255, 255, empty)

# while True:
#     img = cv2.imread(path)
#     # 调用回调函数，获取滑动条的值
#     r_min, r_max, g_min, g_max, b_min, b_max = empty(0)
#     lower = np.array([b_min, g_min, r_min])
#     upper = np.array([b_max, g_max, r_max])
#     # 获得指定颜色范围内的掩码
#     mask = cv2.inRange(img, lower, upper)
#     # 对原图图像进行按位与的操作，掩码区域保留
#     imgResult = cv2.bitwise_and(img, img, mask=mask)
#     cv2.imshow("Mask", mask)
#     cv2.imshow("Result", imgResult)
#     cv2.waitKey(1)