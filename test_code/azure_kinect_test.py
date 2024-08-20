import open3d as o3d
import matplotlib.pyplot as plt
import numpy as np
import pykinect_azure as pykinect
import cv2
import os
import sys 
import json
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from utils.camera import get_intrinsics

# 配置文件路径
config_path = os.path.join(os.path.dirname(__file__), '../config/default_config.json')
# 创建相机参数对象
# config = o3d.io.AzureKinectSensorConfig()
config = o3d.io.read_azure_kinect_sensor_config(config_path)

# 读取配置文件
with open(config_path, 'r') as f:
    json_data = json.load(f)
   
# 获取相机内参
intrinsic =  get_intrinsics(json_data.get('color_resolution'),json_data.get('depth_mode'))
# print(intrinsic)

# 列出可用的设备
list_devices = False
if list_devices:
    o3d.io.AzureKinectSensor.list_devices()

# 实例化相机对象
sensor = o3d.io.AzureKinectSensor(config)

# 开启相机,index默认为0
if not sensor.connect(0):
  raise RuntimeError('Failed to connect to sensor')

# 存储一帧RGBD图像，需要循环获取，单此次获取可能为空
rgbd = None
while rgbd is None:
  rgbd:o3d.geometry.RGBDImage = sensor.capture_frame(True)

# 获取RGB图像
color = np.asarray(rgbd.color)
# 获取深度图像
depth = np.asarray(rgbd.depth)

# 检查并转换图像格式
if color.dtype != np.uint8:
    color = (color * 255).astype(np.uint8)
if depth.dtype != np.uint16 and depth.dtype != np.float32:
    depth = depth.astype(np.float32)

# 创建新的 RGBD 图像
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    o3d.geometry.Image(color),
    o3d.geometry.Image(depth),
    convert_rgb_to_intensity=False
)


open3d_intrinsic = o3d.camera.PinholeCameraIntrinsic(
    width = intrinsic['width'],
    height = intrinsic['height'],
    fx = intrinsic['fx'],
    fy = intrinsic['fy'],
    cx = intrinsic['cx'],
    cy = intrinsic['cy']
    )

# 从RGBD图像生成点云
pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, open3d_intrinsic)

# 转换点云坐标系，使其与Open3D的坐标系对齐
pcd.transform([[1, 0, 0, 0],
               [0, -1, 0, 0],
               [0, 0, -1, 0],
               [0, 0, 0, 1]])


# 定义RGB范围
lower_bound = np.array([145, 0, 0]) /255 # 下界
upper_bound = np.array([255, 80, 90])/255  # 上界

# # 定义HSV范围
# lower_bound_hsv = np.array([156, 70, 50])  # 下界
# upper_bound_hsv = np.array([179, 255, 255])  # 上界

# 提取符合条件的点
colors = np.asarray(pcd.colors)
points = np.asarray(pcd.points)

# # 将颜色值从 [0, 1] 转换为 [0, 255]
# colors = (colors * 255).astype(np.uint8)

# # 将RGB颜色转换为HSV颜色
# colors_hsv = cv2.cvtColor(colors.reshape(-1, 1, 3), cv2.COLOR_RGB2HSV).reshape(-1, 3)

# colors = colors / 255
# print(colors)
mask = np.all((colors >= lower_bound) & (colors <= upper_bound), axis=1)
filtered_points = points[mask]
filtered_colors = colors[mask]

# # 创建掩码
# mask = np.all((colors_hsv >= lower_bound_hsv) & (colors_hsv <= upper_bound_hsv), axis=1)
# filtered_points = points[mask]
# filtered_colors = colors[mask]

# 创建新的点云
filtered_pcd = o3d.geometry.PointCloud()
filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
filtered_pcd.colors = o3d.utility.Vector3dVector(filtered_colors)


# 半径滤波
radius = 0.01  # 半径
min_neighbors = 13  # 最小邻居数
filtered_pcd_radius, ind = filtered_pcd.remove_radius_outlier(nb_points=min_neighbors, radius=radius)

# 统计滤波
nb_neighbors = 10  # 邻居数
std_ratio = 2.0  # 标准差倍数
filtered_pcd_stat, ind = filtered_pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)

# 可视化RGB图像
plt.subplot(1, 2, 1)
plt.title('RGB Image')
plt.imshow(color)

# 可视化深度图像
plt.subplot(1, 2, 2)
plt.title('Depth Image')
plt.imshow(depth, cmap='gray')

plt.show()

# 可视化点云
o3d.visualization.draw_geometries([pcd])

o3d.visualization.draw_geometries([filtered_pcd])

# 可视化半径滤波后的点云
o3d.visualization.draw_geometries([filtered_pcd_radius])

# 可视化统计滤波后的点云
o3d.visualization.draw_geometries([filtered_pcd_stat])

print(filtered_pcd)
print(filtered_pcd_radius)
print(filtered_pcd_stat)