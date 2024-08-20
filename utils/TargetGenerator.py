# Description: 生成随机目标点的类，目标点在一个球形区域内，球心为origin，半径为radius
# Author: Shen Guang-hui
# Create date: 2024-08-20
# Version: 1.0
# Environment: Windows 11, Python 3.8.0
import numpy as np

class TargetGenerator:
    def __init__(self, origin, radius):
        self.origin = origin
        self.radius = radius
    
    def get(self):
      # 生成均匀分布的随机方向的单位向量
      theta = np.random.uniform(0, 2 * np.pi)  # 方位角
      phi = np.random.uniform(0, np.pi)        # 极角

      x = np.sin(phi) * np.cos(theta)
      y = np.sin(phi) * np.sin(theta)
      z = np.cos(phi)
      direction = np.array([x, y, z])

      # 生成一个随机位置
      random_radius = np.random.uniform(0, self.radius)
      pos = self.origin + direction * random_radius

      # 生成随机旋转角度（rx, ry, rz）
      rx = np.random.uniform(-np.pi, np.pi)
      ry = np.random.uniform(-np.pi, np.pi)
      rz = np.random.uniform(-np.pi, np.pi)

      return pos[0], pos[1], pos[2], rx, ry, rz 
    
    def get_keep_left(self):
       
       # 生成均匀分布的随机方向的单位向量
      theta = np.random.uniform(1/2*np.pi, 3/2 * np.pi)  # 方位角
      phi = np.random.uniform(0, np.pi)        # 极角

      x = np.sin(phi) * np.cos(theta)
      y = np.sin(phi) * np.sin(theta)
      z = np.cos(phi)
      direction = np.array([x, y, z])

      # 生成一个随机位置
      random_radius = np.random.uniform(0, self.radius)
      pos = self.origin + direction * random_radius

      # 生成随机旋转角度（rx, ry, rz）
      rx = np.random.uniform(-np.pi, np.pi)
      ry = np.random.uniform(-np.pi, 0)
      rz = np.random.uniform(-np.pi, np.pi)

      return pos[0], pos[1], pos[2], rx, ry, rz

    def get_keep_right(self):
       
       # 生成均匀分布的随机方向的单位向量
      theta = np.random.uniform(-1/2*np.pi, 1/2*np.pi)  # 方位角
      phi = np.random.uniform(0, np.pi)        # 极角

      x = np.sin(phi) * np.cos(theta)
      y = np.sin(phi) * np.sin(theta)
      z = np.cos(phi)
      direction = np.array([x, y, z])

      # 生成一个随机位置
      random_radius = np.random.uniform(0, self.radius)
      pos = self.origin + direction * random_radius

      # 生成随机旋转角度（rx, ry, rz）
      rx = np.random.uniform(-np.pi, np.pi)
      ry = np.random.uniform(0, np.pi)
      rz = np.random.uniform(-np.pi, np.pi)

      return pos[0], pos[1], pos[2], rx, ry, rz 
        
if __name__ == "__main__":
    target_generator = TargetGenerator(np.array([0, 0, 1.375]), 0.3)
    print(target_generator.get())
    print(target_generator.get_keep_left())
    print(target_generator.get_keep_right())
    