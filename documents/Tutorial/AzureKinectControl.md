### 模块介绍
+ 源码位置：模块代码集中在AzureKinectController.py文件中，依赖utils/camera.py文件
+ 功能：该模块可以连接控制AzureKinect传感器，实现相机RGB图、深度图和点云的获取和实时显示，支持连续获取和单帧获取两种模式
### 安装依赖
+ open3d >= 0.8.0
+ opencv 
+ 另外，官方AzureKinectSDK是必须的，具体要求见GetStartWithPython.md

### 函数库
+ Class AzureKinectSensor
  + property:
    + intrinsic:dict 传感器的内参
    + fps：int 相机的帧率
    + sensor：open3d.cpu.pybind.io.AzureKinectSensor 传感器实例
    + config_path：str 传感器配置文件路径
    + align_depth_to_color：bool 深度图是否与彩色图对齐
    + args:argparse.Namespace 命令行参数集
  + method:
    + visualization_with_thread(self:AzureKinectSnsor, message_queue:Queue,name:str = 'viewer',resolution:Tuple = (1280,720))->None 
      + 单开一个子线程，对message_queue的数据进行可视化显示
      + input:
        + message_queue:Queue 可视化数据队列
        + name:str = 'viewer' 可视化窗口名称
        + resolution:Tuple = (1280,720) 可视化窗口分辨率
    + init_intrinsics(self:AzureKinectSnsor)->dict
      + 初始化传感器内参并返回
      + output:
        + intrinsic:dict 传感器内参
    + in_range(self：AzureKinectSnsor, pcd：open3d.geometry.PointCloud,lower_bound：list, upper_bound:list)->open3d.geometry.PointCloud
      + 从pcd中分割出在RGB筛选范围内的点云
      + input：
        + pcd:open3d.geometry.PointCloud 原始点云
        + lower_bound:list RGB下界(unit8)
        + upper_bound:list RGB上界(unit8)
      + output：
        + filtered_pcd:open3d.geometry.PointCloud 在筛选范围内的点云
    + pointcloud_filter(self:AzureKinectSnsor, pcd：open3d.geometry.PointCloud)->open3d.geometry.PointCloud
      + 对pcd进行半径滤波和统计滤波，消除离群点，滤波参数取决于传感器config文件
      + input:
        + pcd:open3d.geometry.PointCloud 原始点云
      + output:
        + filtered_pcd_stat:open3d.geometry.PointCloud 滤波后的点云
    + run(self:AzureKinectSnsor,RgbAndDepth:bool=True,pointcloud:bool=True,filters:bool=True,data_dir:str=None)->None
      + 连续采集RGB图、深度图和点云数据
      + input：
        + RgbAndDepth:bool=True 是否采集、显示RGB图和深度图
        + pointcloud:bool=True 是否采集、显示点云数据(.npy)
        + filters:bool=True 是否对点云数据进行滤波
        + data_dir:str=None 文件保存路径
    + run_once(self:AzureKinectSnsor,RgbAndDepth:bool=True,pointcloud:bool=True,filters:bool=True,data_dir:str=None)->None
      + 采集一帧RGB图、深度图和点云数据
      + input：
        + RgbAndDepth:bool=True 是否采集、显示RGB图和深度图
        + pointcloud:bool=True 是否采集、显示点云数据(.npy)
        + filters:bool=True 是否对点云数据进行滤波
        + data_dir:str=None 文件保存路径 
+ add_arguments()->
  + 向命令行添加参数