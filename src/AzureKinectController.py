# This module is used to get the RGB and depth images from 
# Azure Kinect sensor and save them to the specified directory.
# Author: Shen Guang-hui
# Date: 2024-8-19
# Version: 1.0
# Environment: Windows 11, Python 3.8.0

from __init__ import *

# 将当前目录添加到系统路径中
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from utils.camera import get_intrinsics , resolution

class AzureKinectSensor:
    def __init__(self, args):
        self.args = args
        self.align_depth_to_color = args.align_depth_to_color

        # 配置文件路径
        self.config_path = os.path.join(os.path.dirname(__file__), args.config_path)
        # 创建相机参数对象
        config = o3d.io.read_azure_kinect_sensor_config(self.config_path)
        with open(self.config_path, 'r') as f:
            self.camera_config = json.load(f)
        # 初始化内参
        self.intrinsic = self.init_intrinsics()
        self.fps = (int)(self.camera_config["camera_fps"].split("_")[-1])
        print(f'Camera FPS: {self.fps}')
        self.sensor = o3d.io.AzureKinectSensor(config)
        if not self.sensor.connect(args.device):
            raise RuntimeError('Failed to connect to sensor')
        else:
            print("Sensor initialized. Press [ESC] to exit.")
 
    
    def visualization_with_thread(self, message_queue,name = 'viewer',resolution = (1280,720)):
        vis = o3d.visualization.Visualizer()
        vis.create_window(name, resolution[0], resolution[1])
        
        while True:
            
            try:
                # 阻塞等到消息队列中的消息
                data = message_queue.get(timeout=1)
                
                vis.clear_geometries()
                vis.add_geometry(data)
                vis.update_geometry(data)
                vis.poll_events()
                vis.update_renderer()
            except queue.Empty:
                continue
            

    def init_intrinsics(self):
        # 读取配置文件
        with open(self.config_path, 'r') as f:
            json_data = json.load(f)
        # 获取相机内参
        intrinsic = get_intrinsics(json_data.get('color_resolution'), json_data.get('depth_mode'))
        if intrinsic is None:
            raise ValueError('Intrinsic not found')
        self.intrinsic = intrinsic
        return intrinsic
    def in_range(self, pcd,lower_bound, upper_bound):
        # 定义RGB范围
        lower_bound = np.array(lower_bound) /255 # 下界
        upper_bound = np.array(upper_bound)/255  # 上界

        # 提取符合条件的点
        colors = np.asarray(pcd.colors)
        points = np.asarray(pcd.points)
       
        mask = np.all((colors >= lower_bound) & (colors <= upper_bound), axis=1)
        filtered_points = points[mask]
        filtered_colors = colors[mask]

        # 创建分割点云
        filtered_pcd = o3d.geometry.PointCloud()
        filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)
        filtered_pcd.colors = o3d.utility.Vector3dVector(filtered_colors)

        return filtered_pcd

    def pointcloud_filter(self, pcd):
        # 半径滤波
        radius = self.camera_config["filter_radius"]  # 半径
        min_neighbors = self.camera_config["filter_min_neighbors"]  # 最小邻居数
        filtered_pcd_radius, ind = pcd.remove_radius_outlier(nb_points=min_neighbors, radius=radius)

        # 统计滤波
        nb_neighbors = self.camera_config["filter_nb_neighbors"]  # 邻居数
        std_ratio = self.camera_config["filter_std_ratio"]  # 标准差倍数
        filtered_pcd_stat, ind = filtered_pcd_radius.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)

        return filtered_pcd_stat
    
            
    def run(self,RgbAndDepth=True,pointcloud=True,filters=True,data_dir=None):
        
        
        width,height = resolution[self.camera_config["color_resolution"]]['width'], resolution[self.camera_config["color_resolution"]]['height']
        if RgbAndDepth:
            rgbd_queue = queue.Queue()
            rgbd_vis = Thread(target=self.visualization_with_thread, args=(rgbd_queue,'RGBD', (width*2, height)),daemon=True)
            rgbd_vis.start()
        if pointcloud:
            pcd_queue = queue.Queue()
            pcd_vis = Thread(target=self.visualization_with_thread, args=(pcd_queue,'Pointcloud', (width, height)),daemon=True)
            pcd_vis.start()
        
        
        for i in tqdm(range(self.fps * args.seconds)):
            
            # 存储一帧RGBD图像，需要循环获取，单此次获取可能为空
            rgbd = None
            while rgbd is None:
                rgbd:o3d.geometry.RGBDImage = self.sensor.capture_frame(self.align_depth_to_color)

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

            if RgbAndDepth:
                rgbd_queue.put(rgbd)
                    # 保存RGB图像为 png 文件
                cv2.imwrite(f'{data_dir}/color_image_{i:04d}.png', cv2.cvtColor(color, cv2.COLOR_RGB2BGR))
                # 保存深度图像为 png 文件
                cv2.imwrite(f'{data_dir}/depth_image_{i:04d}.png', depth)

            if pointcloud:
                if self.intrinsic is  None:
                    self.intrinsic = self.init_intrinsics()

                open3d_intrinsic = o3d.camera.PinholeCameraIntrinsic(
                    width = self.intrinsic['width'],
                    height = self.intrinsic['height'],
                    fx = self.intrinsic['fx'],
                    fy = self.intrinsic['fy'],
                    cx = self.intrinsic['cx'],
                    cy = self.intrinsic['cy']
                    )

                # 从RGBD图像生成点云，相对于相机坐标系
                pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, open3d_intrinsic)
                # 转换点云坐标系，使其与世界坐标系对齐
                pcd.transform(json.loads(self.camera_config["transform2world"]))

                # 定义RGB范围
                lower_bound = [145, 0, 0] # 下界
                upper_bound = [255, 80, 90]  # 上界

                # 提取符合条件的点
                pcd_in_range = self.in_range(pcd,lower_bound, upper_bound)

                if filters:
                    pcd_out = self.pointcloud_filter(pcd_in_range)
                else:
                    pcd_out = pcd_in_range
                # 显示点云并保存
                pcd_queue.put(pcd_out)
                # 保存点云数据为 npy 文件
                np.save(f'{data_dir}/filtered_point_cloud_{i:04d}.npy', np.asarray(pcd_out.points))
            
            else:            
                print(f"Data saved to {data_dir} directory")

    def run_once(self,RgbAndDepth=True,pointcloud=True,filters=True,data_dir=None):
        # 存储一帧RGBD图像，需要循环获取，单此次获取可能为空
        rgbd = None
        while rgbd is None:
          rgbd:o3d.geometry.RGBDImage = self.sensor.capture_frame(self.align_depth_to_color)

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

        if RgbAndDepth:
            width,height = resolution[self.camera_config["color_resolution"]]['width'], resolution[self.camera_config["color_resolution"]]['height']
            vis = o3d.visualization.Visualizer()
            vis.create_window('viewer', width*2, height)
            # self.display_rgbd(rgbd)
            # 可视化RGB图像
            vis.add_geometry(rgbd)
            vis.update_geometry(rgbd)
            vis.poll_events()
            vis.update_renderer()
            vis.run()
            if data_dir is not None:
                # 保存RGB图像为 png 文件
                cv2.imwrite(f'{data_dir}/color_image.png', cv2.cvtColor(color, cv2.COLOR_RGB2BGR))
                # 保存深度图像为 png 文件
                cv2.imwrite(f'{data_dir}/depth_image.png', depth)
        
        if pointcloud:
            if self.intrinsic is  None:
                self.intrinsic = self.init_intrinsics()

            open3d_intrinsic = o3d.camera.PinholeCameraIntrinsic(
                width = self.intrinsic['width'],
                height = self.intrinsic['height'],
                fx = self.intrinsic['fx'],
                fy = self.intrinsic['fy'],
                cx = self.intrinsic['cx'],
                cy = self.intrinsic['cy']
                )

            # 从RGBD图像生成点云，相对于相机坐标系
            pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, open3d_intrinsic)
            # 转换点云坐标系，使其与世界坐标系对齐
            pcd.transform(json.loads(self.camera_config["transform2world"]))

            # 定义RGB范围
            lower_bound = [145, 0, 0] # 下界
            upper_bound = [255, 80, 90]  # 上界

            pcd_in_range = self.in_range(pcd,lower_bound, upper_bound)

            if filters:
                pcd_out = self.pointcloud_filter(pcd_in_range)
            else:
                pcd_out = pcd_in_range

            # 可视化半径滤波和统计滤波后的点云
            o3d.visualization.draw_geometries([pcd_out])
            if data_dir is not None:
                # 保存点云数据为 npy 文件
                np.save(f'{data_dir}/filtered_point_cloud.npy', np.asarray(pcd_out.points))
            # 打印滤波后的点云信息
            print(pcd_out)
          
def add_arguments():
     # 添加相机需要的参数
    parser = argparse.ArgumentParser(description='Azure kinect rgb & depth collector.')
    # 传感器配置文件路径
    parser.add_argument('--config_path', type=str, default="../config/default_config.json", help='input json kinect config path')
    parser.add_argument('--data_dir',
                        type=str,
                        default="../kinect_data",
                        help='input kinect data save path')
    parser.add_argument('--list',
                        action='store_true',
                        help='list available azure kinect sensors')
    # 传感其设备 id
    parser.add_argument('--device',
                        type=int,
                        default=0,
                        help='input kinect device id')
    # 采样时长
    parser.add_argument('-s',
                        '--seconds',
                        type=int,
                        default=5,
                        help='Record duration')
    parser.add_argument('-d',
                        '--delay',
                        type=int,
                        default=1,
                        help='Delay before recording')
    # 深度图像是否对齐到彩色图像
    parser.add_argument('-a',
                        '--align_depth_to_color',
                        type=bool,
                        default=True,
                        help='enable align depth image to color')
    return parser

if __name__ == '__main__':
    # 添加命令行参数
    parser = add_arguments()
    # 解析命令行参数
    args = parser.parse_args()
    # 创建数据保存目录
    kinect_data_path = os.path.join("../kinect_data")
    Path(kinect_data_path).mkdir(parents=True, exist_ok=True)
    
    if args.list:
        o3d.io.AzureKinectSensor.list_devices()
        exit()
    # 设备校验    
    device = args.device
    if device < 0 or device > 255:
        print('Unsupported device id, fall back to 0')
        device = 0
    
    # 是否需要延迟
    for i in tqdm(range(args.delay)):
        sleep(1)

    # 实例化相机对象
    sensor = AzureKinectSensor(args)
    # 运行相机采集数据
    # sensor.run_once(RgbAndDepth=True,pointcloud=True,filters=True,data_dir=args.data_dir)
    sensor.run(RgbAndDepth=True,pointcloud=True,filters=True,data_dir=args.data_dir)
