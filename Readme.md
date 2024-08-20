### 安装
+ 安装平台：Windows11家庭版
+ 使用AzureKinect需要安装的库和软件
  + Azure kinect SDK:微软官方发行的工具包和底层驱动，是连接相机必需的，推荐v1.4.1
  + open3d/pykinect_azure:两者任选其一，微软官方是没有提供相机的python API的，因此我们需要第三方库来控制kinect相机，open3d.io和pykinect_azure都可以，不过首推open3d毕竟是常用库，点云处理也能用
+ 安装步骤：
  + 安装 Azure kinect SDK并调试：
    + 在官网（https://learn.microsoft.com/zh-cn/azure/kinect-dk/sensor-sdk-download）上找到.exe安装包，下载安装。默认安装路径是"C:\Program Files\Azure Kinect SDK v1.4.1",在安装路径下可以看到tools文件夹，里面的k4aviewer.exe可以用于相机的连接和参数调试（亲测，这个不装连不上相机！）
  + 安装open3d：
    + 这个比较简单，直接用pip安装就好，最好安装0.8以后的版本
    ```
    pip install open3d
    ```
    + 官方文档：https://www.open3d.org/docs/release/python_api/open3d.io.html
    + 示例代码：
    ```python
    import open3d as o3d
    import matplotlib.pyplot as plt

    # 列出可用的设备
    list_devices = False
    if list_devices:
        o3d.io.AzureKinectSensor.list_devices()

    # 创建相机参数对象
    # config = o3d.io.read_azure_kinect_sensor_config("default_config.json")
    config = o3d.io.AzureKinectSensorConfig()
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
    color = rgbd.color
    # 获取深度图像
    depth = rgbd.depth

    # 可视化RGB图像
    plt.subplot(1, 2, 1)
    plt.title('RGB Image')
    plt.imshow(color)

    # 可视化深度图像
    plt.subplot(1, 2, 2)
    plt.title('Depth Image')
    plt.imshow(depth, cmap='gray')

    plt.show()
    ```
  + （可选）安装pykinect_azure:
    + 用git下载第三方包
    ```
    git clone https://github.com/ibaiGorordo/pyKinectAzure.git
    ```
    + 然后将第三方包放入python环境第三方包sit-packages文件夹下
    + 示例代码：
    ```python
    import cv2
    import pykinect_azure as pykinect
    
    
    # 初始化库，可以库的路径作为参数
    pykinect.initialize_libraries()
    
    # 修改相机配置
    device_config = pykinect.default_configuration  # 默认初始配置
    device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_2160P  # 修改分辨率
    
    # 开启设备
    device = pykinect.start_device(config=device_config)
    
    while True:
      capture = device.update()  # 更新获取
      ret, rgb_img = capture.get_color_image()
      # rgb_img = cv2.resize(rgb_img, fx=0.5, fy=0.5)
      if not ret:
        continue
      cv2.imshow('rgb image', rgb_img)
      cv2.imwrite('216p.jpg', rgb_img)
      if cv2.waitKey(1) == ord('q'):
        break
    ```