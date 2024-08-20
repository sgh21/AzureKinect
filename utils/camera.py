# Function: Get the camera intrinsics of Azure Kinect DK use pykinect_azure
# Author: Shen Guang-hui
# Create date: 2024-08-15
# Version: 1.0
# Environment: Windows 11, Python 3.8.0

import pykinect_azure as pykinect

resolution = {
    'K4A_COLOR_RESOLUTION_OFF': {'key': pykinect.K4A_COLOR_RESOLUTION_OFF, 'width': 0, 'height': 0},
    'K4A_COLOR_RESOLUTION_720P': {'key': pykinect.K4A_COLOR_RESOLUTION_720P, 'width': 1280, 'height': 720},
    'K4A_COLOR_RESOLUTION_1080P': {'key': pykinect.K4A_COLOR_RESOLUTION_1080P, 'width': 1920, 'height': 1080},
    'K4A_COLOR_RESOLUTION_1440P': {'key': pykinect.K4A_COLOR_RESOLUTION_1440P, 'width': 2560, 'height': 1440},
    'K4A_COLOR_RESOLUTION_1536P': {'key': pykinect.K4A_COLOR_RESOLUTION_1536P, 'width': 2048, 'height': 1536},
    'K4A_COLOR_RESOLUTION_2160P': {'key': pykinect.K4A_COLOR_RESOLUTION_2160P, 'width': 3840, 'height': 2160},
    'K4A_COLOR_RESOLUTION_3072P': {'key': pykinect.K4A_COLOR_RESOLUTION_3072P, 'width': 4096, 'height': 3072}
}
depth_mode_dict = {
    'K4A_DEPTH_MODE_OFF': pykinect.K4A_DEPTH_MODE_OFF,
    'K4A_DEPTH_MODE_NFOV_2X2BINNED': pykinect.K4A_DEPTH_MODE_NFOV_2X2BINNED,
    'K4A_DEPTH_MODE_NFOV_UNBINNED': pykinect.K4A_DEPTH_MODE_NFOV_UNBINNED,
    'K4A_DEPTH_MODE_WFOV_2X2BINNED': pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED,
    'K4A_DEPTH_MODE_WFOV_UNBINNED' : pykinect.K4A_DEPTH_MODE_WFOV_UNBINNED,
    'K4A_DEPTH_MODE_PASSIVE_IR ': pykinect.K4A_DEPTH_MODE_PASSIVE_IR
}


def get_intrinsics(color_resolution:str='K4A_COLOR_RESOLUTION_720P', depth_mode:str='K4A_DEPTH_MODE_NFOV_UNBINNED'):
    # 初始化库
    pykinect.initialize_libraries()

    # 配置相机
    device_config = pykinect.default_configuration
    device_config.color_format = pykinect.K4A_IMAGE_FORMAT_COLOR_BGRA32
    device_config.color_resolution = resolution[color_resolution]['key']
    device_config.depth_mode = depth_mode_dict[depth_mode]

    # 启动设备
    device = pykinect.start_device(config=device_config)

    # 获取相机内参
    calibration = device.get_calibration(device_config.depth_mode, device_config.color_resolution)

    # 将相机内参转换为字典
    intrinsics_dict = {
        'width': resolution[color_resolution]['width'],
        'height': resolution[color_resolution]['height'],
        'cx':calibration.color_params.cx,
        'cy':calibration.color_params.cy,
        'fx':calibration.color_params.fx,
        'fy':calibration.color_params.fy,
        'k1':calibration.color_params.k1,
        'k2':calibration.color_params.k2,
        'k3':calibration.color_params.k3,
        'k4':calibration.color_params.k4,
        'k5':calibration.color_params.k5,
        'k6':calibration.color_params.k6,
        'codx':calibration.color_params.codx,
        'cody':calibration.color_params.cody,
        'p2':calibration.color_params.p2,
        'p1':calibration.color_params.p1,
        'metric_radius':calibration.color_params.metric_radius
    }
    # print(intrinsics_dict)
    return intrinsics_dict

if __name__ == '__main__':
    get_intrinsics('K4A_COLOR_RESOLUTION_720P', 'K4A_DEPTH_MODE_WFOV_2X2BINNED')