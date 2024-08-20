### 常用函数库
+ Class Robot(URRobot)
  + methods:
    + movej(self, joints, acc=0.1, vel=0.05, wait=True, relative=False, threshold=None)
      + 在关节空间移动
      + joints:list 6个关节角度，单位是弧度
      + acc: float 最大加速度 (默认值为 0.1 m/s²)
      + vel: float 最大速度 (默认值为 0.05 m/s)
      + wait: bool 是否等待运动到指定位置周围 (默认值为 True)
      + relative:bool 是否为相对当前姿态运动 (默认值为 False)
      + threshold: 运动完成的阈值 (默认值为 None)
    + getj(self, wait=False)
      + 获取关节姿态
      + wait:bool 等待获取下一个数据包(默认值为 False)
    + set_tcp(self, tcp)
      + 设置机器人工具中心坐标相对于法兰的位姿
      + tcp:Tuple (x,y,z,Rx,Ry,Rz)前三个为位置，后三个为姿态
    + movel(self, tpose, acc=0.01, vel=0.01, wait=True, relative=False, threshold=None)
      + 移动tcp末端到指定位置(相对于csys，默认为基座坐标系)
      + tpose:Tuple (x,y,z,Rx,Ry,Rz)前三个为位置，后三个为姿态
      + acc: float 最大加速度 (默认值为 0.1 m/s²)
      + vel: float 最大速度 (默认值为 0.05 m/s)
      + wait: bool 是否等待运动到指定位置周围 (默认值为 True)
      + relative:bool 是否为相对当前姿态运动 (默认值为 False)
      + threshold: 运动完成的阈值 (默认值为 None)
    + getl(self, wait=False, _log=True)->list
      + 返回tcp相对于csys的位姿
      + wait:bool 等待获取下一个数据包(默认值为 False)
      + _log：bool 是否打印位姿(默认值为 True)
    + get_pose(self, wait=False, _log=True)->m3d.Transform
      + 返回tcp相对于csys的位姿变换矩阵
      + wait:bool 等待获取下一个数据包(默认值为 False)
      + _log：bool 是否打印位姿(默认值为 True)
    + close(self)
      + 断开连接
    + set_csys(self, transform)
      + 设置参考坐标系，后续将相对它运动
    + set_orientation(self, orient, acc=0.01, vel=0.01, wait=True, threshold=None)
      + 用orientation向量或者math3d矩阵设置tcp姿态
    + translate(self, vect, acc=0.01, vel=0.01, wait=True, threshold=None)
      + 相对csys移动，并且保持tcp姿态
    + translate_tool(self, vect, acc=0.01, vel=0.01, wait=True, threshold=None)
      + 相对工具坐标系移动，并且保持tcp姿态
    + set_pos(self, vect, acc=0.01, vel=0.01, wait=True, threshold=None)
      + 用math3d向量设置tcp位置，保持姿态
    + set_pose(self, trans, acc=0.01, vel=0.01, wait=True, command="movel", threshold=None)
      + set_pos+set_orientation，使用math3d的trans
    + get_pos(self, wait=False)
      + get tool tip pos(x, y, z) in base coordinate(csys) system
    + get_orientation(self, wait=False)
      + get tool orientation in base coordinate(csys) system
    + speedl(self, velocities, acc, min_time)
      + move at given velocities(in csys) until minimum min_time seconds
    + speedj(self, velocities, acc, min_time)
      + move at given joint velocities until minimum min_time seconds
    + speedl_tool(self, velocities, acc, min_time)
      + move at given velocities in tool csys until minimum min_time seconds
    +  movel_tool(self, pose, acc=0.01, vel=0.01, wait=True, threshold=None)
       +  move linear to given pose in tool coordinate
    +  set_gravity(self, vector)
       + 设置重力方向
     + new_csys_from_xpy(self)->csys
       + 用xoy标定新的坐标系