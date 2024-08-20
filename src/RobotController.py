from urx import Robot

import time
import numpy as np
from scipy.spatial.transform import Rotation as Rsci

class robot(object):
    def __init__(self):
        self.grip_ur5 = None
        self.grip_ur5e = None
        self.robot_ur5 = None
        self.robot_ur5e = None
        self.target_ip_ur5 = "192.168.1.10"  # robot left
        self.target_ip_ur5e = "192.168.1.55"  # robot right

    def ur5e_connect(self):
        # 实例化机器人类，usr_rt=True表示使用实时控制
        self.robot_ur5e = Robot(self.target_ip_ur5e, use_rt=True)
        time.sleep(0.5)
        # 设置工具坐标系中心的位置和姿态
        self.robot_ur5e.set_tcp((0, 0, 0, 0, 0, 0))  # 设置TCP参数 m
        # self.robot_ur5e.set_payload(2.33, (4, 0, 75))

    def ur5_connect(self):
        self.robot_ur5 = Robot(self.target_ip_ur5, use_rt=True)
        time.sleep(0.5)
        self.robot_ur5.set_tcp((0, 0, 0, 0, 0, 0))  # 设置TCP参数 m

    def movej_both_sendsignal(self, target_ur5e_j, target_ur5_j, v_ur5e, v_ur5):
        self.robot_ur5.movej(target_ur5_j, acc=1, vel=v_ur5, wait=False)
        self.robot_ur5e.movej(target_ur5e_j, acc=1, vel=v_ur5e, wait=False)

    def movej_both(self, target_ur5e_j, target_ur5_j, v_ur5e, v_ur5):
        self.robot_ur5.movej(target_ur5_j, acc=1, vel=v_ur5, wait=False)
        self.robot_ur5e.movej(target_ur5e_j, acc=1, vel=v_ur5e, wait=False)

        Get = 0
        while Get < 1:
            Con_Jpos1 = self.robot_ur5e.getj()
            Con_read1 = Con_Jpos1

            Con_Jpos2 = self.robot_ur5.getj()
            Con_read2 = Con_Jpos2

            Diff = 0.0
            for j in range(6):
                Diff += abs(Con_read1[j] - target_ur5e_j[j])
            for j in range(6):
                Diff += abs(Con_read2[j] - target_ur5_j[j])

            if Diff < 0.0002 * 2:
                Get += 1
                return 1

        return 0

    def movel_both(self, target_ur5e_p, target_ur5_p, v_ur5e, v_ur5):
        self.robot_ur5.movel(target_ur5_p, acc=1.5, vel=v_ur5, wait=False)
        self.robot_ur5e.movel(target_ur5e_p, acc=1.5, vel=v_ur5e, wait=False)

        Get = 0
        while Get < 1:
            Con_Cpos1 = self.robot_ur5e.getl()
            Con_read1 = Con_Cpos1

            Con_Cpos2 = self.robot_ur5.getl()
            Con_read2 = Con_Cpos2

            Diff = 0.0
            for j in range(6):
                Diff += abs(Con_read1[j] - target_ur5e_p[j])
            for j in range(6):
                Diff += abs(Con_read2[j] - target_ur5_p[j])

            if Diff < 0.0002 * 2:
                Get += 1
                return 1

        return 0

    def get_dualarm_pose(self):
        # 相对于左边机械臂基座坐标系下的位姿
        pose1 = self.robot_ur5.get_pose()
        pose2 = self.robot_ur5e.get_pose()
        p1 = pose1.pos
        p2 = pose2.pos
        pos1 = np.array([p1[0], p1[1], p1[2]])
        pos2 = np.array([0.95 + p2[0], p2[1], p2[2]])
        orn1 = pose1.orient
        orn2 = pose2.orient

        # 将旋转向量转换为四元数
        R1 = [[orn1[0, 0], orn1[0, 1], orn1[0, 2]],
              [orn1[1, 0], orn1[1, 1], orn1[1, 2]],
              [orn1[2, 0], orn1[2, 1], orn1[2, 2]]]
        rotation1 = Rsci.from_matrix(R1)
        quat1_ = rotation1.as_quat()  # [x, y, z, w] 格式的四元数
        # print("Quaternion1:", quat1)

        # 将旋转向量转换为四元数
        R2 = [[orn2[0, 0], orn2[0, 1], orn2[0, 2]],
              [orn2[1, 0], orn2[1, 1], orn2[1, 2]],
              [orn2[2, 0], orn2[2, 1], orn2[2, 2]]]
        rotation2 = Rsci.from_matrix(R2)
        quat2_ = rotation2.as_quat()  # [x, y, z, w] 格式的四元数
        # print("Quaternion2:", quat2)

        # 转为wxyz型
        quat1 = [quat1_[3], quat1_[0], quat1_[1], quat1_[2]]
        quat2 = [quat2_[3], quat2_[0], quat2_[1], quat2_[2]]

        # print("pos1", pos1)
        # print("pos2", pos2)
        return pos1, pos2, R1, R2, quat1, quat2

    def disconnect(self):
        self.robot_ur5e.close()
        self.robot_ur5.close()


global robot_control
robot_control = robot()

if __name__ == '__main__':
    robot_control.ur5_connect()
    robot_control.ur5e_connect()

    print(robot_control.robot_ur5.getj())
    print(robot_control.robot_ur5e.getj())

    time.sleep(2)  # 给发送命令点时间

    
    # original pose
    Current_Cpos_5 = [-2.8042081038104456, -3.147838894520895, 1.6517605781555176, -1.559535328541891, -0.3580845038043421, -1.700019661580221]
    Current_Cpos_5e = [-0.7628853956805628, 0.20613591253247066, -1.8010778427124023, -1.5504778188518067, 0.7470250129699707, 1.5594158172607422]

    robot_control.movej_both(Current_Cpos_5e, Current_Cpos_5, 0.15, 0.15)

    robot_control.disconnect()
