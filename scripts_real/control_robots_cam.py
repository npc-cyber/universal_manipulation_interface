# %%
import sys
import os

ROOT_DIR = os.path.dirname(os.path.dirname(__file__))
print(ROOT_DIR)
sys.path.append(ROOT_DIR)
os.chdir(ROOT_DIR)

# %%
import click
import time
import numpy as np
from multiprocessing.managers import SharedMemoryManager
import scipy.spatial.transform as st
from umi.real_world.rtde_interpolation_controller import RTDEInterpolationController
from umi.common.precise_sleep import precise_wait
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class TwistSubscriber(Node):
    def __init__(self):
        super().__init__("twist_subscriber")
        self.subscription = self.create_subscription(
            TwistStamped, "/left_servo/delta_twist_cmds", self.callback_twist, 1
        )
        self.init_tcp_pose = np.array(
            [
                -0.29630044,
                -0.12824886,
                0.16598541,
                -2.25567639,
                -2.18396001,
                -0.01605237,
            ]
        )
        self.latest_twist = None
        self.robot_limit = 0.05

    def callback_twist(self, msg):
        self.latest_twist = msg.twist
        self.target_tcp_pose = np.array(
            [
                self.latest_twist.linear.x,
                self.latest_twist.linear.y,
                self.latest_twist.linear.z,
                self.latest_twist.angular.x,
                self.latest_twist.angular.y,
                self.latest_twist.angular.z,
            ]
        )
        self.target_tcp_pose[:3] = np.clip(
                    self.target_tcp_pose[:3],
                    self.init_tcp_pose[:3] - self.robot_limit,  # 下限
                    self.init_tcp_pose[:3] + self.robot_limit,  # 上限
                )
        self.target_tcp_pose[3:] = self.init_tcp_pose[3:]
        print(self.target_tcp_pose)


def Ur_Robot_Msg_to_T(target_pose):
    position = target_pose[:3]
    rotvec = target_pose[3:]

    # 构建4x4齐次变换矩阵
    T = np.eye(4)
    T[:3, :3] = st.Rotation.from_rotvec(rotvec).as_matrix()
    T[:3, 3] = position
    return T


def T_to_Ur_Robot_Msg(T):
    position = T[:3, 3]
    rotvec = st.Rotation.from_matrix(T[:3, :3]).as_rotvec()
    return np.concatenate([position, rotvec])


# %%
@click.command()
@click.option("-rh", "--robot_hostname", default="192.168.1.50")
@click.option("-f", "--frequency", type=float, default=30)
def main(robot_hostname, frequency):
    # 初始化ROS2
    rclpy.init()
    twist_node = TwistSubscriber()

    max_pos_speed = 0.1
    max_rot_speed = 0.6
    cube_diag = np.linalg.norm([1, 1, 1])
    tcp_offset = 0.13
    dt = 1 / frequency
    command_latency = dt / 2

    with SharedMemoryManager() as shm_manager:
        with RTDEInterpolationController(
            shm_manager=shm_manager,
            robot_ip=robot_hostname,
            frequency=500,# spacemouse ur 500hz gripper 125hz 
            lookahead_time=0.05,
            gain=1000,
            max_pos_speed=max_pos_speed * cube_diag,
            max_rot_speed=max_rot_speed * cube_diag,
            tcp_offset_pose=[0, 0, tcp_offset, 0, 0, 0],
            verbose=False,
            joints_init = [0,-1.57,1.57,-1.57,-1.57,0],# 初始值
        ) as controller:
            print("Ready!")
            state = controller.get_state()

            t_start = time.monotonic()
            iter_idx = 0

            while True:
                s = time.time()
                t_cycle_end = t_start + (iter_idx + 1) * dt
                t_sample = t_cycle_end - command_latency
                t_command_target = t_cycle_end + dt

                precise_wait(t_sample)

                # 处理ROS2消息（非阻塞方式）
                rclpy.spin_once(twist_node, timeout_sec=0)
                current_rotvec = np.array([0.0, 0.0, 0.0])
                current_translation = np.array([0.0, 0.0, 0.00])
                state = controller.get_state()
                current_pose = state["ActualTCPPose"]

                if twist_node.latest_twist is not None:
                    # 目标位置
                    target_tcp_pose = twist_node.target_tcp_pose.copy()
                    # 反向试一下 可能正常一些
                    current_translation = (target_tcp_pose - current_pose)[:3]
                    # 归一化
                    if np.linalg.norm(current_translation) > 0.000001:
                        current_translation = current_translation / np.linalg.norm(
                            current_translation
                        )
                        current_translation = current_translation * (
                            max_pos_speed / frequency
                        )
                # print(current_translation)
                # 当前位姿
                T_tcp1_to_base = Ur_Robot_Msg_to_T(current_pose)
                # tcp input
                T_tcp2_to_tcp1 = Ur_Robot_Msg_to_T(
                    np.concatenate([current_translation, current_rotvec])
                )
                # print(T_tcp2_to_tcp1)
                T_tcp2_to_base = T_tcp1_to_base @ T_tcp2_to_tcp1
                target_pose = T_to_Ur_Robot_Msg(T_tcp2_to_base)
                if twist_node.latest_twist:
                    target_pose = twist_node.target_tcp_pose

                # 这里可以添加你自己的控制逻辑
                # 例如：target_pose = ... 更新目标位姿
                # target_pose = [x,y,z,rx,ry,rz]
                # print(np.round(Ur_Robot_Msg_to_T(target_pose), 5))
                controller.schedule_waypoint(
                    target_pose,
                    t_command_target - time.monotonic() + time.time(),
                )
                precise_wait(t_cycle_end)
                iter_idx += 1
                # print(1 / (time.time() - s))


# %%
if __name__ == "__main__":
    main()
