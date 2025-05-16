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
            TwistStamped, "/ur_servo/target_pose", self.callback_twist, 1
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
        self.prev_target_tcp_pose = None
        self.target_tcp_pose = None
        self.robot_limit = 0.05

    def callback_twist(self, msg):
        target_tcp_pose = np.array(
            [
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.linear.z,
                msg.twist.angular.x,
                msg.twist.angular.y,
                msg.twist.angular.z,
            ]
        )
        target_tcp_pose[:3] = np.clip(
            target_tcp_pose[:3],
            self.init_tcp_pose[:3] - self.robot_limit,  # 下限
            self.init_tcp_pose[:3] + self.robot_limit,  # 上限
        )
        if self.prev_target_tcp_pose is not None:
            dist = np.linalg.norm(self.prev_target_tcp_pose[:3] - target_tcp_pose[:3])
            if dist > 0.05:
                self.target_tcp_pose = target_tcp_pose
        else:
            self.target_tcp_pose = target_tcp_pose
        self.prev_target_tcp_pose = self.target_tcp_pose

# 这个地方把周期改小就好了 我觉得
# %%
@click.command()
@click.option("-rh", "--robot_hostname", default="192.168.1.50")
@click.option("-f", "--frequency", type=float, default=10)
def main(robot_hostname, frequency):
    # 初始化ROS2
    rclpy.init()
    twist_node = TwistSubscriber()

    max_pos_speed = 0.10
    max_rot_speed = 0.25
    cube_diag = np.linalg.norm([1, 1, 1])
    tcp_offset = 0.13
    dt = 1 / frequency
    command_latency = dt / 2

    with SharedMemoryManager() as shm_manager:
        with RTDEInterpolationController(
            shm_manager=shm_manager,
            robot_ip=robot_hostname,
            frequency=125,  # spacemouse ur 500hz gripper 125hz
            lookahead_time=0.05,
            gain=1000,
            max_pos_speed=max_pos_speed * cube_diag,
            max_rot_speed=max_rot_speed * cube_diag,
            tcp_offset_pose=[0, 0, tcp_offset, 0, 0, 0],
            verbose=False,
            joints_init=[0, -1.57, 1.57, -1.57, -1.57, 0],  # 初始值
        ) as controller:
            print("Ready!")
            state = controller.get_state()
            current_tcp_pose = state["ActualTCPPose"]
            print("current_tcp_pose", current_tcp_pose)

            t_start = time.monotonic()
            iter_idx = 0

            while True:
                s = time.time()
                t_cycle_end = t_start + (iter_idx + 1) * dt
                t_sample = t_cycle_end - command_latency
                t_command_target = t_cycle_end + dt

                # 处理ROS2消息（非阻塞方式）
                rclpy.spin_once(twist_node, timeout_sec=0)
                target_pose = None
                if twist_node.target_tcp_pose is not None:
                    target_pose = twist_node.target_tcp_pose.copy()

                precise_wait(t_sample)
                # target_pose = [x,y,z,rx,ry,rz]
                # 这个地方大概率是在本次循环到 target_pose
                # 所以周期越长越好 中间点会自己插值
                # RTDEInterpolationController 有自己的内部周期 中间点会自己插值
                if target_pose is not None:
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
