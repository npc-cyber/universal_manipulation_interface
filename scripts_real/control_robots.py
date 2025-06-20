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
from umi.real_world.spacemouse_shared_memory import Spacemouse
from umi.real_world.rtde_interpolation_controller import RTDEInterpolationController
from umi.real_world.wsg_controller import WSGController
from umi.common.precise_sleep import precise_wait

# %%
@click.command()
@click.option('-rh', '--robot_hostname', default='172.24.95.9')
@click.option('-gh', '--gripper_hostname', default='172.24.95.27')
@click.option('-gp', '--gripper_port', type=int, default=1000)
@click.option('-f', '--frequency', type=float, default=30)
@click.option('-gs', '--gripper_speed', type=float, default=200.0)
def main(robot_hostname, gripper_hostname, gripper_port, frequency, gripper_speed):
    max_pos_speed = 0.25
    max_rot_speed = 0.6
    max_gripper_width = 90.
    cube_diag = np.linalg.norm([1,1,1])
    tcp_offset = 0.13
    # tcp_offset = 0
    dt = 1/frequency
    command_latency = dt / 2

    with SharedMemoryManager() as shm_manager:
        with WSGController(
            shm_manager=shm_manager,
            hostname=gripper_hostname,
            port=gripper_port,
            frequency=frequency,
            move_max_speed=400.0,
            verbose=False
        ) as gripper,\
        RTDEInterpolationController(
            shm_manager=shm_manager,
            robot_ip=robot_hostname,
            frequency=125,
            lookahead_time=0.05,
            gain=1000,
            max_pos_speed=max_pos_speed*cube_diag,
            max_rot_speed=max_rot_speed*cube_diag,
            tcp_offset_pose=[0,0,tcp_offset,0,0,0],
            verbose=False
        ) as controller,\
        Spacemouse(
            shm_manager=shm_manager
        ) as sm:
            print('Ready!')
            # to account for recever interfance latency, use target pose
            # to init buffer.
            state = controller.get_state()
            # 这边是读取目标位姿 然后不断增量
            target_pose = state['TargetTCPPose']
            gripper_target_pos = gripper.get_state()['gripper_position']
            t_start = time.monotonic()
            gripper.restart_put(t_start-time.monotonic() + time.time())
            
            iter_idx = 0
            while True:
                s = time.time()
                t_cycle_end = t_start + (iter_idx + 1) * dt
                t_sample = t_cycle_end - command_latency
                t_command_target = t_cycle_end + dt

                precise_wait(t_sample)
                # 这个地方读取了空间鼠标的状态
                sm_state = sm.get_motion_state_transformed()
                # print(sm_state)
                dpos = sm_state[:3] * (max_pos_speed / frequency)
                drot_xyz = sm_state[3:] * (max_rot_speed / frequency)

                drot = st.Rotation.from_euler('xyz', drot_xyz)
                target_pose[:3] += dpos
                target_pose[3:] = (drot * st.Rotation.from_rotvec(
                    target_pose[3:])).as_rotvec()
                
                dpos = 0
                if sm.is_button_pressed(0):
                    # close gripper
                    dpos = -gripper_speed / frequency
                if sm.is_button_pressed(1):
                    dpos = gripper_speed / frequency
                gripper_target_pos = np.clip(gripper_target_pos + dpos, 0, max_gripper_width)
                
                # 应该是此处更新了target pose
                controller.schedule_waypoint(target_pose, 
                    t_command_target-time.monotonic()+time.time())
                gripper.schedule_waypoint(gripper_target_pos, 
                    t_command_target-time.monotonic()+time.time())

                precise_wait(t_cycle_end)
                iter_idx += 1

# %%
if __name__ == '__main__':
    main()