# save data
# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node
# import numpy as np
# import math
# import time
# from threading import Thread, Event
# from collections import defaultdict

# from .robotiq_gripper import RobotiqGripper
# from spnav import spnav_open, spnav_poll_event, spnav_close, SpnavMotionEvent, SpnavButtonEvent

# import message_filters
# from sensor_msgs.msg import Image
# from common_msgs.msg import ArmDataCollect

# from rtde_control import RTDEControlInterface
# from rtde_receive import RTDEReceiveInterface
# from rtde_io import RTDEIOInterface as RTDEIO

# class Spacemouse(Thread):
#     def __init__(self, scale_factor=0.3, max_value=500, deadzone=0.0, dtype=np.float32):
#         super().__init__()
#         self.stop_event = Event()
#         self.max_value = max_value
#         self.scale_factor = scale_factor
#         self.dtype = dtype
#         self.deadzone = np.full(6, fill_value=deadzone, dtype=dtype) if np.issubdtype(type(deadzone), np.number) else np.array(deadzone, dtype=dtype)
#         self.motion_event = SpnavMotionEvent([0,0,0], [0,0,0], 0)
#         self.button_state = defaultdict(lambda: False)
#         self.tx_zup_spnav = np.array([
#             [0,0,-1],
#             [1,0,0],
#             [0,1,0]
#         ], dtype=dtype)

#     def get_motion_state_transformed(self):
#         state = np.array(self.motion_event.translation + self.motion_event.rotation, dtype=self.dtype) / self.max_value
#         is_dead = (-self.deadzone < state) & (state < self.deadzone)
#         state[is_dead] = 0

#         tf_state = np.zeros_like(state)
#         tf_state[:3] = self.tx_zup_spnav @ state[:3]
#         tf_state[3:] = self.tx_zup_spnav @ state[3:]
#         tf_state[np.abs(tf_state) < 0.3] = 0
#         return tf_state * self.scale_factor

#     def is_button_pressed(self, button_id):
#         return self.button_state[button_id]

#     def run(self):
#         spnav_open()
#         try:
#             while not self.stop_event.is_set():
#                 event = spnav_poll_event()
#                 if isinstance(event, SpnavMotionEvent):
#                     self.motion_event = event
#                 elif isinstance(event, SpnavButtonEvent):
#                     self.button_state[event.bnum] = event.press
#                 else:
#                     time.sleep(0.005)  # 提高检测频率
#         finally:
#             spnav_close()

#     def stop(self):
#         self.stop_event.set()
#         self.join()

# class SpacemouseControlNode(Node):
#     def __init__(self):
#         super().__init__('spacemouse_control_node')

#         # Robot配置
#         self.robot_ip = "192.168.1.50"  # 单臂
#         self.rtde_c = RTDEControlInterface(self.robot_ip)
#         self.rtde_r = RTDEReceiveInterface(self.robot_ip)
#         self.rtde_io = RTDEIO(self.robot_ip)

#         self.gripper = RobotiqGripper()
#         self.gripper.connect(self.robot_ip, 63352)
#         self.gripper.activate()

#         self.spacemouse = Spacemouse()
#         self.spacemouse.start()

#         # 创建数据发布器
#         self.arm_data_pub = self.create_publisher(ArmDataCollect, "/spacemouse_arm_data_cmd", 10)

#         # 同步右相机+顶相机
#         self.right_image_sub = message_filters.Subscriber(self, Image, '/camera/color/image_rect_raw')
#         self.top_image_sub = message_filters.Subscriber(self, Image, '/camera/color/image_raw')
#         self.ts = message_filters.ApproximateTimeSynchronizer([self.right_image_sub, self.top_image_sub], 10, 0.02)
#         self.ts.registerCallback(self.image_callback)

#         # 定时器：每33ms处理一次运动与控制（30Hz）
#         self.timer = self.create_timer(0.033, self.motion_control_loop)

#         # 夹爪控制
#         self.gripper_direction = 0
#         self.gripper_step = 3  # 更细腻地控制开合

#         # 临时保存最新图像
#         self.right_image_msg = None
#         self.top_image_msg = None

#         self.get_logger().info("Spacemouse Control Node Initialized at 30Hz")

    # def is_near_singularity(rtde_r):
    #     """检测是否接近奇异点（示例：肘部或腕部接近0或pi）"""
    #     q_actual = rtde_r.getActualQ()
    #     elbow_angle = q_actual[2]  # 第3轴（elbow）
    #     # wrist_2_angle = q_actual[4]  # 第5轴（wrist2）

    #     # 判断接近0或pi的情况
    #     near_elbow = abs(elbow_angle) < 0.1
    #     # near_wrist2 = abs(wrist_2_angle) < 0.1 or abs(abs(wrist_2_angle) - np.pi) < 0.1

    #     # return near_elbow or near_wrist2
    #     return near_elbow

#     def image_callback(self, right_image_msg, top_image_msg):
#         """保存同步图像"""
#         self.right_image_msg = right_image_msg
#         self.top_image_msg = top_image_msg

#     def motion_control_loop(self):
#         """周期运动控制 + 采集数据"""

#         if self.rtde_r.getRobotMode() != 7:
#             return  # Robot未准备好时直接跳过

#         # --- 控制机器人运动 ---
#         motion_state = self.spacemouse.get_motion_state_transformed()
        # if is_near_singularity(rtde_r):
            # print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
            # self.rtde_c.speedStop()
            # continue  # 跳过当前循环，避免再执行正常指令
#         self.rtde_c.speedL(motion_state.tolist(), acceleration=1.5, time=0.1)

#         # --- 夹爪控制 ---
#         gripper = self.gripper
#         current_pos = gripper.get_current_position()

#         if self.spacemouse.is_button_pressed(0):
#             new_pos = min(current_pos + self.gripper_step, 255)
#             gripper.move(new_pos, 100, 100)
#         elif self.spacemouse.is_button_pressed(1):
#             new_pos = max(current_pos - self.gripper_step, 0)
#             gripper.move(new_pos, 100, 100)

#         # --- 发布数据 ---
#         data_msg = ArmDataCollect()
#         now = self.get_clock().now().to_msg()

#         tcp_pose = self.rtde_r.getActualTCPPose()
#         data_msg.pose.header.stamp = now
#         data_msg.pose.header.frame_id = "base_link"
#         data_msg.pose.pose.position.x = tcp_pose[0]
#         data_msg.pose.pose.position.y = tcp_pose[1]
#         data_msg.pose.pose.position.z = tcp_pose[2]

#         qx, qy, qz, qw = self.axis_angle_to_quaternion(tcp_pose[3], tcp_pose[4], tcp_pose[5])
#         data_msg.pose.pose.orientation.x = qx
#         data_msg.pose.pose.orientation.y = qy
#         data_msg.pose.pose.orientation.z = qz
#         data_msg.pose.pose.orientation.w = qw

#         data_msg.gripper_distance = gripper.get_current_position() / 255.0

#         data_msg.joint_name = [
#             "shoulder_pan_joint", "shoulder_lift_joint",
#             "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
#         ]
#         data_msg.joints_angle = self.rtde_r.getActualQ()

#         if self.right_image_msg and self.top_image_msg:
#             data_msg.right_image = self.right_image_msg
#             data_msg.top_image = self.top_image_msg

#         self.arm_data_pub.publish(data_msg)

#     def axis_angle_to_quaternion(self, rx, ry, rz):
#         """轴角转四元数"""
#         angle = math.sqrt(rx**2 + ry**2 + rz**2)
#         if angle < 1e-6:
#             return [0.0, 0.0, 0.0, 1.0]
#         axis = [rx/angle, ry/angle, rz/angle]
#         half_angle = angle / 2.0
#         sin_half = math.sin(half_angle)
#         return [axis[0]*sin_half, axis[1]*sin_half, axis[2]*sin_half, math.cos(half_angle)]

#     def __del__(self):
#         """资源清理"""
#         self.rtde_c.stopScript()
#         self.spacemouse.stop()

# def main(args=None):
#     rclpy.init(args=args)
#     node = SpacemouseControlNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()






# spacemouse test
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface 
from rtde_io import RTDEIOInterface as RTDEIO
from .robotiq_gripper import RobotiqGripper
from spnav import spnav_open, spnav_poll_event, spnav_close, SpnavMotionEvent, SpnavButtonEvent
from threading import Thread, Event
from collections import defaultdict
import numpy as np
import time

class Spacemouse(Thread):
    def __init__(self, max_value=500, deadzone=(0,0,0,0,0,0), dtype=np.float32):
        """
        Continuously listen to 3D connection space naviagtor events
        and update the latest state.

        max_value: {300, 500} 300 for wired version and 500 for wireless
        deadzone: [0,1], number or tuple, axis with value lower than this value will stay at 0
        
        front
        z
        ^   _
        |  (O) space mouse
        |
        *----->x right
        y
        """
        if np.issubdtype(type(deadzone), np.number):
            deadzone = np.full(6, fill_value=deadzone, dtype=dtype)
        else:
            deadzone = np.array(deadzone, dtype=dtype)
        assert (deadzone >= 0).all()

        super().__init__()
        self.stop_event = Event()
        self.max_value = max_value
        self.dtype = dtype
        self.deadzone = deadzone
        self.motion_event = SpnavMotionEvent([0,0,0], [0,0,0], 0)
        self.button_state = defaultdict(lambda: False)
        self.tx_zup_spnav = np.array([
            [0,0,-1],
            [1,0,0],
            [0,1,0]
        ], dtype=dtype)

    def get_motion_state(self): #this method gets the movement of the mouse 
        me = self.motion_event
        state = np.array(me.translation + me.rotation, 
            dtype=self.dtype) / self.max_value
        is_dead = (-self.deadzone < state) & (state < self.deadzone)
        state[is_dead] = 0
        return state
    
    def get_motion_state_transformed(self): #transforms get_motion_state 
        """
        Return in right-handed coordinate
        z
        *------>y right
        |   _
        |  (O) space mouse
        v
        x
        back

        """
        state = self.get_motion_state()
        tf_state = np.zeros_like(state)
        tf_state[:3] = self.tx_zup_spnav @ state[:3]
        tf_state[3:] = self.tx_zup_spnav @ state[3:]

        # Set values lesser than 0.3 to 0 for better control
        tf_state[np.abs(tf_state) < 0.3] = 0
        tf_state = tf_state * SCALE_FACTOR

        return tf_state

    def is_button_pressed(self, button_id):
        return self.button_state[button_id]

    def stop(self):
        self.stop_event.set()
        self.join()

    def __enter__(self):
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    def run(self):
        spnav_open()
        try:
            while not self.stop_event.is_set():
                event = spnav_poll_event()
                if isinstance(event, SpnavMotionEvent):
                    self.motion_event = event
                elif isinstance(event, SpnavButtonEvent):
                    self.button_state[event.bnum] = event.press
                else:
                    time.sleep(1/200)
        finally:
            spnav_close()

# Define robot parameters
ROBOT_HOST = "192.168.1.50"  # IP address of the robot controller
SCALE_FACTOR = 0.3 # Scale factor for velocity command


# no use
def is_near_singularity(rtde_r):
    """检测是否接近奇异点（示例：肘部或腕部接近0或pi）"""
    q_actual = rtde_r.getActualQ()
    elbow_angle = q_actual[2]  # 第3轴（elbow）
    # wrist_2_angle = q_actual[4]  # 第5轴（wrist2）

    # 判断接近0或pi的情况
    near_elbow = abs(elbow_angle) < 0.1
    # near_wrist2 = abs(wrist_2_angle) < 0.1 or abs(abs(wrist_2_angle) - np.pi) < 0.1

    # return near_elbow or near_wrist2
    return near_elbow



def main():
    sm = Spacemouse()
    sm.start()
    # Initialize RTDEControlInterface
    rtde_c = RTDEControlInterface(ROBOT_HOST)
    rtde_r = RTDEReceiveInterface(ROBOT_HOST)
    rtde_io = RTDEIO(ROBOT_HOST)
    
    print("Creating gripper...")
    gripper = RobotiqGripper()
    print("Connecting to gripper...")
    gripper.connect(ROBOT_HOST, 63352)
    print("Activating gripper...")
    gripper.activate()
    gripper_position = gripper.get_current_position()
    try:
        while True:
            if rtde_r.getRobotMode() == 7:
                if is_near_singularity(rtde_r):
                    print("@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@")
                    rtde_c.speedStop()
                    continue  # 跳过当前循环，避免再执行正常指令

                # Read motion state from SpaceMouse
                motion_state = sm.get_motion_state_transformed()
                print("motion_state: ", motion_state)
                
                #send command to robot 
                rtde_c.speedL(motion_state, acceleration = 1.5, time = 0.1) #adjust the acceleration if required 
                if is_near_singularity(rtde_r):
                    print("[!] Robot is near singularity after movement.")
                    rtde_c.speedStop()  # Stop robot in case it entered a singularity after moving
                    continue  # Skip to next loop iteration

                #get TCP velocity of robot
                actual_velocity = rtde_r.getActualTCPSpeed()
                actual_velocity = [0 if abs(x) < 0.01 else x for x in actual_velocity] #filter out extremely small numbers
                print("Current velocity vector" , actual_velocity)

                #get TCP pose of robot
                #actual_pose = rtde_r.getActualTCPPose()
                #print(actual_pose)
  
                if sm.is_button_pressed(0):
                    gripper_position += 3
                    gripper.move(gripper_position, 155, 255)

                if sm.is_button_pressed(1):
                    gripper_position -= 3
                    gripper.move(gripper_position, 155, 255)

                if gripper_position < 0:
                    gripper_position = 0

                if gripper_position > 255:
                    gripper_position = 255

                # print("Gripper Position (0 to 255): ", gripper_position)
    
                #wait awhile before proceeding 
                time.sleep(1/100)

            else:
                print("Robot is not ready.")
                time.sleep(1)  # Wait longer if robot is not ready

    except KeyboardInterrupt:
        # Handle graceful shutdown here
        print("Stopping robot")
        rtde_c.stopScript()
        sm.stop()

if __name__ == "__main__":
    main()
