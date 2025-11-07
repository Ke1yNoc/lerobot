# Piper SDK interface for LeRobot integration

import time
from typing import Any
from pika.gripper import Gripper
import math

from piper_sdk import C_PiperInterface_V2

class PiperSDKInterface:
    def __init__(self, port: str, gripper_serial: str):
        if C_PiperInterface_V2 is None:
            raise ImportError("piper_sdk is not installed. Please install it with `pip install piper_sdk`.")
        try:
            self.piper = C_PiperInterface_V2(port, dh_is_offset=1)
        except Exception as e:
            print(
                f"Failed to initialize Piper SDK: {e} Did you activate the can interface with `piper_sdk/can_activate.sh can0 1000000`"
            )
            self.piper = None
            return
        self.piper.ConnectPort()
        time.sleep(0.1)  # wait for connection to establish

        # reset the arm if it's not in idle state
        if self.piper.GetArmStatus().arm_status.motion_status != 0:
            self.piper.EmergencyStop(0x02)  # resume

        if self.piper.GetArmStatus().arm_status.ctrl_mode == 2:
            print("The arm is in teaching mode, the light is green, press the button to exit teaching mode.")
            self.piper.EmergencyStop(0x02)  # resume

        while( not self.piper.EnablePiper()):
            time.sleep(0.01)
        print("Piper Enabled!")
        self.piper.ModeCtrl(0x01, 0x00, 100, 0x00)
        # Set motion control to joint mode at 100% speed
        # self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)

        # Get the min and max positions for each joint and gripper
        angel_status = self.piper.GetAllMotorAngleLimitMaxSpd()
        self.min_pos = [
            pos.min_angle_limit for pos in angel_status.all_motor_angle_limit_max_spd.motor[1:7]
        ] + [0]
        self.max_pos = [
            pos.max_angle_limit for pos in angel_status.all_motor_angle_limit_max_spd.motor[1:7]
        ] + [10]  # Gripper max position in mm

        # Connect to the pika gripper
        self.gripper = Gripper(gripper_serial)
        self.tool_offset = (0, 0, 0.1773)

        if not self.gripper.connect():
            print("连接 Pika Gripper 设备失败，请检查设备连接和串口路径")
            return
        
        print("Pika Gripper Connected!",gripper_serial)
        if not self.gripper.enable():
            print("电机启用失败")
            return
        print("电机启用成功")
            

    def set_joint_positions(self, positions):
        # positions: list of 7 floats, first 6 are joint and 7 is gripper position (distance in mm)
        # positions are in -100% to 100% range, we need to map them on the min and max positions
        # so -100% is min_pos and 100% is max_pos
        # self.piper.MotionCtrl_2(0x01, 0x01, 100, 0x00)
        scaled_positions = [
            self.min_pos[i] + (self.max_pos[i] - self.min_pos[i]) * (pos + 100) / 200
            for i, pos in enumerate(positions[:6])
        ]
        scaled_positions = [100.0 * pos for pos in scaled_positions]  # Adjust factor

        # the gripper is from 0 to 100% range
        scaled_positions.append(self.min_pos[6] + (self.max_pos[6] - self.min_pos[6]) * positions[6] / 100)
        scaled_positions[6] = int(scaled_positions[6] * 10000)  # Convert to mm

        # joint 0, 3 and 5 are inverted
        joint_0 = int(-scaled_positions[0])
        joint_1 = int(scaled_positions[1])
        joint_2 = int(scaled_positions[2])
        joint_3 = int(-scaled_positions[3])
        joint_4 = int(scaled_positions[4])
        joint_5 = int(-scaled_positions[5])
        joint_6 = int(scaled_positions[6])

        self.piper.JointCtrl(joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
        print("Setting joint position:",joint_0, joint_1, joint_2, joint_3, joint_4, joint_5)
        # Set gripper ctrl with pika gripper
        self.gripper.set_gripper_distance(positions[6])

    def set_end_position(self, positions):
        # positions: list of 7 floats, first 6 are joint and 7 is gripper position (distance in mm)
        factor = 1000
        # self.piper.MotionCtrl_2(0x01, 0x00, 100, 0x00)
        X = round(positions[0] * factor)
        Y = round(positions[1] * factor)
        Z = round(positions[2] * factor)
        RX = round(positions[3] * factor)
        RY = round(positions[4] * factor)
        RZ = round(positions[5] * factor)
        eef_status = (X,Y,Z,RX,RY,RZ)
        x6, y6, z6 = self.remove_tool_offset(eef_status, self.tool_offset)

        print("Setting end position:",X,Y,Z,RX,RY,RZ)
        print("Setting end position:",x6,y6,z6,RX,RY,RZ)
        self.piper.EndPoseCtrl(x6,y6,z6,RX,RY,RZ)
        # Set gripper ctrl with pika gripper
        print("Setting Gripper distance:",positions[6])
        self.gripper.set_gripper_distance(positions[6])

    def get_status(self) -> dict[str, Any]:
        joint_status = self.piper.GetArmJointMsgs()
        gripper = self.piper.GetArmGripperMsgs()

        joint_state = joint_status.joint_state
        obs_dict = {
            "joint_0.pos": joint_state.joint_1,
            "joint_1.pos": joint_state.joint_2,
            "joint_2.pos": joint_state.joint_3,
            "joint_3.pos": joint_state.joint_4,
            "joint_4.pos": joint_state.joint_5,
            "joint_5.pos": joint_state.joint_6,
        }
        obs_dict.update(
            {
                "joint_6.pos": gripper.gripper_state.grippers_angle,
            }
        )

        return obs_dict

    def get_pos(self)  -> dict[str, Any]:
        eef_status = self.piper.GetArmEndPoseMsgs()
        gripper_dis = self.gripper.get_gripper_distance()

        eef_status = eef_status.end_pose
        eef_pose = (eef_status.X_axis ,eef_status.Y_axis, eef_status.Z_axis, eef_status.RX_axis, eef_status.RY_axis, eef_status.RZ_axis)
        x_tcp, y_tcp, z_tcp = self.apply_tool_offset(eef_pose, self.tool_offset)
        
        obs_dict = {
            "X": x_tcp,
            "Y": y_tcp,
            "Z": z_tcp,
            "RX": eef_status.RX_axis,
            "RY": eef_status.RY_axis,
            "RZ": eef_status.RZ_axis,
        }
        obs_dict.update(
            {
                "Gripper_dis": gripper_dis,
            }
        )

        return obs_dict

    def disconnect(self):
        self.piper.ModeCtrl(0x01, 0x01, 30, 0x00)
        self.piper.JointCtrl(0, 0, 0, 0, 17788, 0)
        time.sleep(3)
        while(self.piper.DisablePiper()):
            time.sleep(0.01)
        print("Disconnected ！")
    
    def connect(self):
        while( not self.piper.EnablePiper()):
            time.sleep(0.01)
        print("Piper Connected!")
        if not self.gripper.connect():
            print("连接 Pika Gripper 设备失败，请检查设备连接和串口路径")
        else:
            print("Piper Gripper Connected!")

    def go_zero(self):
        self.piper.ModeCtrl(0x01, 0x01, 30, 0x00)
        self.piper.JointCtrl(0, 0, 0, 0, 0, 0)
        self.gripper.set_gripper_distance(96.7)
    
    @staticmethod
    def apply_tool_offset(pose, tool_offset):
        X, Y, Z, RX, RY, RZ = pose
    
        # Convert angles: 0.001 degrees → radians
        rx_rad = math.radians(RX / 1000.0)
        ry_rad = math.radians(RY / 1000.0)
        rz_rad = math.radians(RZ / 1000.0)
    
        # Precompute trig values
        cx, sx = math.cos(rx_rad), math.sin(rx_rad)
        cy, sy = math.cos(ry_rad), math.sin(ry_rad)
        cz, sz = math.cos(rz_rad), math.sin(rz_rad)
    
        # Compute rotation matrix (XYZ order: R = Rz * Ry * Rx)
        r00 = cy * cz
        r01 = sx * sy * cz - cx * sz
        r02 = cx * sy * cz + sx * sz
    
        r10 = cy * sz
        r11 = sx * sy * sz + cx * cz
        r12 = cx * sy * sz - sx * cz
    
        r20 = -sy
        r21 = sx * cy
        r22 = cx * cy
    
        # Convert tool offset: meters → 0.001mm
        tool_x_mm = tool_offset[0] * 1000000.0
        tool_y_mm = tool_offset[1] * 1000000.0
        tool_z_mm = tool_offset[2] * 1000000.0
    
        # Apply rotation to tool offset
        offset_x = r00 * tool_x_mm + r01 * tool_y_mm + r02 * tool_z_mm
        offset_y = r10 * tool_x_mm + r11 * tool_y_mm + r12 * tool_z_mm
        offset_z = r20 * tool_x_mm + r21 * tool_y_mm + r22 * tool_z_mm
    
        # Calculate TCP position with rounding
        return (
            round(X + offset_x),
            round(Y + offset_y),
            round(Z + offset_z)
        )
    
    @staticmethod
    def remove_tool_offset(pose_tcp, tool_offset):
        X, Y, Z, RX, RY, RZ = pose_tcp

        # 0.001 deg -> rad
        rx_rad = math.radians(RX / 1000.0)
        ry_rad = math.radians(RY / 1000.0)
        rz_rad = math.radians(RZ / 1000.0)

        # trig
        cx, sx = math.cos(rx_rad), math.sin(rx_rad)
        cy, sy = math.cos(ry_rad), math.sin(ry_rad)
        cz, sz = math.cos(rz_rad), math.sin(rz_rad)

        # Rotation matrix (R = Rz * Ry * Rx)
        r00 = cy * cz
        r01 = sx * sy * cz - cx * sz
        r02 = cx * sy * cz + sx * sz

        r10 = cy * sz
        r11 = sx * sy * sz + cx * cz
        r12 = cx * sy * sz - sx * cz

        r20 = -sy
        r21 = sx * cy
        r22 = cx * cy

        # meters -> 0.001 mm
        tool_x_mm = tool_offset[0] * 1_000_000.0
        tool_y_mm = tool_offset[1] * 1_000_000.0
        tool_z_mm = tool_offset[2] * 1_000_000.0

        # Rotate tool offset into base frame (same as forward)
        offset_x = r00 * tool_x_mm + r01 * tool_y_mm + r02 * tool_z_mm
        offset_y = r10 * tool_x_mm + r11 * tool_y_mm + r12 * tool_z_mm
        offset_z = r20 * tool_x_mm + r21 * tool_y_mm + r22 * tool_z_mm

        # Inverse: p_j6 = p_tcp - R_j6 * t_tool
        return (
            round(X - offset_x),
            round(Y - offset_y),
            round(Z - offset_z),
        )
