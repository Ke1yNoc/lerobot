# Piper SDK interface for LeRobot integration

import time
from typing import Any
from pika.gripper import Gripper 
import math

from piper_sdk import C_PiperInterface_V2


class PiperSDKInterfaceDual:
    def __init__(self, port_r: str,port_l: str, gripper_r_serial: str,gripper_l_serial: str):
        if C_PiperInterface_V2 is None:
            raise ImportError("piper_sdk is not installed. Please install it with `pip install piper_sdk`.")
        try:
            self.piper_r = C_PiperInterface_V2(port_r, dh_is_offset=1)
            self.piper_l = C_PiperInterface_V2(port_l, dh_is_offset=1)
        except Exception as e:
            print(
                f"Failed to initialize Piper SDK: {e} Did you activate the can interface with `piper_sdk/can_activate.sh can0 1000000`"
            )
            self.piper_r = None
            self.piper_l = None
            return
        self.piper_r.ConnectPort()
        self.piper_l.ConnectPort()
        time.sleep(0.1)
        
        self.gripper_offset = [0.19, 0, 0, 0, 0, 0]
        self.tool_offset = (0, 0, 0.1773)

        # Connect to the pika gripper
        self.gripper_r = Gripper(gripper_r_serial)
        # Connect to the pika gripper
        self.gripper_l = Gripper(gripper_l_serial)

        # joints in degs
        self.joint_limits = {
            "min": [-150.0, 0.0, -170.0, -100.0, -70.0, -120.0],
            "max": [150.0 , 180.0, 0.0  , 100.0 , 70.0 , 120.0]
        }
        self.factor = 57295.7795 #1000*180/3.1415926
            
    def set_joint_positions(self, positions):
        # Joint position (14), unit : Radian & mm
        # Position 0-5 are right arm, 6 is right gripper (in 100 mm)
        # Position 7-12 are left arm, 13 is left gripper (in 100 mm)
        joint_0_r = int(positions[0] * self.factor)
        joint_1_r = int(positions[1] * self.factor)
        joint_2_r = int(positions[2] * self.factor)
        joint_3_r = int(positions[3] * self.factor)
        joint_4_r = int(positions[4] * self.factor)
        joint_5_r = int(positions[5] * self.factor)
        gripper_r = int(positions[6] * 100)

        joint_0_l = int(positions[7] * self.factor)
        joint_1_l = int(positions[8] * self.factor)
        joint_2_l = int(positions[9] * self.factor)
        joint_3_l = int(positions[10] * self.factor)
        joint_4_l = int(positions[11] * self.factor)
        joint_5_l = int(positions[12] * self.factor)
        gripper_l = int(positions[13] * 100)

        self.piper_r.JointCtrl(joint_0_r, joint_1_r, joint_2_r, joint_3_r, joint_4_r, joint_5_r)
        self.gripper_r.set_gripper_distance(gripper_r)
        print("Setting right piper joint position:",joint_0_r, joint_1_r, joint_2_r, joint_3_r, joint_4_r, joint_5_r,"Gripper:", gripper_r)
        self.piper_l.JointCtrl(joint_0_l, joint_1_l, joint_2_l, joint_3_l, joint_4_l, joint_5_l)
        self.gripper_l.set_gripper_distance(gripper_l)
        print("Setting left piper joint position:",joint_0_l, joint_1_l, joint_2_l, joint_3_l, joint_4_l, joint_5_l,"Gripper:", gripper_l)
        

    def set_end_position(self, positions):
        # Right 7 + Left 7
        # positions: list of 7 floats, first 6 are pose in mm and 7 is gripper position (in mm)
        factor = 1000
        self.piper_r.MotionCtrl_2(0x01, 0x00, 100, 0x00)
        self.piper_l.MotionCtrl_2(0x01, 0x00, 100, 0x00)
        X_r = int(positions[0] * factor)
        Y_r = int(positions[1] * factor)
        Z_r = int(positions[2] * factor)
        RX_r = int(positions[3] * factor)
        RY_r = int(positions[4] * factor)
        RZ_r = int(positions[5] * factor)
        X_l = int(positions[7] * factor)
        Y_l = int(positions[8] * factor)
        Z_l = int(positions[9] * factor)
        RX_l = int(positions[10] * factor)
        RY_l = int(positions[11] * factor)
        RZ_l = int(positions[12] * factor)
        
        eef_status_r = (X_r,Y_r,Z_r,RX_r,RY_r,RZ_r)
        x6_r, y6_r, z6_r = self.remove_tool_offset(eef_status_r, self.tool_offset)
        
        eef_status_l = (X_l,Y_l,Z_l,RX_l,RY_l,RZ_l)
        x6_l, y6_l, z6_l = self.remove_tool_offset(eef_status_l, self.tool_offset)
        print("Setting right end position:",X_r,Y_r,Z_r,RX_r,RY_r,RZ_r)
        print("Setting j6 position:",x6_r,y6_r,z6_r,RX_r,RY_r,RZ_r)
        
        print("Setting left end position:",X_l,Y_l,Z_l,RX_l,RY_l,RZ_l)
        print("Setting j6 position:",x6_l,y6_l,z6_l,RX_l,RY_l,RZ_l)
        
        self.piper_r.EndPoseCtrl(x6_r,y6_r,z6_r,RX_r,RY_r,RZ_r)
        self.piper_l.EndPoseCtrl(x6_l,y6_l,z6_l,RX_l,RY_l,RZ_l)
        print("Right Gripper Distance:",positions[6])
        print("Left Gripper Distance:",positions[13])
        self.gripper_r.set_gripper_distance(positions[6])
        self.gripper_l.set_gripper_distance(positions[13])
        

    def get_status(self) -> dict[str, Any]:
        joint_status_r = self.piper_r.GetArmJointMsgs()
        joint_status_l = self.piper_l.GetArmJointMsgs()
        gripper_dis_r = self.gripper_r.get_gripper_distance()
        gripper_dis_l = self.gripper_l.get_gripper_distance()

        joint_state_r = joint_status_r.joint_state
        joint_state_l = joint_status_l.joint_state
        obs_dict = {
            "r_joint_0.pos": joint_state_r.joint_1 / self.factor,
            "r_joint_1.pos": joint_state_r.joint_2 / self.factor,
            "r_joint_2.pos": joint_state_r.joint_3 / self.factor,
            "r_joint_3.pos": joint_state_r.joint_4 / self.factor,
            "r_joint_4.pos": joint_state_r.joint_5 / self.factor,
            "r_joint_5.pos": joint_state_r.joint_6 / self.factor,
            "r_gripper.dis": gripper_dis_r / 100,
            "l_joint_0.pos": joint_state_l.joint_1 / self.factor,
            "l_joint_1.pos": joint_state_l.joint_2 / self.factor,
            "l_joint_2.pos": joint_state_l.joint_3 / self.factor,
            "l_joint_3.pos": joint_state_l.joint_4 / self.factor,
            "l_joint_4.pos": joint_state_l.joint_5 / self.factor,
            "l_joint_5.pos": joint_state_l.joint_6 / self.factor,
            "l_gripper.dis": gripper_dis_l / 100,
        }

        return obs_dict

    def get_pos(self)  -> dict[str, Any]:
        eef_status_r = self.piper_r.GetArmEndPoseMsgs()
        eef_status_l = self.piper_l.GetArmEndPoseMsgs()
        gripper_dis_r = self.gripper_r.get_gripper_distance()
        gripper_dis_l = self.gripper_l.get_gripper_distance()

        eef_status_r = eef_status_r.end_pose
        eef_pose_r = (eef_status_r.X_axis ,eef_status_r.Y_axis, eef_status_r.Z_axis, eef_status_r.RX_axis, eef_status_r.RY_axis, eef_status_r.RZ_axis)
        x_tcp_r, y_tcp_r, z_tcp_r = self.apply_tool_offset(eef_pose_r, self.tool_offset)
        
        eef_status_l = eef_status_l.end_pose
        eef_pose_l = (eef_status_l.X_axis ,eef_status_l.Y_axis, eef_status_l.Z_axis, eef_status_l.RX_axis, eef_status_l.RY_axis, eef_status_l.RZ_axis)
        x_tcp_l, y_tcp_l, z_tcp_l = self.apply_tool_offset(eef_pose_l, self.tool_offset)
        factor = 1000000
        gripper_factor = 96.4056660485538
        rot_factor = math.pi/180000
        obs_dict = {
            "X_r": x_tcp_r/factor,
            "Y_r": y_tcp_r/factor,
            "Z_r": z_tcp_r/factor,
            "RX_r": eef_status_r.RX_axis * rot_factor,
            "RY_r": eef_status_r.RY_axis * rot_factor,
            "RZ_r": eef_status_r.RZ_axis * rot_factor,
            "Gripper_dis_r": gripper_dis_r / gripper_factor,
            "X_l": x_tcp_l/factor,
            "Y_l": y_tcp_l/factor,
            "Z_l": z_tcp_l/factor,
            "RX_l": eef_status_l.RX_axis * rot_factor,
            "RY_l": eef_status_l.RY_axis * rot_factor,
            "RZ_l": eef_status_l.RZ_axis * rot_factor,
            "Gripper_dis_l": gripper_dis_l / gripper_factor
        }

        return obs_dict

    def disconnect(self):
        self.piper_r.ModeCtrl(0x01, 0x01, 10, 0x00)
        self.piper_l.ModeCtrl(0x01, 0x01, 10, 0x00)
        self.piper_r.JointCtrl(0, 0, 0, 0, 17900, 0)
        self.piper_l.JointCtrl(0, 0, 0, 0, 17900, 0)
        time.sleep(2)
        while(self.piper_r.DisablePiper() and self.piper_l.DisablePiper()):
            time.sleep(0.01)
        print("Piper arms disconnected")
    
    def connect(self):
        # reset the arm if it's not in idle state
        if self.piper_r.GetArmStatus().arm_status.motion_status != 0:
            self.piper_r.EmergencyStop(0x02)  # resume

        if self.piper_r.GetArmStatus().arm_status.ctrl_mode == 2:
            print("The right arm is in teaching mode, the light is green, press the button to exit teaching mode.")
            self.piper_r.EmergencyStop(0x02)  # resume

            # reset the arm if it's not in idle state
        if self.piper_l.GetArmStatus().arm_status.motion_status != 0:
            self.piper_l.EmergencyStop(0x02)  # resume

        if self.piper_l.GetArmStatus().arm_status.ctrl_mode == 2:
            print("The right arm is in teaching mode, the light is green, press the button to exit teaching mode.")
            self.piper_l.EmergencyStop(0x02)  # resume

        while True:
            ok_r = self.piper_r.EnablePiper()
            ok_l = self.piper_l.EnablePiper()
            if ok_r and ok_l:
                print("Right & Left Piper Enabled!")
                break
            time.sleep(0.01)

        # Set motion control to joint mode at 100% speed
        self.piper_r.ModeCtrl(0x01, 0x01, 100, 0x00)
        # Set motion control to joint mode at 100% speed
        self.piper_l.ModeCtrl(0x01, 0x01, 100, 0x00)

        if not self.gripper_r.connect():
            print("连接 Right Pika Gripper 设备失败，请检查设备连接和串口路径")
            return
        print("Right Pika Gripper Connected!")

        if not self.gripper_l.connect():
            print("连接 Left Pika Gripper 设备失败，请检查设备连接和串口路径")
            return
        print("Left Pika Gripper Connected!")

        if not self.gripper_r.enable():
            print("Right Pika Gripper 电机启用失败")
            return
        print("Right Pika Gripper 电机启用成功")

        if not self.gripper_l.enable():
            print("Left Pika Gripper 电机启用失败")
            return
        print("Left Pika Gripper 电机启用成功")

        print("前往零位")
        self.piper_r.JointCtrl(0, 0, 0, 0, 0, 0)
        self.piper_l.JointCtrl(0, 0, 0, 0, 0, 0)
        self.gripper_l.set_gripper_distance(30)
        self.gripper_r.set_gripper_distance(30)

    def go_zero(self):
        self.piper_r.ModeCtrl(0x01, 0x01, 30, 0x00)
        self.piper_l.ModeCtrl(0x01, 0x01, 30, 0x00)
        self.piper_r.JointCtrl(0, 0, 0, 0, 0, 0)
        self.piper_l.JointCtrl(0, 0, 0, 0, 0, 0)
        self.gripper_r.set_gripper_distance(0)
        self.gripper_l.set_gripper_distance(0)
        
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
