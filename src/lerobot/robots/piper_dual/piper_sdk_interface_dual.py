# Piper SDK interface for LeRobot integration

import time
from typing import Any
from pika.gripper import Gripper 

from piper_sdk import C_PiperInterface_V2


class PiperSDKInterfaceDual:
    def __init__(self, port_r: str,port_l: str, gripper_r_serial: str,gripper_l_serial: str):
        if C_PiperInterface_V2 is None:
            raise ImportError("piper_sdk is not installed. Please install it with `pip install piper_sdk`.")
        try:
            self.piper_r = C_PiperInterface_V2(port_r)
            self.piper_l = C_PiperInterface_V2(port_l)
        except Exception as e:
            print(
                f"Failed to initialize Piper SDK: {e} Did you activate the can interface with `piper_sdk/can_activate.sh can0 1000000`"
            )
            self.piper_r = None
            self.piper_l = None
            return
        self.piper_r.ConnectPort()
        time.sleep(0.1)
        self.piper_l.ConnectPort()
        time.sleep(0.1)

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

        while( not self.piper_r.EnablePiper()):
            time.sleep(0.01)
        print("Right Piper Enabled!")

        while( not self.piper_l.EnablePiper()):
            time.sleep(0.01)
        print("Left Piper Enabled!")

        # Set motion control to joint mode at 100% speed
        self.piper_r.MotionCtrl_2(0x01, 0x01, 100, 0x00)
        # Set motion control to joint mode at 100% speed
        self.piper_l.MotionCtrl_2(0x01, 0x01, 100, 0x00)

        # Get the min and max positions for each joint and gripper
        angel_status = self.piper_r.GetAllMotorAngleLimitMaxSpd()
        self.min_pos_r = [
            pos.min_angle_limit for pos in angel_status.all_motor_angle_limit_max_spd.motor[1:7]
        ] + [0]
        self.max_pos_r = [
            pos.max_angle_limit for pos in angel_status.all_motor_angle_limit_max_spd.motor[1:7]
        ] + [10]  # Gripper max position in mm

        # Get the min and max positions for each joint and gripper
        angel_status = self.piper_l.GetAllMotorAngleLimitMaxSpd()
        self.min_pos_l = [
            pos.min_angle_limit for pos in angel_status.all_motor_angle_limit_max_spd.motor[1:7]
        ] + [0]
        self.max_pos_l = [
            pos.max_angle_limit for pos in angel_status.all_motor_angle_limit_max_spd.motor[1:7]
        ] + [10]  # Gripper max position in mm

        # Connect to the pika gripper
        self.gripper_r = Gripper(gripper_r_serial)
        # Connect to the pika gripper
        self.gripper_l = Gripper(gripper_l_serial)

        if not self.gripper_r.connect():
            print("连接 Right Pika Gripper 设备失败，请检查设备连接和串口路径")
            return
        print("Right Pika Gripper Connected!",gripper_r_serial)

        if not self.gripper_l.connect():
            print("连接 Left Pika Gripper 设备失败，请检查设备连接和串口路径")
            return
        print("Left Pika Gripper Connected!",gripper_l_serial)

        if not self.gripper_r.enable():
            print("Right Pika Gripper 电机启用失败")
            return
        print("Right Pika Gripper 电机启用成功")

        if not self.gripper_l.enable():
            print("Left Pika Gripper 电机启用失败")
            return
        print("Left Pika Gripper 电机启用成功")
        time.sleep(0.5)
            

    def set_joint_positions(self, positions):
        # positions: list of 7 floats, first 6 are joint and 7 is gripper position (distance in mm)
        # positions are in -100% to 100% range, we need to map them on the min and max positions
        # so -100% is min_pos and 100% is max_pos
        self.piper_r.MotionCtrl_2(0x01, 0x01, 100, 0x00)
        self.piper_l.MotionCtrl_2(0x01, 0x01, 100, 0x00)
        scaled_positions = [
            self.min_pos[i] + (self.max_pos[i] - self.min_pos[i]) * (pos + 100) / 200
            for i, pos in enumerate(positions[:6])
        ]
        scaled_positions = [100.0 * pos for pos in scaled_positions]  # Adjust factor

        # the gripper is from 0 to 100% range
        scaled_positions.append(self.min_pos[6] + (self.max_pos[6] - self.min_pos[6]) * positions[6] / 100)
        scaled_positions[6] = int(scaled_positions[6] * 10000)  # Convert to mm
        scaled_positions[13] = int(scaled_positions[13] * 10000)  # Convert to mm

        # joint 0, 3 and 5 are inverted
        joint_0_r = int(-scaled_positions[0])
        joint_1_r = int(scaled_positions[1])
        joint_2_r = int(scaled_positions[2])
        joint_3_r = int(-scaled_positions[3])
        joint_4_r = int(scaled_positions[4])
        joint_5_r = int(-scaled_positions[5])
        joint_6_r = int(scaled_positions[6])
        # joint 0, 3 and 5 are inverted
        joint_0_l = int(-scaled_positions[7])
        joint_1_l = int(scaled_positions[8])
        joint_2_l = int(scaled_positions[9])
        joint_3_l = int(-scaled_positions[10])
        joint_4_l = int(scaled_positions[11])
        joint_5_l = int(-scaled_positions[12])
        joint_6_l = int(scaled_positions[13])

        self.piper_r.JointCtrl(joint_0_r, joint_1_r, joint_2_r, joint_3_r, joint_4_r, joint_5_r)
        print("Setting right piper joint position:",joint_0_r, joint_1_r, joint_2_r, joint_3_r, joint_4_r, joint_5_r)
        self.piper_l.JointCtrl(joint_0_l, joint_1_l, joint_2_l, joint_3_l, joint_4_l, joint_5_l)
        print("Setting right piper joint position:",joint_0_l, joint_1_l, joint_2_l, joint_3_l, joint_4_l, joint_5_l)
        
        # Set gripper ctrl with pika gripper
        self.piper_r.set_gripper_distance(joint_6_r)
        # Set gripper ctrl with pika gripper
        self.piper_l.set_gripper_distance(joint_6_l)

    def set_end_position(self, positions):
        # Right 7 + Left 7
        # positions: list of 7 floats, first 6 are joint and 7 is gripper position (in mm)
        factor = 1000
        self.piper_r.MotionCtrl_2(0x01, 0x00, 100, 0x00)
        self.piper_l.MotionCtrl_2(0x01, 0x00, 100, 0x00)
        X_r = round(positions[0] * factor)
        Y_r = round(positions[1] * factor)
        Z_r = round(positions[2] * factor)
        RX_r = round(positions[3] * factor)
        RY_r = round(positions[4] * factor)
        RZ_r = round(positions[5] * factor)
        X_l = round(positions[7] * factor)
        Y_l = round(positions[8] * factor)
        Z_l = round(positions[9] * factor)
        RX_l = round(positions[10] * factor)
        RY_l = round(positions[11] * factor)
        RZ_l = round(positions[12] * factor)
        print("Setting right end position:",X_r,Y_r,Z_r,RX_r,RY_r,RZ_r)
        print("Setting left end position:",X_l,Y_l,Z_l,RX_l,RY_l,RZ_l)
        self.piper_r.EndPoseCtrl(X_r,Y_r,Z_r,RX_r,RY_r,RZ_r)
        self.piper_l.EndPoseCtrl(X_l,Y_l,Z_l,RX_l,RY_l,RZ_l)
        print("Right Gripper Distance:",position[6])
        print("Left Gripper Distance:",position[13])
        self.gripper_r.set_gripper_distance(positions[6])
        self.gripper_l.set_gripper_distance(positions[13])
        

    def get_status(self) -> dict[str, Any]:
        joint_status_r = self.piper_r.GetArmJointMsgs()
        joint_status_l = self.piper_l.GetArmJointMsgs()
        gripper_r = self.piper_r.GetArmGripperMsgs()
        gripper_l = self.piper_l.GetArmGripperMsgs()

        joint_state_r = joint_status_r.joint_state
        joint_state_l = joint_status_l.joint_state
        obs_dict = {
            "r_joint_0.pos": joint_state_r.joint_1,
            "r_joint_1.pos": joint_state_r.joint_2,
            "r_joint_2.pos": joint_state_r.joint_3,
            "r_joint_3.pos": joint_state_r.joint_4,
            "r_joint_4.pos": joint_state_r.joint_5,
            "r_joint_5.pos": joint_state_r.joint_6,
            "r_joint_6.pos": gripper_r.gripper_state.grippers_angle,
            "l_joint_0.pos": joint_state_l.joint_1,
            "l_joint_1.pos": joint_state_l.joint_2,
            "l_joint_2.pos": joint_state_l.joint_3,
            "l_joint_3.pos": joint_state_l.joint_4,
            "l_joint_4.pos": joint_state_l.joint_5,
            "l_joint_5.pos": joint_state_l.joint_6,
            "l_joint_6.pos": gripper_l.gripper_state.grippers_angle
        }

        return obs_dict

    def get_pos(self)  -> dict[str, Any]:
        eef_status_r = self.piper_r.GetArmEndPoseMsgs()
        eef_status_l = self.piper_l.GetArmEndPoseMsgs()
        gripper_dis_r = self.gripper_r.get_gripper_distance()
        gripper_dis_l = self.gripper_l.get_gripper_distance()

        eef_status_r = eef_status_r.end_pose
        eef_status_l = eef_status_l.end_pose
        obs_dict = {
            "X_r": eef_status_r.X_axis,
            "Y_r": eef_status_r.Y_axis,
            "Z_r": eef_status_r.Z_axis,
            "RX_r": eef_status_r.RX_axis,
            "RY_r": eef_status_r.RY_axis,
            "RZ_r": eef_status_r.RZ_axis,
            "Gripper_dis_r": gripper_dis_r,
            "X_l": eef_status_l.X_axis,
            "Y_l": eef_status_l.Y_axis,
            "Z_l": eef_status_l.Z_axis,
            "RX_l": eef_status_l.RX_axis,
            "RY_l": eef_status_l.RY_axis,
            "RZ_l": eef_status_l.RZ_axis,
            "Gripper_dis_l": gripper_dis_l
        }

        return obs_dict

    def disconnect(self):
        self.piper_r.ModeCtrl(0x01, 0x01, 30, 0x00)
        self.piper_l.ModeCtrl(0x01, 0x01, 30, 0x00)
        self.piper_r.JointCtrl(0, 0, 0, 0, 17788, 0)
        self.piper_l.JointCtrl(0, 0, 0, 0, 17788, 0)
        time.sleep(3)
        while(self.piper_r.DisablePiper() and self.piper_l.DisablePiper()):
            time.sleep(0.01)
        print("Both Piper Disconnected ！")
    
    def connect(self):
        while( not self.piper_r.EnablePiper()):
            time.sleep(0.01)
        print("Right Piper Connected!")
        while( not self.piper_l.EnablePiper()):
            time.sleep(0.01)
        print("Left Piper Connected!")
        if not self.gripper_r.connect():
            print("连接 Pika Gripper 设备失败，请检查设备连接和串口路径")
        else:
            print("Piper Gripper Connected!")
        if not self.gripper_l.connect():
            print("连接 Pika Gripper 设备失败，请检查设备连接和串口路径")
        else:
            print("Piper Gripper Connected!")

    def go_zero(self):
        self.piper_r.ModeCtrl(0x01, 0x01, 30, 0x00)
        self.piper_l.ModeCtrl(0x01, 0x01, 30, 0x00)
        self.piper_r.JointCtrl(0, 0, 0, 0, 0, 0)
        self.piper_l.JointCtrl(0, 0, 0, 0, 0, 0)
        self.gripper_r.set_gripper_distance(96.7)
        self.gripper_l.set_gripper_distance(96.7)