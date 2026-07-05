#!/usr/bin/env python3
"""
手臂行为通用工具模块
包含欧拉角和四元数转换等通用方法
"""

import math
from typing import List


class ArmUtils:
    """手臂行为通用工具类"""
    
    @staticmethod
    def quaternion_to_euler(q: List[float]) -> List[float]:
        """
        将四元数转换为欧拉角 (roll, pitch, yaw)
        
        Args:
            q: 四元数 [qx, qy, qz, qw]
            
        Returns:
            欧拉角 [roll, pitch, yaw] (单位：弧度)
        """
        qx, qy, qz, qw = q[0], q[1], q[2], q[3]
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return [roll, pitch, yaw]
    
    @staticmethod
    def euler_to_quaternion(euler: List[float]) -> List[float]:
        """
        将欧拉角转换为四元数
        
        Args:
            euler: 欧拉角 [roll, pitch, yaw] (单位：弧度)
            
        Returns:
            四元数 [qx, qy, qz, qw]
        """
        roll, pitch, yaw = euler[0], euler[1], euler[2]
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return [qx, qy, qz, qw]
