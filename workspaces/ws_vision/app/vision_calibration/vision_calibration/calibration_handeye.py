"""
@Descripttion: 手眼标定
@version: 1.0
@Author: 崔译文
@Date: 2024-03-18 14:41:48
@LastEditors: 崔译文
@LastEditTime: 2024-04-02 09:18:29
"""

import cv2


class HandEyeCalibrator:
    def __init__(self):
        self.AVAILABLE_ALGORITHMS = {
            "Tsai": cv2.CALIB_HAND_EYE_TSAI,
            "Park": cv2.CALIB_HAND_EYE_PARK,
            "Horaud": cv2.CALIB_HAND_EYE_HORAUD,
            "Andreff": cv2.CALIB_HAND_EYE_ANDREFF,
            "Daniilidis": cv2.CALIB_HAND_EYE_DANIILIDIS,
        }

    def calibration(
        self, R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, cali_type="eye_to_hand", method="Horaud"
    ):
        R = None
        t = None
        if cali_type == "eye_in_hand":  # 摄像头固定在手臂末端，标定板固定在base坐标系
            R, t = cv2.calibrateHandEye(
                R_gripper2base=R_gripper2base,
                t_gripper2base=t_gripper2base,
                R_target2cam=R_target2cam,
                t_target2cam=t_target2cam,
                method=self.AVAILABLE_ALGORITHMS[method],
            )
        elif cali_type == "eye_to_hand":  # 摄像头固定在base坐标系，标定板固定在手臂末端
            R_base2gripper, t_base2gripper = [], []
            for R, t in zip(R_gripper2base, t_gripper2base):
                R_b2g = R.T
                t_b2g = -R_b2g @ t
                R_base2gripper.append(R_b2g)
                t_base2gripper.append(t_b2g)
            R, t = cv2.calibrateHandEye(
                R_gripper2base=R_base2gripper,
                t_gripper2base=t_base2gripper,
                R_target2cam=R_target2cam,
                t_target2cam=t_target2cam,
                method=self.AVAILABLE_ALGORITHMS[method],
            )
        return R, t
