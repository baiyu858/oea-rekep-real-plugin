"""
Orbbec 相机外参配置

此文件包含 Orbbec 相机的外参配置（R 旋转矩阵和 T 平移向量）。
标定结果由 franky_calibration.py 生成后填入。

外参格式:
- R: 3x3 旋转矩阵
- T: 3x1 平移向量 (毫米)
"""

import numpy as np
from dataclasses import dataclass


@dataclass
class CameraConfig:
    serial: str
    name: str
    id: int
    eye_in_hand: bool
    T: np.ndarray = None  # 位置向量 (3,) 单位毫米
    R: np.ndarray = None  # 旋转矩阵 (3,3)
    depth_scale: float = 0.001  # 深度比例系数


# Orbbec 相机外参配置
# 注意: 运行 hand_eye_calibration/franky_calibration.py 进行手眼标定后更新此处
CAMERA_CONFIGS = {
    "global1": CameraConfig(
        serial="ORBBEC_SERIAL_PLACEHOLDER",
        name="orbbec",
        id=0,
        eye_in_hand=False,
        R=np.eye(3),
        T=np.zeros(3)
    ),
    "global2": CameraConfig(
        serial="ORBBEC_SERIAL_PLACEHOLDER",
        name="orbbec",
        id=1,
        eye_in_hand=False,
        R=np.eye(3),
        T=np.zeros(3)
    ),
    "global3": CameraConfig(
        serial="ORBBEC_SERIAL_PLACEHOLDER",
        name="orbbec",
        id=2,
        eye_in_hand=False,
        R=np.eye(3),
        T=np.zeros(3)
    ),
}


def create_transform_matrix(R, T):
    """从 R 旋转矩阵和 T 平移向量创建 4x4 变换矩阵"""
    if R is None or T is None:
        return None

    transform = np.eye(4)
    transform[:3, :3] = R
    transform[:3, 3] = T / 1000.0  # 毫米转米
    return transform
