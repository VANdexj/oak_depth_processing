#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""深度数据发布工具

该脚本用于捕获 OAK-D 相机的深度数据并通过 ROS2 发布。

功能特性：
- 深度数据采集和发布
- 原始数据保存功能
"""

from __future__ import annotations

import time
from collections import defaultdict, deque
from datetime import datetime
from pathlib import Path

import cv2
import depthai as dai
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from std_msgs.msg import Header


# ============================================================================
# 全局常量配置
# ============================================================================

# 相机配置参数
CAMERA_FPS = 30  # 相机帧率

# 深度图输出尺寸配置
DEPTH_WIDTH = 640
DEPTH_HEIGHT = 400


class DepthCameraPublisher:
    """深度相机数据发布器"""
    
    def __init__(self, node: Node):
        self.node = node
        self.bridge = CvBridge()
        
        # 创建发布器
        self.depth_pub = self.node.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.depth_colormap_pub = self.node.create_publisher(Image, '/camera/depth/colormap', 10)
        self.disparity_pub = self.node.create_publisher(Image, '/camera/disparity', 10)
        
        self.node.get_logger().info('深度相机话题发布器已初始化')

    def publish_data(self, depth_frame: np.ndarray, disparity_frame: np.ndarray, max_disparity: float):
        """发布相机数据"""
        try:
            # 发布原始深度图 (16位)
            if depth_frame is not None:
                depth_msg = self.bridge.cv2_to_imgmsg(depth_frame.astype(np.uint16), encoding="16UC1")
                depth_msg.header = self._create_header()
                self.depth_pub.publish(depth_msg)

            # 发布彩色化深度图
            if depth_frame is not None:
                depth_colormap = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_frame, alpha=255.0/3000.0), 
                    cv2.COLORMAP_JET
                )
                depth_colormap_msg = self.bridge.cv2_to_imgmsg(depth_colormap, encoding="bgr8")
                depth_colormap_msg.header = self._create_header()
                self.depth_colormap_pub.publish(depth_colormap_msg)

            # 发布视差图
            if disparity_frame is not None and max_disparity > 0:
                disparity_colormap = cv2.applyColorMap(
                    (disparity_frame * (255 / max_disparity)).astype(np.uint8), 
                    cv2.COLORMAP_JET
                )
                disparity_msg = self.bridge.cv2_to_imgmsg(disparity_colormap, encoding="bgr8")
                disparity_msg.header = self._create_header()
                self.disparity_pub.publish(disparity_msg)

        except Exception as e:
            self.node.get_logger().error(f'数据发布错误: {e}')

    def _create_header(self):
        """创建消息头"""
        header = Header()
        header.stamp = self.node.get_clock().now().to_msg()
        header.frame_id = "camera_frame"
        return header


def create_pipeline(device: dai.Device) -> tuple[dai.Pipeline, float]:
    """创建 DepthAI 处理流水线

    Args:
        device: DepthAI 设备对象，用于读取校准数据

    Returns:
        tuple: 包含以下两个元素的元组
            - pipeline: 配置好的 DepthAI 流水线对象
            - max_disparity: 最大视差值，用于深度可视化
    """
    # 创建 DepthAI 流水线
    pipeline = dai.Pipeline()
    print(f"正在为设备创建流水线：{device.getDeviceName()}")

    # 创建相机节点
    mono_left = pipeline.create(dai.node.MonoCamera)
    mono_right = pipeline.create(dai.node.MonoCamera)

    # 创建处理节点
    stereo = pipeline.create(dai.node.StereoDepth)

    # 创建输出节点
    xout_depth = pipeline.create(dai.node.XLinkOut)
    xout_disparity = pipeline.create(dai.node.XLinkOut)
    xout_depth.setStreamName("depth")
    xout_disparity.setStreamName("disparity")

    # 配置单目相机
    mono_resolution = dai.MonoCameraProperties.SensorResolution.THE_800_P
    mono_left.setResolution(mono_resolution)
    mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    mono_left.setFps(CAMERA_FPS)

    mono_right.setResolution(mono_resolution)
    mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    mono_right.setFps(CAMERA_FPS)

    # 配置立体深度节点 - 简化配置
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setMedianFilter(dai.StereoDepthProperties.MedianFilter.KERNEL_5x5) 
    stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
    stereo.setOutputSize(DEPTH_WIDTH, DEPTH_HEIGHT)
    
    # 基本配置
    stereo.setConfidenceThreshold(220)
    stereo.setLeftRightCheck(True)
    stereo.setExtendedDisparity(True)
    stereo.setSubpixel(True)
    
    config = stereo.initialConfig.get()

    config.postProcessing.thresholdFilter.minRange = 0  
    config.postProcessing.thresholdFilter.maxRange = 10000  

    config.postProcessing.speckleFilter.enable=True
    config.postProcessing.speckleFilter.differenceThreshold=2
    config.postProcessing.speckleFilter.speckleRange=50

    config.postProcessing.temporalFilter.enable = True

    # config.postProcessing.spatialFilter.enable = True
    # config.postProcessing.spatialFilter.holeFillingRadius = 15
    # config.postProcessing.spatialFilter.numIterations = 1

    config.postProcessing.decimationFilter.decimationFactor = 2

    stereo.initialConfig.set(config)

    # 连接节点
    mono_left.out.link(stereo.left)
    mono_right.out.link(stereo.right)
    stereo.depth.link(xout_depth.input)
    stereo.disparity.link(xout_disparity.input)

    return pipeline, stereo.initialConfig.getMaxDisparity()


def save_raw_frames(
    depth_frame: np.ndarray,
    disparity_frame: np.ndarray,
) -> tuple[Path, Path]:
    """保存原始深度数据

    Args:
        depth_frame: 原始深度帧（单位：毫米）
        disparity_frame: 视差帧（用于可视化）

    Returns:
        tuple: 包含两个文件路径的元组
            - depth_path: 深度图保存路径
            - disp_path: 视差图保存路径
    """
    save_dir = Path("captures")
    try:
        save_dir.mkdir(exist_ok=True)
    except Exception as e:
        print(f"警告：无法创建捕获目录 - {e}")
        save_dir = Path(".")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

    # 深度图保存路径 (16 位 PNG，保留原始毫米精度)
    depth_path = save_dir / f"{timestamp}_depth_raw.png"

    # 视差图保存路径
    disp_path = save_dir / f"{timestamp}_disparity.png"

    try:
        # 保存原始深度数据 (16 位 PNG)
        depth_16bit = depth_frame.astype(np.uint16)
        cv2.imwrite(str(depth_path), depth_16bit, [cv2.IMWRITE_PNG_COMPRESSION, 9])

        # 保存视差图 (8 位用于可视化)
        disp_8bit = disparity_frame.astype(np.uint8)
        cv2.imwrite(str(disp_path), disp_8bit, [cv2.IMWRITE_PNG_COMPRESSION, 9])
    except Exception as e:
        error_msg = f"保存文件时发生错误：{e}"
        print(f"错误：{error_msg}")
        raise RuntimeError(error_msg) from e

    print(f"已保存原始数据:\n  深度图：{depth_path}\n  视差图：{disp_path}")

    return depth_path, disp_path


class FpsHandler:
    """FPS处理器"""

    def __init__(self, max_ticks: int = 100) -> None:
        self._ticks: defaultdict[str, deque[float]] = defaultdict(lambda: deque(maxlen=max_ticks))
        self._last_fps = {}

    def tick(self, name: str, timestamp: float | None = None) -> None:
        self._ticks[name].append(timestamp or time.monotonic())

    def tick_fps(self, name: str) -> float:
        queue = self._ticks[name]
        if len(queue) < 2:
            return 0.0

        if name in self._last_fps:
            last_time, fps = self._last_fps[name]
            if queue[-1] == last_time:
                return fps

        time_diff = queue[-1] - queue[0]
        fps = (len(queue) - 1) / time_diff if time_diff != 0 else 0.0
        self._last_fps[name] = (queue[-1], fps)
        return fps

    def print_status(self):
        print("=== FPS STATUS ===")
        for name in self._ticks:
            print(f"[{name}]: {self.tick_fps(name):.1f}")


def main() -> None:
    """主运行函数"""
    rclpy.init()
    node = Node('depth_camera_node')
    publisher = DepthCameraPublisher(node)

    # 创建 FPS 处理器
    fps_handler = FpsHandler()

    # 创建捕获目录
    save_dir = Path("captures")
    try:
        save_dir.mkdir(exist_ok=True)
        print(f"按 'c' 保存原始数据。文件将保存至：{save_dir.absolute()}")
    except Exception as e:
        print(f"警告：无法创建捕获目录 - {e}")

    # 初始化设备
    with dai.Device() as device:
        pipeline, max_disparity = create_pipeline(device)
        device.startPipeline(pipeline)

        # 创建队列
        depth_queue = device.getOutputQueue("depth", maxSize=4, blocking=False)
        disparity_queue = device.getOutputQueue("disparity", maxSize=4, blocking=False)

        # 初始化帧数据
        depth_frame = None
        disparity_frame = None

        print("深度数据采集开始...")
        print("按 'q' 退出程序")
        print("按 'c' 保存当前帧数据")

        # 主循环
        try:
            while True:
                # 获取深度数据
                depth_msg = depth_queue.tryGet()
                disparity_msg = disparity_queue.tryGet()

                if depth_msg is not None:
                    depth_frame = depth_msg.getFrame()
                    fps_handler.tick("depth")

                if disparity_msg is not None:
                    disparity_frame = disparity_msg.getFrame()
                    fps_handler.tick("disparity")

                # 发布数据
                if depth_frame is not None and disparity_frame is not None:
                    publisher.publish_data(depth_frame, disparity_frame, max_disparity)

                # 处理键盘输入
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                # 保存原始数据
                if key == ord("c") and depth_frame is not None and disparity_frame is not None:
                    save_raw_frames(depth_frame, disparity_frame)

                # 定期打印FPS状态
                if int(time.time()) % 5 == 0:
                    fps_handler.print_status()
                    time.sleep(0.1)  # 避免频繁打印

        except KeyboardInterrupt:
            print("\n程序被用户中断")

    # 清理
    node.destroy_node()
    rclpy.shutdown()
    print("程序正常退出")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"程序执行过程中发生错误：{e}")
        import traceback
        traceback.print_exc()