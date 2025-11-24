#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""SDK增强版 OAK-D 深度数据发布工具

使用 DepthAI SDK 简化开发，同时保留滤波参数控制的灵活性
"""

import time
from pathlib import Path
from datetime import datetime

import cv2
import depthai as dai
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float32, Header
from cv_bridge import CvBridge
from depthai_sdk import OakCamera
from depthai_sdk.components.stereo_component import WLSLevel


class SdkEnhancedOakDepthPublisher(Node):
    """SDK增强版 OAK-D 深度数据发布器"""
    
    def __init__(self):
        super().__init__('sdk_enhanced_oak_depth_publisher')
        
        # 参数声明
        self.declare_parameters(
            namespace='',
            parameters=[
                ('camera_fps', 30),
                ('color_resolution', '1080p'),
                ('depth_resolution', '800p'),
                ('confidence_threshold', 240),
                ('median_filter', 5),  # 0=关闭, 3=3x3, 5=5x5, 7=7x7
                ('lr_check', True),
                ('extended_disparity', True),
                ('subpixel', True),
                ('subpixel_bits', 3),
                ('wls_lambda', 0),
                ('wls_sigma', 1.5),
                ('wls_level', 'LOW'),  # LOW, MEDIUM, HIGH
                ('max_depth', 10000),  # 毫米
                ('enable_temporal_filter', True),
                ('enable_spatial_filter', True),
                ('publish_colored_depth', True),
                ('publish_disparity', True),
                ('hardware_shaves', 3),  # 新增：硬件资源分配
                ('hardware_memory_slices', 2),  # 新增：内存切片分配
            ]
        )
        
        # 获取参数
        self.camera_fps = self.get_parameter('camera_fps').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.median_filter = self.get_parameter('median_filter').value
        self.lr_check = self.get_parameter('lr_check').value
        self.extended_disparity = self.get_parameter('extended_disparity').value
        self.subpixel = self.get_parameter('subpixel').value
        self.subpixel_bits = self.get_parameter('subpixel_bits').value
        self.wls_lambda = self.get_parameter('wls_lambda').value
        self.wls_sigma = self.get_parameter('wls_sigma').value
        self.wls_level = self.get_parameter('wls_level').value
        self.max_depth = self.get_parameter('max_depth').value
        self.enable_temporal_filter = self.get_parameter('enable_temporal_filter').value
        self.enable_spatial_filter = self.get_parameter('enable_spatial_filter').value
        self.publish_colored_depth = self.get_parameter('publish_colored_depth').value
        self.publish_disparity = self.get_parameter('publish_disparity').value
        self.hardware_shaves = self.get_parameter('hardware_shaves').value
        self.hardware_memory_slices = self.get_parameter('hardware_memory_slices').value
        
        # 初始化工具
        self.bridge = CvBridge()
        
        # 创建发布器
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_colormap_pub = self.create_publisher(Image, '/camera/depth/colormap', 10)
        self.disparity_pub = self.create_publisher(Image, '/camera/disparity', 10)
        self.fps_pub = self.create_publisher(Float32, '/camera/fps', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)
        
        # 帧率计算
        self.depth_frame_count = 0
        self.color_frame_count = 0
        self.last_fps_time = self.get_clock().now()
        self.current_depth_fps = 0.0
        self.current_color_fps = 0.0
        
        self.get_logger().info('SDK增强版 OAK-D 深度发布器已初始化')
        self.get_logger().info('开始初始化相机...')
        
        # 初始化相机
        self.setup_camera()

    def setup_camera(self):
        """使用 SDK 初始化相机"""
        try:
            with OakCamera() as oak:
                # 创建彩色相机
                color = oak.create_camera(
                    'color', 
                    resolution=self.get_parameter('color_resolution').value, 
                    fps=self.camera_fps
                )
                
                # 创建立体深度相机
                stereo = oak.stereo(
                    resolution=self.get_parameter('depth_resolution').value,
                    fps=self.camera_fps
                )
                
                # 使用 SDK 的高级配置方法
                stereo.config_stereo(
                    confidence=self.confidence_threshold,
                    median=self.get_median_filter(),
                    lr_check=self.lr_check,
                    extended=self.extended_disparity,
                    subpixel=self.subpixel,
                    subpixel_bits=self.subpixel_bits
                )
                
                # 配置 WLS 滤波
                wls_level_map = {
                    'LOW': WLSLevel.LOW,
                    'MEDIUM': WLSLevel.MEDIUM, 
                    'HIGH': WLSLevel.HIGH
                }
                stereo.config_wls(
                    wls_level=wls_level_map.get(self.wls_level, WLSLevel.HIGH),
                    wls_lambda=self.wls_lambda,
                    wls_sigma=self.wls_sigma
                )
                
                # 获取底层节点进行更精细的配置
                stereo_node = stereo.node

                # 设置硬件资源分配 - 解决警告问题
                self.get_logger().info(f'设置硬件资源: {self.hardware_shaves} shaves, {self.hardware_memory_slices} memory slices')
                stereo_node.setPostProcessingHardwareResources(
                    self.hardware_shaves,
                    self.hardware_memory_slices
                )

                config = stereo_node.initialConfig.get()
                
                # # 深度范围配置
                # config.postProcessing.thresholdFilter.minRange = 0
                # config.postProcessing.thresholdFilter.maxRange = self.max_depth
                
                # 时域滤波
                config.postProcessing.temporalFilter.enable = self.enable_temporal_filter
                if self.enable_temporal_filter:
                    config.postProcessing.temporalFilter.persistencyMode = (
                        dai.StereoDepthConfig.PostProcessing.TemporalFilter.PersistencyMode.VALID_2_IN_LAST_3
                    )
                
                # # 空间滤波
                # config.postProcessing.spatialFilter.enable = self.enable_spatial_filter
                # if self.enable_spatial_filter:
                #     config.postProcessing.spatialFilter.holeFillingRadius = 5
                #     config.postProcessing.spatialFilter.numIterations = 1
                #     config.postProcessing.spatialFilter.alpha = 0.7
                #     config.postProcessing.spatialFilter.delta = 80

                # 散斑滤波                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
                config.postProcessing.speckleFilter.enable = True
                config.postProcessing.speckleFilter.differenceThreshold=1
                config.postProcessing.speckleFilter.speckleRange = 5

                config.postProcessing.decimationFilter.decimationFactor = 2
                
                stereo_node.initialConfig.set(config)
                
                self.get_logger().info('相机配置完成，开始设置回调...')
                
                # 设置回调函数
                oak.callback(stereo.out.depth, self.publish_depth_data)
                oak.callback(color.out.main, self.publish_color_data)
                oak.callback(stereo.out.disparity, self.publish_disparity_data)
                
                self.get_logger().info('相机初始化成功！开始发布数据...')
                
                # 启动非阻塞处理
                oak.start(blocking=False)
                
                # 主循环
                self.run_main_loop(oak)
                
        except Exception as e:
            self.get_logger().error(f'相机初始化失败: {str(e)}')
            raise

    def get_median_filter(self):
        """将参数转换为 MedianFilter 枚举"""
        median_map = {
            0: dai.MedianFilter.MEDIAN_OFF,
            3: dai.MedianFilter.KERNEL_3x3,
            5: dai.MedianFilter.KERNEL_5x5,
            7: dai.MedianFilter.KERNEL_7x7
        }
        return median_map.get(self.median_filter, dai.MedianFilter.KERNEL_5x5)

    def run_main_loop(self, oak):
        """主运行循环"""
        self.get_logger().info('主循环开始运行')
        self.get_logger().info('按 "q" 退出程序')
        
        try:
            while rclpy.ok() and oak.running():
                # 处理 SDK 事件
                oak.poll()
                
                # 处理键盘输入
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                
                # 短暂休眠以避免过度占用CPU
                time.sleep(0.01)
                    
        except KeyboardInterrupt:
            self.get_logger().info('接收到中断信号')
        except Exception as e:
            self.get_logger().error(f'主循环运行错误: {e}')
        finally:
            self.cleanup()

    def publish_depth_data(self, packet):
        """发布深度数据 - SDK 回调版本"""
        try:
            depth_frame = packet.frame
            
            # 发布原始深度图 (16位)
            depth_msg = self.bridge.cv2_to_imgmsg(depth_frame.astype(np.uint16), "16UC1")
            depth_msg.header = self.create_header()
            self.depth_pub.publish(depth_msg)
            
            # 发布彩色化深度图
            if self.publish_colored_depth:
                depth_colormap = self.colorize_depth(depth_frame)
                depth_colormap_msg = self.bridge.cv2_to_imgmsg(depth_colormap, "bgr8")
                depth_colormap_msg.header = depth_msg.header
                self.depth_colormap_pub.publish(depth_colormap_msg)
            
            # 更新深度帧率
            self.depth_frame_count += 1
            self.update_fps()
                
        except Exception as e:
            self.get_logger().error(f'发布深度数据错误: {e}')

    def publish_disparity_data(self, packet):
        """发布视差数据 - SDK 回调版本"""
        if not self.publish_disparity:
            return
            
        try:
            disparity_frame = packet.frame
            
            # 归一化视差图用于可视化
            disparity_normalized = cv2.normalize(disparity_frame, None, 0, 255, cv2.NORM_MINMAX)
            disparity_colormap = cv2.applyColorMap(disparity_normalized.astype(np.uint8), cv2.COLORMAP_JET)
            
            disparity_msg = self.bridge.cv2_to_imgmsg(disparity_colormap, "bgr8")
            disparity_msg.header = self.create_header()
            self.disparity_pub.publish(disparity_msg)
            
        except Exception as e:
            self.get_logger().error(f'发布视差数据错误: {e}')

    def publish_color_data(self, packet):
        """发布彩色数据 - SDK 回调版本"""
        try:
            color_frame = packet.frame
            
            color_msg = self.bridge.cv2_to_imgmsg(color_frame, "bgr8")
            color_msg.header = self.create_header()
            self.color_pub.publish(color_msg)
            
            # 更新彩色帧率
            self.color_frame_count += 1
            self.update_fps()
            
        except Exception as e:
            self.get_logger().error(f'发布彩色数据错误: {e}')

    def colorize_depth(self, depth_frame, max_distance=None):
        """深度图彩色化处理"""
        if max_distance is None:
            max_distance = self.max_depth
            
        # 创建有效掩码
        valid_mask = (depth_frame > 0) & (depth_frame <= max_distance)
        
        if not np.any(valid_mask):
            return np.zeros((*depth_frame.shape, 3), dtype=np.uint8)
        
        # 使用对数变换增强近处细节
        depth_float = depth_frame.astype(np.float32)
        depth_log = np.zeros_like(depth_float)
        depth_log[valid_mask] = np.log1p(depth_float[valid_mask])
        
        # 归一化
        valid_log = depth_log[valid_mask]
        if len(valid_log) > 0:
            log_min, log_max = np.min(valid_log), np.max(valid_log)
            if log_max > log_min:
                depth_normalized = np.zeros_like(depth_log)
                depth_normalized[valid_mask] = (depth_log[valid_mask] - log_min) / (log_max - log_min)
                
                # 反转颜色（近红远蓝）
                depth_normalized[valid_mask] = 1.0 - depth_normalized[valid_mask]
                
                depth_8bit = (depth_normalized * 255).astype(np.uint8)
                depth_colored = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)
                depth_colored[~valid_mask] = [0, 0, 0]  # 无效区域设为黑色
                
                return depth_colored
        
        return np.zeros((*depth_frame.shape, 3), dtype=np.uint8)

    def update_fps(self):
        """更新并发布帧率"""
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_fps_time).nanoseconds / 1e9
        
        # 每秒更新一次帧率
        if time_diff >= 1.0:
            # 计算深度帧率
            if self.depth_frame_count > 0:
                self.current_depth_fps = self.depth_frame_count / time_diff
            else:
                self.current_depth_fps = 0.0
                
            # 计算彩色帧率
            if self.color_frame_count > 0:
                self.current_color_fps = self.color_frame_count / time_diff
            else:
                self.current_color_fps = 0.0
            
            # 发布深度帧率
            fps_msg = Float32()
            fps_msg.data = float(self.current_depth_fps)
            self.fps_pub.publish(fps_msg)
            
            # 定期日志输出
            self.get_logger().info(
                f'深度帧率: {self.current_depth_fps:.1f} FPS, '
                f'彩色帧率: {self.current_color_fps:.1f} FPS', 
                throttle_duration_sec=2.0
            )
            
            # 重置计数器
            self.depth_frame_count = 0
            self.color_frame_count = 0
            self.last_fps_time = current_time

    def create_header(self):
        """创建消息头"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "oak_camera_frame"
        return header

    def cleanup(self):
        """清理资源"""
        self.get_logger().info('正在清理资源...')
        cv2.destroyAllWindows()
        self.get_logger().info('资源清理完成')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SdkEnhancedOakDepthPublisher()
    except Exception as e:
        print(f"节点启动失败: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
