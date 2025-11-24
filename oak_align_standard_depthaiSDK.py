#!/usr/bin/env python3

import depthai as dai
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from depthai_sdk import OakCamera
import cv2
import numpy as np
from depthai_sdk.components.stereo_component import WLSLevel
from depthai_sdk.visualize.configs import StereoColor


class OakDepthPublisher(Node):
    def __init__(self):
        super().__init__('oak_depth_publisher')
        
        # 创建发布器
        self.depth_publisher = self.create_publisher(Image, '/oak/camera/depth', 10)
        self.color_publisher = self.create_publisher(Image, '/oak/camera/color', 10)
        self.depth_colored_publisher = self.create_publisher(Image, '/oak/camera/depth_colored', 10)
        self.fps_publisher = self.create_publisher(Float32, '/oak/camera/fps', 10)
        
        # 用于OpenCV和ROS图像格式转换
        self.bridge = CvBridge()
        
        # 帧率计算变量
        self.frame_count = 0
        self.last_time = self.get_clock().now()
        
        self.get_logger().info('OAK Depth Publisher节点已启动，正在初始化相机...')
        
        # 初始化OAK相机并开始处理
        self.setup_oak_camera()

    def setup_oak_camera(self):
        try:
            with OakCamera() as oak:
                color = oak.create_camera('color', resolution='1080p', fps=30)
                # 创建立体深度流
                stereo = oak.stereo('800p', fps=30)

                stereo.config_stereo(
                    confidence=240,
                    median=dai.MedianFilter.KERNEL_5x5,
                    extended=True,   # 关注近距离物体时开启
                    subpixel=True,   # 开启亚像素提升精度
                    subpixel_bits=3,          # 亚像素位数 (3-5)，值越高精度越高
                    lr_check=True,    # 开启左右校验
                    lr_check_threshold=5
                )
                stereo.config_wls(
                    wls_level=WLSLevel.HIGH,
                    wls_lambda=0,          # Lambda值，调整平滑度
                    wls_sigma=1.5               # Sigma值，控制相似性权重
                )  

                oak.visualize(stereo.out.depth)
            
                # 设置回调函数
                oak.callback(stereo.out.depth, callback=self.publish_depth_data)
                oak.callback(color.out.main, callback=self.publish_color_data)
                
                self.get_logger().info('OAK相机初始化成功，开始发布数据...')
                
                # 启动非阻塞式处理
                oak.start(blocking=False)
           
                # 保持节点运行
                while rclpy.ok() and oak.running():
                    oak.poll()
                    rclpy.spin_once(self, timeout_sec=0.001)
                    
        except Exception as e:
            self.get_logger().error(f'初始化OAK相机时发生错误: {str(e)}')
        finally:
            self.get_logger().info('OAK Depth Publisher节点正在关闭...')

    def publish_depth_data(self, packet):
        """处理和发布深度数据"""
        try:
            # 从packet获取深度图 - SDK已经生成了彩色深度图
            depth_frame = packet.frame

            # 发布原始深度图
            depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding="mono16")
            depth_msg.header.stamp = self.get_clock().now().to_msg()
            depth_msg.header.frame_id = "oak_depth_frame"
            self.depth_publisher.publish(depth_msg)
            
            # 简单生成彩色深度图（备用方案）
            depth_colored = self.simple_colorize(depth_frame)
            depth_colored_msg = self.bridge.cv2_to_imgmsg(depth_colored, encoding="bgr8")
            depth_colored_msg.header.stamp = depth_msg.header.stamp
            depth_colored_msg.header.frame_id = "oak_depth_colored_frame"
            self.depth_colored_publisher.publish(depth_colored_msg)
            
            # 计算并发布帧率
            self.calculate_and_publish_fps()
            
        except Exception as e:
            self.get_logger().error(f'处理深度数据时发生错误: {str(e)}')


    def simple_colorize(self, depth_frame):
        """对数归一化 - 近处颜色变化更明显"""
        # 创建有效掩码
        valid_mask = depth_frame > 0
        
        if not np.any(valid_mask):
            return np.zeros((*depth_frame.shape, 3), dtype=np.uint8)
        
        # 转换为浮点数
        depth_float = depth_frame.astype(np.float32)
        
        # 对数变换增强近处细节
        depth_log = np.zeros_like(depth_float)
        depth_log[valid_mask] = np.log1p(depth_float[valid_mask])  # log1p = log(1+x)
        
        # 归一化
        valid_log = depth_log[valid_mask]
        log_min, log_max = np.min(valid_log), np.max(valid_log)
        
        depth_normalized = np.zeros_like(depth_log)
        depth_normalized[valid_mask] = (depth_log[valid_mask] - log_min) / (log_max - log_min)
        
        # 反转颜色（近红远蓝）
        depth_normalized[valid_mask] = 1.0 - depth_normalized[valid_mask]
        
        depth_8bit = (depth_normalized * 255).astype(np.uint8)
        depth_colored = cv2.applyColorMap(depth_8bit, cv2.COLORMAP_JET)
        depth_colored[~valid_mask] = [0, 0, 0]
        
        return depth_colored

    def publish_color_data(self, packet):
        """处理和发布彩色图像数据"""
        try:
            # 从packet获取彩色图像
            color_frame = packet.frame
            
            # 将彩色图像转换为ROS消息
            color_msg = self.bridge.cv2_to_imgmsg(color_frame, encoding="bgr8")
            color_msg.header.stamp = self.get_clock().now().to_msg()
            color_msg.header.frame_id = "oak_color_frame"
            
            # 发布彩色图像
            self.color_publisher.publish(color_msg)
            
        except Exception as e:
            self.get_logger().error(f'处理彩色数据时发生错误: {str(e)}')

    def calculate_and_publish_fps(self):
        """计算并发布帧率"""
        self.frame_count += 1
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_time).nanoseconds / 1e9
        
        # 每秒更新一次帧率
        if time_diff >= 1.0:
            fps = self.frame_count / time_diff
            fps_msg = Float32()
            fps_msg.data = float(fps)
            self.fps_publisher.publish(fps_msg)
            
            # 在控制台也输出帧率信息
            self.get_logger().info(f'当前帧率: {fps:.2f} FPS', throttle_duration_sec=1.0)
            
            # 重置计数器
            self.frame_count = 0
            self.last_time = current_time

    def get_current_fps(self):
        """获取当前帧率（用于显示在图像上）"""
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_time).nanoseconds / 1e9
        if time_diff > 0:
            return self.frame_count / time_diff
        return 0.0

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = OakDepthPublisher()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()