# oak_depth_processing
用于OAK相机深度图采集、可视化、参数调优和数据发布

## 目录结构

```
oak_depth_calibration/
├── oak_align_standard_depthaiSDK.py    # 利用官方SDK接口,完成参数设置
├── oak_align_standard_combine.py       # 综合利用SDK api接口
├── oak_align_standard_depthai.py       # 利用官方api接口,在底层node完成参数设置
└── README.md                           # 项目说明文档
```
综合利用SDK api接口的优化效果是最理想的，如果需要对特定的情景进行调参，直接在
oak_align_standard_combine.py 的参数列表中进行修改

## 功能特性

### SDK增强版深度发布器 (oak_align_standard_combine.py)
- **ROS 2支持**: 通过ROS 2节点发布深度数据
- **参数配置**: 支持多种滤波和校准参数
- **硬件优化**: 可配置硬件资源分配
- **多话题发布**: 发布原始深度图、彩色深度图、视差图

### 简化版深度发布工具 (oak_align_standard_depthai.py)
- **轻量级实现**: 简化的深度数据采集和发布
- **ROS 2支持**: 基础的ROS 2节点功能

**发布的话题**:
- `/camera/depth/image_raw` - 原始深度图
- `/camera/color/image_raw` - 彩色图像
- `/camera/depth/colormap` - 彩色化深度图
- `/camera/disparity` - 视差图
- `/camera/fps` - 帧率信息

## 关键参数设置原理

### 立体深度节点参数

#### 1. 置信度阈值 (Confidence Threshold)
```python
stereo.setConfidenceThreshold(220)  # 范围: 0-255
```
**原理**: 过滤低置信度的深度值，提高深度图质量
- **低值**: 更严格的过滤，减少噪声但可能增加空洞
- **高值**: 保留更多数据但可能包含更多噪声

#### 2. 左右一致性检查 (Left-Right Check)
```python
stereo.setLeftRightCheck(True)
```
**原理**: 验证左右视差图的一致性，过滤错误匹配
- **True**: 提高深度图精度，增加计算量
- **False**: 减少计算量，可能包含更多错误
实际在调参过程中对结果的影响很小

#### 3. 扩展视差 (Extended Disparity)
```python
stereo.setExtendedDisparity(True)
```
**原理**: 扩展视差范围，提高近处物体的深度精度
- **True**: 增加视差范围，提高近距离精度
- **False**: 标准视差范围，适合中等距离
Extended Disparity是否开启，会在根本上影响参数设置的逻辑

#### 4. 亚像素精度 (Subpixel)
```python
stereo.setSubpixel(True)
stereo.setSubpixelBits(3)  # 亚像素位数
```
**原理**: 提供亚像素级别的视差计算，提高深度图分辨率
- **高位数**: 高精度 
- **低位数**: 低精度 

### 滤波参数

#### 1. 中值滤波 (Median Filter)
```python
stereo.setMedianFilter(dai.MedianFilter.KERNEL_5x5)
```
**原理**: 减少椒盐噪声，平滑深度图
- **KERNEL_3x3**: 轻微平滑
- **KERNEL_5x5**: 中等平滑
- **KERNEL_7x7**: 强平滑

#### 2. WLS滤波 (Weighted Least Squares)
```python
stereo.config_wls(
    wls_level=WLSLevel.HIGH,
    wls_lambda=8000,
    wls_sigma=1.5
)
```
**原理**: 利用RGB图像的边缘信息优化深度图
- **wls_level**: 滤波强度 (LOW, MEDIUM, HIGH)
- **wls_lambda**: 平滑强度参数
- **wls_sigma**: 边缘感知参数

#### 3. 时域滤波 (Temporal Filter)
```python
config.postProcessing.temporalFilter.enable = True
```
**原理**: 利用时间相关性平滑深度图，减少动态噪声

#### 4. 散斑滤波 (Speckle Filter)
```python
config.postProcessing.speckleFilter.enable = True
config.postProcessing.speckleFilter.differenceThreshold = 2
config.postProcessing.speckleFilter.speckleRange = 50
```
**原理**: 去除小的孤立区域（散斑噪声）
- **differenceThreshold**: 相邻像素差异阈值
- **speckleRange**: 散斑区域大小限制


### 硬件资源配置

```python
stereo_node.setPostProcessingHardwareResources(
    hardware_shaves=3,
    hardware_memory_slices=2
)
```
**原理**: 配置硬件资源分配，平衡性能和资源占用
- **hardware_shaves**: 分配的SHAVE处理器数量 (1-14)
- **hardware_memory_slices**: 分配的内存切片数量 (1-3)


## 性能优化建议

### 帧率优化
- 降低分辨率可以提高帧率（分辨率需最先设置，否则分辨率的变化会影响滤波的效果）
- 减少滤波器的数量可以显著提高处理速度
- 关闭不必要的功能（如LR check）

### 深度质量优化
- 启用亚像素模式提高分辨率
- 调整置信度阈值过滤噪声

### 内存优化
- 合理配置硬件资源分配

## 常见问题解决

### 1. 深度图出现大量空洞
- 降低置信度阈值
- 关闭或调整散斑滤波
- 检查相机校准状态

### 2. 帧率过低
- 降低相机分辨率
- 减少滤波强度
- 优化硬件资源配置

### 3. 深度图与RGB图像不对齐
- 确保深度对齐配置正确
- 检查相机校准数据
- 调整ROI坐标转换参数

## 参考资料

- [DepthAI官方文档]([https://docs.luxonis.com/software/])


**作者**: VAN
