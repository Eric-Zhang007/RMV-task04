# ROS2 Driver for Hikvision MVS Cameras

这是一个为海康威视机器视觉工业相机 (Hikvision MVS) 开发的ROS2 Humble功能包。它通过封装官方的MVS SDK，使得在ROS2项目中使用海康相机变得简单高效。

## ✨ 核心功能

- **自动连接**: 节点启动后可自动发现并连接到指定IP地址的相机。
- **断线重连**: 在相机掉线或重启后，节点会自动尝试重新连接。
- **高效图像发布**: 在独立的抓图线程中获取图像数据，并以`sensor_msgs/msg/Image`格式发布到`image_raw`话题，支持高帧率。
- **动态参数配置**: 通过ROS2参数系统，可以实时读取和修改相机的常用参数，如曝光时间、增益和帧率。
- **可移植性**: 项目将核心的MVS SDK库文件包含在内，无需在系统中单独安装MVS SDK即可编译。

## ⚙️ 环境要求

- Ubuntu 22.04
- ROS2 Humble Hawksbill
- OpenCV (通常随ROS2 Desktop Full一同安装)
- MVS Runtime: 虽然项目编译**不依赖**外部SDK，但**运行时**目标机器上需要安装海康MVS的**运行库(Runtime)**。请从[海康机器人官网](https://www.hikrobotics.com/cn/machine-vision/service/download)下载并安装。

## 📦 编译指南

1.  **克隆本仓库**:
    将本功能包克隆到你的ROS2工作区的`src`目录下。

    ```bash
    # 假设你的工作区在 ~/ros2_ws
    cd ~/ros2_ws/src
    git clone [你的Git仓库URL]
    ```

2.  **安装依赖 (关键步骤!)**:
    回到工作区根目录，使用`rosdep`来自动安装所有缺失的公共ROS依赖（如`cv_bridge`）。

    ```bash
    cd ~/ros2_ws
    sudo apt update
    rosdep install --from-paths src -y --ignore-src
    ```

3.  **编译功能包**:
    回到工作区根目录，使用`colcon`进行编译。

    ```bash
    cd ~/ros2_ws
    colcon build --packages-select hik_camera_ros2
    ```

## 🚀 运行节点

在运行前，请确保你已经正确配置了参数文件。

1.  **配置参数**:
    打开文件 `config/camera_params.yaml`，修改以下关键参数：
    - `camera_ip`: **必须**修改为你相机的实际IP地址。
    - `camera_info_url`: (可选) 相机标定文件的路径。

2.  **Source工作区**:
    在**新的终端**中，source你的工作区配置文件。

    ```bash
    source ./install/local_setup.zsh //.bash or .ps1
    ```

3.  **启动相机节点**:
    使用我们提供的launch文件来启动节点。
    ```bash
    export LD_LIBRARY_PATH=$(pwd)/src/hik_camera_ros2/external/MVS/lib/64:$LD_LIBRARY_PATH 
    ```

    ```bash
    ros2 launch hik_camera_ros2 hik_camera.launch.py
    ```
    
## ✅ 验证与使用

1.  **查看图像**:
    启动`RViz2`：
    ```bash
    rviz2
    ```
    - 点击左下角的 "Add" 按钮。
    - 选择 "By topic" 标签页。
    - 找到 `/image_raw` 话题下的 "Image" 并点击 "OK"。
    - 你应该能在RViz2中看到相机发布的实时图像。

2.  **查看和修改参数**:
    - **列出所有参数**:
      ```bash
      ros2 param list
      ```
      (你应该能看到 `/hik_camera` 节点下的 `exposure_time`, `gain` 等参数)

    - **获取当前曝光值**:
      ```bash
      ros2 param get /hik_camera exposure_time
      ```
    
    - **动态设置新的曝光值**:
      ```bash
      ros2 param set /hik_camera exposure_time 8000.0
      ```
      (观察RViz2中的图像，亮度应该会发生实时变化)