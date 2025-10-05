import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # 找到我们的参数文件
    pkg_share_dir = get_package_share_directory('hik_camera_ros2')
    parameter_file = os.path.join(pkg_share_dir, 'config', 'camera_params.yaml')

    # 创建相机节点，并直接告诉它使用YAML文件作为参数源
    hik_camera_node = Node(
        package='hik_camera_ros2',
        executable='hik_camera_node',
        name='hik_camera',
        output='screen',
        emulate_tty=True,
        # 这是唯一的参数源，干净利落
        parameters=[parameter_file]
    )

    return LaunchDescription([
        hik_camera_node
    ])