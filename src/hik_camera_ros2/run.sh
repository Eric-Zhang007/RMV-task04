
#!/bin/zsh
source /home/eric/RMVcode/RMV-task4/ROS-HIk_MVS-SDK/install/local_setup.zsh
export LD_LIBRARY_PATH=$(pwd)/external/MVS/lib/64:$LD_LIBRARY_PATH 
echo $LD_LIBRARY_PATH
ros2 launch hik_camera_ros2 hik_camera.launch.py "$@"