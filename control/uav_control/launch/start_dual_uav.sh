#!/bin/bash

# 双无人机系统启动脚本
# 用法: ./start_dual_uav.sh [选项]
# 选项: 
#   full - 启动完整系统（LIDAR + 控制 + 规划）
#   fast_lio_only - 仅启动LIDAR处理
#   control_only - 仅启动控制节点
#   planning_only - 仅启动规划节点
#   mavros_only - 仅启动MAVROS

set -e

MODE=${1:-"full"}

echo "===================================="
echo "双无人机系统启动脚本"
echo "模式: $MODE"
echo "===================================="

case $MODE in
    full)
        echo "启动完整双无人机系统..."
        roslaunch uav_control start_dual_uav.launch
        ;;
    fast_lio_only)
        echo "启动LIDAR处理节点..."
        echo "启动UAV1 LIDAR..."
        gnome-terminal -- bash -c "roslaunch fast_lio mapping_mid360.launch uav_name:=uav1; read"
        sleep 2
        echo "启动UAV2 LIDAR..."
        gnome-terminal -- bash -c "roslaunch fast_lio mapping_mid360.launch uav_name:=uav2; read"
        ;;
    control_only)
        echo "启动控制节点..."
        echo "启动UAV1 控制..."
        gnome-terminal -- bash -c "roslaunch uav_control control_real_jy2025.launch uav_name:=uav1; read"
        sleep 2
        echo "启动UAV2 控制..."
        gnome-terminal -- bash -c "roslaunch uav_control control_real_jy2025.launch uav_name:=uav2; read"
        ;;
    planning_only)
        echo "启动规划节点..."
        echo "启动UAV1 规划..."
        gnome-terminal -- bash -c "roslaunch super_planner jy_real.launch uav_name:=uav1 start_x:=0.0 start_y:=0.0 start_z:=1.0; read"
        sleep 2
        echo "启动UAV2 规划..."
        gnome-terminal -- bash -c "roslaunch super_planner jy_real.launch uav_name:=uav2 start_x:=1.0 start_y:=0.0 start_z:=1.0; read"
        ;;
    mavros_only)
        echo "启动MAVROS..."
        roslaunch uav_control start_dual_uav_mavros.launch
        ;;
    *)
        echo "未知模式: $MODE"
        echo "支持的模式: full, fast_lio_only, control_only, planning_only, mavros_only"
        exit 1
        ;;
esac
