#! /usr/bin/python3.8

import numpy as np
import time

# ROS
import csv
import rospy

from ius_msgs.msg import Trajectory
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
#
import os, sys

from math import cos, sin

rospy.init_node("test_traj_pub")
tarj_publisher = rospy.Publisher("/mpc_trajectory", Trajectory, tcp_nodelay=True, queue_size=1)

marker_pub = rospy.Publisher('/mpc_trajectory_marker', MarkerArray, queue_size=10)


BASEPATH = os.path.abspath(__file__).split('script', 1)[0]+'script/function_model/ref_traj/'
sys.path += [BASEPATH]

trajectory_pub = rospy.Publisher("/ius_uav/trajectory", Trajectory, tcp_nodelay=True, queue_size=1)

start_time=None
def traj_pub():
    global start_time
    
    # points = [[0.0, 0.0, 0.125],
    #           [0.05, 0.0, 0.225],
    #           [0.25, 0, 0.325],
    #           [0.45, 0.0, 0.425],
    #           [0.8, 0.0, 0.525],
    #           [1.3, 0.0, 0.625],
    #           [1.8, 0.0, 0.725],
    #           [2.3, 0.0, 0.825],
    #           [2.8, 0.0, 0.9],
    #           [3.3, 0.0, 1.0],
    #           [3.8, 0.0, 1.0],
    #           [4.3, 0.0, 1.0],
    #           ]
    points = [
            [0.0, 0.0, 0.2],
            [0.0, 0.0, 0.3],
            [0.0, 0.0, 0.4],
            [0.0, 0.0, 0.5],
            [0.0, 0.0, 0.6],
            [0.0, 0.0, 0.7],
            [0.0, 0.0, 0.8],
            [0.0, 0.0, 0.9],
            [0.0, 0.0, 1.0],
            [0.0, 0.0, 1.1],
            [0.0, 0.0, 1.2],
            [0.0, 0.0, 1.3],
            [0.0, 0.0, 1.4],
            [0.0, 0.0, 1.5],
              ]
    # 创建轨迹数据
    if start_time == None:
        start_time=rospy.Time.now()
    
    traj_msg=Trajectory()
    traj_msg.header.stamp = start_time
    traj_msg.header.frame_id="map"
    traj_msg.traj_id = 1  # 轨迹 ID
    for i, p in enumerate(points):
        # 设置位置点
        point = Point()
        point.x, point.y, point.z = p
        traj_msg.pos.append(point)
        # 设置偏航角 (yaw)
        # yaw = 90/180*3.14159 
        yaw=0 
        traj_msg.yaw.append(yaw)
        # 设置时间
        traj_msg.time.append(i * 0.3)  # 时间间隔 0.1s
    tarj_publisher.publish(traj_msg)
    # 创建 MarkerArray 用于 Rviz 显示
    # marker_array = MarkerArray()
    # for i, p in enumerate(points):
    #     point_marker = Marker()
    #     point_marker.header.frame_id = "camera_init"  # 替换为你的坐标系
    #     point_marker.header.stamp = rospy.Time.now()
    #     point_marker.ns = "fixed_points"
    #     point_marker.id = i
    #     point_marker.type = Marker.SPHERE
    #     point_marker.action = Marker.ADD
    #     point_marker.pose.position.x, point_marker.pose.position.y, point_marker.pose.position.z = p
    #     point_marker.pose.orientation.w = 1.0  # 默认朝向
    #     point_marker.scale.x = 0.1  # 点的大小
    #     point_marker.scale.y = 0.1
    #     point_marker.scale.z = 0.1
    #     point_marker.color.r = 0.0
    #     point_marker.color.g = 1.0##绿色
    #     point_marker.color.b = 0.0
    #     point_marker.color.a = 1.0
    #     marker_array.markers.append(point_marker)
    # # 发布 MarkerArray
    # marker_pub.publish(marker_array)
    # rospy.loginfo("Markers published to Rviz.")
    
    rospy.loginfo("published.")



if __name__ == "__main__":
    
    # timer

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        traj_pub()
        rate.sleep()