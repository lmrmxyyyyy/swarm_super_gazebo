#! /usr/bin/python3.8

from __future__ import print_function, division, absolute_import

import copy
import threading
import time
import struct
import open3d as o3d
import yaml
import rospy
# import ros_numpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
import std_msgs.msg
import sensor_msgs.msg
import numpy as np
import tf
import tf.transformations
import os
from collections import deque
from std_msgs.msg import Header
from std_msgs.msg import Int8, Float64

BASEPATH = os.path.dirname(os.path.abspath(__file__))
global_map = None
initialized = False
T_map_to_odom = np.eye(4)
cur_odom = None
cur_scan = None
start_value =None
end_value =None
init_map_flag=False






def voxel_down_sample(pcd, voxel_size):
    # try:
    pcd_down = pcd.voxel_down_sample(voxel_size)
    # except:
    #     # for opend3d 0.7 or lower
    #     pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
    return pcd_down




def initialize_global_map(file_path):
    tic = time.time()
    global global_map
    global_map = o3d.io.read_point_cloud(file_path)  # Load the PCD file using Open3D
    rospy.loginfo('Global map loaded from file: {}'.format(file_path))

    # # Voxel downsample the point cloud
    global_map = voxel_down_sample(global_map, MAP_VOXEL_SIZE)

   

    # 保存体素化后的点云为新的 .pcd 文件
    output_file_path = file_path.replace('.pcd', '_downsampled.pcd')
    o3d.io.write_point_cloud(output_file_path, global_map)
    toc = time.time()
    
    rospy.loginfo('ALL Time: {}'.format(toc - tic))
     # 可视化体素化后的点云
    rospy.loginfo('Visualizing voxel downsampled global map...')
    o3d.visualization.draw_geometries([global_map])
    rospy.loginfo('Voxel downsampled global map saved to: {}'.format(output_file_path))

    rospy.loginfo('Global map initialized and processed.')





if __name__ == '__main__':
    MAP_VOXEL_SIZE = 0.1

    
    rospy.logwarn(f'Waiting for global map......')
    # 拼接 scans.pcd 的路径（从 scripts/ 向上跳一级到项目根）
    file_path = os.path.join(BASEPATH, '../PCD/scans.pcd')
    print(BASEPATH)
    initialize_global_map(file_path)

    # publisher

   


    # # 初始化
    # while not initialized:
    #     rospy.logwarn('Waiting for initial pose....')

    #     # 等待初始位姿
    #     pose_msg = rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)
    #     initial_pose = pose_to_mat(pose_msg)
    #     if cur_scan:
    #         initialized = global_localization(initial_pose)
    #     else:
    #         rospy.logwarn('First scan not received!!!!!')

    rospy.loginfo('')
    # rospy.loginfo('Initialize successfully!!!!!!')
    rospy.loginfo('')
    # 开始定期全局定位
    # threading.Thread(target=thread_localization)


