#! /usr/bin/python3.8

from __future__ import print_function, division, absolute_import


import time
import struct
import open3d as o3d
import rospy
# import ros_numpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import numpy as np
import tf
import tf.transformations
import os
from collections import deque
from std_msgs.msg import Header
from std_msgs.msg import Int8, Float64
import yaml
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance, Pose, Point, Quaternion, Twist, Vector3
from scipy.spatial.transform import Rotation as R
global_map = None
initialized = False
T_map_to_odom = np.eye(4)
cur_odom = None
cur_scan = None
start_value =None
end_value =None
init_map_flag=False
transform_matrix=None
state_machine=None
initial_pose_inited=False


def odom_to_mat(pose_msg):
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )

def pose_to_mat(pose_msg):
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.position),
        tf.listener.xyzw_to_mat44(pose_msg.orientation),
    )

# def msg_to_array(pc_msg):
#     pc_array = ros_numpy.numpify(pc_msg)
#     pc = np.zeros([len(pc_array), 3])
#     pc[:, 0] = pc_array['x']
#     pc[:, 1] = pc_array['y']
#     pc[:, 2] = pc_array['z']
#     return pc

def msg_to_array(pc_msg):
    # 获取消息中的数据字节
    data = pc_msg.data
    # 每个点的字节大小
    point_step = pc_msg.point_step
    # 点云的宽度和高度
    num_points = pc_msg.width * pc_msg.height

    # 创建一个 NumPy 数组来存储点云数据
    pc = np.zeros((num_points, 3), dtype=np.float32)

    # 解析数据并填充 NumPy 数组
    for i in range(num_points):
        point_data = data[i * point_step:(i + 1) * point_step]  # 获取单个点的数据
        x, y, z = struct.unpack('fff', point_data[:12])  # 解包为三个浮点数
        pc[i, 0] = x  # x 坐标
        pc[i, 1] = y  # y 坐标
        pc[i, 2] = z  # z 坐标

    return pc



def registration_at_scale(pc_scan, pc_map, initial, scale):
    # Perform voxel downsampling and registration at scale
    #o3d.visualization.draw_geometries([voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE)])
    result_icp = o3d.pipelines.registration.registration_icp(
        voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale),
        # voxel_down_sample(pc_map, MAP_VOXEL_SIZE * scale),
        pc_map,
        1.0 * scale,
        initial,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=5000)
    )

    return result_icp.transformation, result_icp.fitness


def inverse_se3(trans):
    trans_inverse = np.eye(4)
    # R
    trans_inverse[:3, :3] = trans[:3, :3].T
    # t
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])
    return trans_inverse



def point_cloud_to_ros_msg(o3d_pc):
    """
    Converts Open3D PointCloud to ROS PointCloud2 message without using ros_numpy.
    """
    # Get points as numpy array from Open3D point cloud
    points = np.asarray(o3d_pc.points)  # Shape (N, 3)

    # Create a PointCloud2 message
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'camera_init'

    # Pack data for PointCloud2 message
    # 'x', 'y', 'z' are the only fields here, each is a float32
    pc_data = np.zeros(len(points), dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)])
    pc_data['x'] = points[:, 0]
    pc_data['y'] = points[:, 1]
    pc_data['z'] = points[:, 2]

    # Create the PointCloud2 message using a byte array
    pc_msg = PointCloud2()
    pc_msg.header = header
    pc_msg.height = 1  # Single row (flat point cloud)
    pc_msg.width = len(points)  # Number of points
    pc_msg.is_dense = True  # If the cloud contains NaN values (false) or not (true)
    pc_msg.is_bigendian = False  # Little-endian (most common in ROS)

    # Define fields for the point cloud data using PointField
    pc_msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
    ]
    
    # Set the point step (bytes per point)
    pc_msg.point_step = 12  # 3 float32 (x, y, z) = 12 bytes

    # Set the row step (bytes per row)
    pc_msg.row_step = pc_msg.point_step * pc_msg.width

    # Convert the structured numpy array to byte array
    pc_msg.data = pc_data.tobytes()

    return pc_msg

def crop_global_map_in_FOV(global_map, pose_estimation,scan):
    # 当前scan原点的位姿
    # T_odom_to_base_link = odom_to_mat(cur_odom)
    T_init_to_base_link = odom_to_mat(pose_estimation)
    # T_map_to_base_link = np.matmul(T_init_to_base_link, T_odom_to_base_link)
    T_base_link_to_map = inverse_se3(T_init_to_base_link)

    # 把地图转换到lidar系下
    global_map_in_map = np.array(global_map.points)
    global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
    global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

    # 将视角内的地图点提取出来
    if FOV >= 3.14:
        # 环状lidar 仅过滤距离
        indices = np.where(
            (np.abs(global_map_in_base_link[:, 0]) < FOV_FAR) &
            (np.abs(global_map_in_base_link[:, 1]) < FOV_FAR) &
            (np.abs(global_map_in_base_link[:, 2]) < FOV_FAR)
            # (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    else:
        # 非环状lidar 保前视范围
        # FOV_FAR>x>0 且角度小于FOV
        indices = np.where(
            (global_map_in_base_link[:, 0] > 0) &
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    global_map_in_FOV = o3d.geometry.PointCloud()
    global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3]))

    # 发布fov内点云
    # 创建一个 Header 对象
    header = Header()
    # 设置当前时间戳
    global_map_in_FOV_msg = point_cloud_to_ros_msg(global_map_in_FOV)
    pub_submap.publish(global_map_in_FOV_msg)
    # o3d.visualization.draw_geometries([global_map_in_FOV])
    return global_map_in_FOV

def compute_ransac_initial(scan, map_in_FOV, voxel_size=1.0):
    """
    用 FPFH + RANSAC 做 scan-to-map 粗对齐，返回初始刚体变换矩阵。
    
    Args:
        scan (o3d.geometry.PointCloud): 当前帧点云
        map_in_FOV (o3d.geometry.PointCloud): 截取后的局部地图
        voxel_size (float): 降采样体素大小，默认 1m
    
    Returns:
        np.ndarray: 4x4 transformation matrix
    """
    # 1. 下采样
    source_down = scan.voxel_down_sample(voxel_size)
    target_down = map_in_FOV.voxel_down_sample(voxel_size)

    # 2. 法向估计
    source_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    target_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))

    # 3. FPFH 特征
    source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        source_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100))
    target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        target_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100))

    # 4. RANSAC 全局粗配准
    result_ransac = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down,
        source_fpfh, target_fpfh,
        mutual_filter=True,
        max_correspondence_distance=voxel_size * 1.5,
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        ransac_n=4,
        checkers=[
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(voxel_size * 1.5)
        ],
        criteria=o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 1000)
    )

    # 提取平移和平移向量
    T = result_ransac.transformation
    translation = T[:3, 3]
    rotation_matrix = T[:3, :3]

    # 用 scipy 直接转换欧拉角（ZYX: yaw-pitch-roll）
    rotation_matrix = rotation_matrix.copy()
    r = R.from_matrix(rotation_matrix)
    yaw, pitch, roll = r.as_euler('zyx', degrees=True)  # 注意 Open3D 默认世界系右手

    print("\n[compute_ransac_initial] ---------------------------")
    print(f"[RANSAC] Fitness (inlier ratio)   : {result_ransac.fitness:.4f}")
    print(f"[RANSAC] Inlier RMSE              : {result_ransac.inlier_rmse:.4f}")
    print(f"[RANSAC] #Correspondences (inlier): {len(result_ransac.correspondence_set)}")
    print(f"[RANSAC] Translation (x y z)      : [{translation[0]:.3f}, {translation[1]:.3f}, {translation[2]:.3f}]")
    print(f"[RANSAC] Rotation  (roll pitch yaw): [{roll:.2f}°, {pitch:.2f}°, {yaw:.2f}°]")
    print("[RANSAC] Estimated Transformation:\n", result_ransac.transformation)
    print("[compute_ransac_initial] RANSAC done.")
    print("-----------------------------------------------------\n")
    return result_ransac.transformation

def global_localization(pose_estimation,scan):
    global global_map, T_map_to_odom,initial_pose
    tic = time.time()
    # 用icp配准
    # print(global_map, cur_scan, T_map_to_odom)
    rospy.loginfo('Global localization by scan-to-map matching......')

    # TODO 这里注意线程安全
    # scan_tobe_mapped = copy.copy(cur_scan)



    global_map_in_FOV = crop_global_map_in_FOV(global_map, pose_estimation,scan)
    ## --------------------
    ## 1️⃣ FPFH 粗配准
    ## --------------------

    T_ransac = compute_ransac_initial(scan, global_map_in_FOV, voxel_size=1.0)
    ## --------------------
    ## 2️⃣ ICP 多层细配准
    ## --------------------
    # # 粗配准
    transformation, fitness = registration_at_scale(scan, global_map_in_FOV,
                            initial=T_ransac, scale=3.0)   #initial=odom_to_mat(pose_estimation)
    print("fitness1:",fitness)
    # 精配准
    transformation, fitness = registration_at_scale(scan, global_map_in_FOV,
                              initial=transformation, scale=1.0)
    # transformation, fitness = registration_at_scale(cur_scan, global_map_in_FOV,
    #                             initial=odom_to_mat(pose_estimation), scale=1)
    print("fitness2:",fitness)
    
    transformation, fitness = registration_at_scale(scan, global_map_in_FOV,
                              initial=transformation, scale=0.3)
    print("fitness3:",fitness)
    toc = time.time()
    rospy.loginfo('配准 Time: {}'.format(toc - tic))
    rospy.loginfo('')
    
    # 当全局定位成功时才更新map2odom
    if fitness > LOCALIZATION_TH:
        # T_map_to_odom = np.matmul(transformation, pose_estimation)
        T_map_to_odom = transformation

        # 发布map_to_odom
        map_to_odom = Odometry()
        xyz = tf.transformations.translation_from_matrix(T_map_to_odom)
        quat = tf.transformations.quaternion_from_matrix(T_map_to_odom)
        map_to_odom.pose.pose = Pose(Point(*xyz), Quaternion(*quat))
        map_to_odom.header.stamp =  rospy.Time.now()
        map_to_odom.header.frame_id = 'camera_init'
        pub_map_to_odom.publish(map_to_odom)
        # 提取平移
        xyz = tf.transformations.translation_from_matrix(T_map_to_odom)
        # 提取四元数
        quat = tf.transformations.quaternion_from_matrix(T_map_to_odom)
        # 转欧拉角 (roll, pitch, yaw)
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
        # 转成角度可读
        roll_deg = np.degrees(roll)
        pitch_deg = np.degrees(pitch)
        yaw_deg = np.degrees(yaw)
        print("\n[map_to_odom] ---------------------------")
        print(f"Translation (x y z): [{xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f}]")
        print(f"Rotation   (roll pitch yaw): [{roll_deg:.2f}°, {pitch_deg:.2f}°, {yaw_deg:.2f}°]")
        print("-----------------------------------------\n")
        # print(map_to_odom)
        initial_pose=map_to_odom
        return True
    else:
        rospy.logwarn('Not match!!!!')
        rospy.logwarn('{}'.format(transformation))
        rospy.logwarn('fitness score:{}'.format(fitness))
        return False


def voxel_down_sample(pcd, voxel_size):
    # try:
    pcd_down = pcd.voxel_down_sample(voxel_size)
    # except:
    #     # for opend3d 0.7 or lower
    #     pcd_down = o3d.geometry.voxel_down_sample(pcd, voxel_size)
    return pcd_down

def get_transformation_matrix(start_pose):
    """ 根据起始位置和四元数构建变换矩阵 """
    x, y, z = start_pose["position"]["x"], start_pose["position"]["y"], start_pose["position"]["z"]
    qx, qy, qz, qw = start_pose["orientation"]["x"], start_pose["orientation"]["y"], start_pose["orientation"]["z"], start_pose["orientation"]["w"]

    # 旋转矩阵
    rotation_matrix = Rotation.from_quat([qx, qy, qz, qw]).as_matrix()

    # 构建 4x4 变换矩阵
    transform = np.eye(4)
    transform[:3, :3] = rotation_matrix
    transform[:3, 3] = [x, y, z]

    return np.linalg.inv(transform)  # 计算全局到局部的变换

def load_yaml(file_path):
    with open(file_path, 'r') as f:
        data = yaml.safe_load(f)
    return data

def transform_point_cloud(global_map, transformation_matrix):
    """ 应用变换矩阵到点云 """
    global_map.transform(transformation_matrix)
    return global_map

def initialize_global_map(file_path):
    global global_map
    global_map = o3d.io.read_point_cloud(file_path)  # Load the PCD file using Open3D
    rospy.loginfo('Global map loaded from file: {}'.format(file_path))

    # # Voxel downsample the point cloud
    global_map = voxel_down_sample(global_map, MAP_VOXEL_SIZE)

    # # 可视化体素化后的点云
    # rospy.loginfo('Visualizing voxel downsampled global map...')
    # o3d.visualization.draw_geometries([global_map])

    # # 保存体素化后的点云为新的 .pcd 文件
    # output_file_path = file_path.replace('.pcd', '_downsampled.pcd')
    # o3d.io.write_point_cloud(output_file_path, global_map)
    # rospy.loginfo('Voxel downsampled global map saved to: {}'.format(output_file_path))

    rospy.loginfo('Global map initialized and processed.')

# 存储最近的50帧数据
max_odom_queue_size = 10
odom_queue = deque(maxlen=max_odom_queue_size)

initial_pose = Odometry()
# 四元数转换为旋转矩阵
def quaternion_to_rotation_matrix(q):
    # 使用 tf.transformations 库来进行转换
    q = np.array([q.x, q.y, q.z, q.w])
    return tf.transformations.quaternion_matrix(q)[:3, :3]  # 只取旋转部分

# 旋转矩阵转换为四元数
def rotation_matrix_to_quaternion(R):
    # 使用 tf.transformations 库将旋转矩阵转换为四元数
    return tf.transformations.quaternion_from_matrix(np.vstack([np.hstack([R, np.zeros((3, 1))]), np.array([0, 0, 0, 1])]))

# def initial_odom_cbk(msg_in):
#     global initial_pose_inited, initial_pose, odom_queue
#     global cur_odom
#     if not initial_pose_inited:  # 初始化位置
#         # 将当前位置数据存储到队列中
#         odom_queue.append(msg_in)

#         # 如果队列大小超过50帧，移除最早的帧
#         if len(odom_queue) > max_odom_queue_size:
#             odom_queue.popleft()

#         # 计算位置的平均值
#         avg_x = 0
#         avg_y = 0
#         avg_z = 0
#         rotation_sum = np.zeros((3, 3))  # 初始化旋转矩阵求和

#         for odom in odom_queue:
#             avg_x += odom.pose.pose.position.x
#             avg_y += odom.pose.pose.position.y
#             avg_z += odom.pose.pose.position.z

#             # 获取四元数并转换为旋转矩阵
#             q = odom.pose.pose.orientation
#             R = quaternion_to_rotation_matrix(q)
#             rotation_sum += R  # 累加旋转矩阵

#         # 计算旋转矩阵的平均值
#         avg_rotation = rotation_sum / len(odom_queue)

#         # 将平均旋转矩阵转换回四元数
#         avg_orientation = rotation_matrix_to_quaternion(avg_rotation)

#         # 计算位置的平均值
#         avg_x /= len(odom_queue)
#         avg_y /= len(odom_queue)
#         avg_z /= len(odom_queue)

#         # 将位置和方向的平均值赋值给 initial_pose
#         initial_pose.pose.pose.position.x = avg_x
#         initial_pose.pose.pose.position.y = avg_y
#         initial_pose.pose.pose.position.z = avg_z

#         # 将四元数的平均值赋值给 initial_pose 的方向
#         initial_pose.pose.pose.orientation.x = avg_orientation[0]
#         initial_pose.pose.pose.orientation.y = avg_orientation[1]
#         initial_pose.pose.pose.orientation.z = avg_orientation[2]
#         initial_pose.pose.pose.orientation.w = avg_orientation[3]

#         # 设置初始化标志
#         if len(odom_queue) >= max_odom_queue_size:
#             initial_pose_inited = True
#             rospy.loginfo("Initial pose initialized successfully")
#             print(initial_pose)
#             # cur_odom=Odometry()
#             # cur_odom.pose.pose.position = initial_pose.pose.pose.position
#             # cur_odom.pose.pose.orientation = initial_pose.pose.pose.orientation
            
            
#         # print(initial_pose)
#         # print(initial_pose_inited)
        
def cb_save_cur_odom(odom_msg):
    global cur_odom,initial_pose_inited
    cur_odom = odom_msg
    # print(cur_odom)
    initial_pose_inited = True


accumulated_scans = o3d.geometry.PointCloud()
accumulated_count = 0
ACCUM_LIMIT = 30  # 累积5帧
initial_pose_inited = True
first_match_flag=False

init_odom = Odometry()
    # 设置 header
init_odom.header.frame_id = "camera_init"          # 可按需要修改
init_odom.child_frame_id = ""
    # 设置位置（可根据实际设定）
init_odom.pose.pose = Pose(
        position=Point(x=0.0, y=0.0, z=-0.0),
        orientation=Quaternion(x=-0.0, y=  0.0, z= -0.0 , w= 1.0)
    )
    # 设置速度（线速度 + 角速度）
init_odom.twist.twist = Twist(
        linear=Vector3(x=0.0, y=0.0, z=0.0),
        angular=Vector3(x=0.0, y=0.0, z=0.0)
    )

def cb_save_cur_scan(pc_msg):
    global initial_pose_inited, init_map_flag
    global accumulated_scans, accumulated_count,first_match_flag
    
    if not (initial_pose_inited and init_map_flag):
        rospy.logwarn(f'Waiting for initial_pose_inited and init_map_flag......')
        print("initial_pose_inited",initial_pose_inited)
        print("init_map_flag",init_map_flag)
        return
    tic = time.time()
    global cur_scan
    points = []
    for point in point_cloud2.read_points(pc_msg, field_names=("x", "y", "z"), skip_nans=True):
        points.append([point[0], point[1], point[2]])
    
    # 2. 转换为NumPy数组
    points_np = np.array(points, dtype=np.float32)

    cur_scan = o3d.geometry.PointCloud()
    cur_scan.points = o3d.utility.Vector3dVector(points_np)
    # 对点云进行范围限制（类似 PCL 的 CropBox）
    # 例如，只保留 x, y, z 在某个范围内的点
    min_bound = np.array([-50, -50, 2])  # 范围最小值（例如 -5, -5, -5）
    max_bound = np.array([50, 50, 20])  # 范围最大值（例如 5, 5, 5）

    # 使用 Open3D 的 crop 方法进行范围限制
    cur_scan = cur_scan.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound))
    # 累积点云
    
    
    if accumulated_count < ACCUM_LIMIT:
        rospy.loginfo(f'Accumulating scans... {accumulated_count}/{ACCUM_LIMIT}')
        accumulated_scans += cur_scan
        accumulated_count += 1
        return
    
    # 对点云进行体素化（类似 PCL 的 VoxelGrid）
    voxel_size = SCAN_VOXEL_SIZE  # 体素化尺寸（可以调整）
    accumulated_scans = accumulated_scans.voxel_down_sample(voxel_size)
    pc_prossed_msg=point_cloud_to_ros_msg(accumulated_scans)
    pc_prossed_msg.header.frame_id = 'body'
    pc_prossed_msg.header.stamp = rospy.Time.now()
    pub_pc_in_map.publish(pc_prossed_msg)
    if  first_match_flag:
        return

     
    
        # 可视化点云（如果需要）
    # o3d.visualization.draw_geometries([cur_scan])
    if not initial_pose_inited :
        return

    else:
        match_state= global_localization(init_odom,accumulated_scans)
        print("match_state",match_state)
        rospy.loginfo('global_localization !!!!!!')
        toc = time.time()
        rospy.loginfo('ALL Time: {}'.format(toc - tic))
        if match_state==True:
            first_match_flag=True
        
     # 清空计数器
    # accumulated_count = 0
    # accumulated_scans.clear()



def init_map():
    global start_value , init_map_flag,end_value,state_machine
    # 初始化全局地图
    rospy.logwarn(f'Waiting for global map......')
    rel_file_path = f'../maps_pcd/sim_2025.pcd'
    file_path = os.path.join(os.path.dirname(__file__), rel_file_path)
    file_path = os.path.abspath(file_path)  # 转为绝对路径
    initialize_global_map(file_path)
    init_map_flag=True
        
def state_callback( msg: Int8):
    global state_machine
    state_machine = msg.data
    print("self.state_machine:",state_machine)
if __name__ == '__main__':
    MAP_VOXEL_SIZE = 0.2
    SCAN_VOXEL_SIZE = 0.3

    # Global localization frequency (HZ)
    # FREQ_LOCALIZATION = 0.08

    # The threshold of global localization,
    # only those scan2map-matching with higher fitness than LOCALIZATION_TH will be taken
    LOCALIZATION_TH = 0.8

    # FOV(rad), modify this according to your LiDAR type
    FOV = 3.1415

    # The farthest distance(meters) within FOV
    FOV_FAR = 80

    rospy.init_node('fast_lio_localization')
    rospy.loginfo('Localization Node Inited...')

    # publisher
    pub_pc_in_map = rospy.Publisher('/cur_scan_in_map', PointCloud2, queue_size=1)
    pub_submap = rospy.Publisher('/submap_in_fov', PointCloud2, queue_size=1)
    pub_map_to_odom = rospy.Publisher('/localization_odom', Odometry, queue_size=1, latch=True)

    rospy.Subscriber('/cloud_registered_body', PointCloud2, cb_save_cur_scan, queue_size=1)
    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)
     # 订阅 /odom 或其他需要的位置数据
    # rospy.Subscriber('/fused_pose', Odometry, initial_odom_cbk, queue_size=1)
    # rospy.Subscriber("/start_value", Int8, start_value_cbk,queue_size=1)
    # rospy.Subscriber("/end_value", Int8, end_value_cbk,queue_size=1)
    rospy.Subscriber('/state_machine', Int8,  state_callback,queue_size=1)
    init_map()

   

    rospy.loginfo('')
    # rospy.loginfo('Initialize successfully!!!!!!')
    rospy.loginfo('')
    # 开始定期全局定位
    # threading.Thread(target=thread_localization)

    rospy.spin()
