#!/usr/bin/env python3
# coding=utf-8 

# 动态圆环变速规划
# 静态圆环轨迹规划

import rospy
import time
import math
from std_msgs.msg import Int8, Bool, Header
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Point
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import numpy as np
from ius_msgs.msg import Trajectory 
import open3d as o3d  #ycr
from collections import deque #ycr
from apriltag_robomaster import AprilTagPoseEstimator
import tf.transformations as tf_trans
from math import copysign


class StateMachine:
    def __init__(self):
        rospy.init_node("state_machine")
        self.real_fly = rospy.get_param('~real_fly','')
        self.uav_name = rospy.get_param('~uav_name', 'uav1')
        
        # 构建命名空间相关的主题
        self.mavros_ns = '/' + self.uav_name + '/mavros'

        # 状态变量
        self.odom = Odometry()
        self.current_velocity = np.zeros(3)
        self.mavros_state = State()
        self.state = 0
        self.pose = PoseStamped()
        self.last_request = rospy.Time.now()
        self.point_count = 0    # 计数点
        self.limit_xy = 0 # xy的接近阈值
        self.forest_points = []  # 树林预设点
        self.container_points = []  # 集装箱预设点
        self.maze_points = []  # 迷宫预设点
        self.static_circle_points = []  # 静环预设点
        self.dynamic_circle_points = [] # 动环预设点
        self.land_goal_points = [] # 降落预设点
        self.end = [] # ###

        self.present_done = False # 状态机标志位1
        self.present_done2 = False # 状态机标志位2
        self.present_done3 = False # 状态机标志位3
        self.present_done4 = False # 状态机标志位4
        self.present_done5 = False # 状态机标志位5
        self.present_done6 = False # 状态机标志位6
        self.stop_super = False # super避障开关

        
        # 静态圆环指定区域
        self.xmin1, self.xmax1 = 10.0, 14.0
        self.ymin1, self.ymax1 = -23.0, -17.0  # 注意负数
        self.zmin1, self.zmax1 = 2.0, 4.0
        self.static_points_buff = [] # 静环点缓冲区
        self.left_circle_points1 = [] # 静环左侧预设点1
        self.left_circle_points2 = [] # 静环左侧预设点2
        self.right_circle_points1 = [] # 静环右侧预设点1
        self.right_circle_points2 = [] # 静环右侧预设点2

        # 动态圆环指定区域
        self.xmin2, self.xmax2 = 3.0, 7.0
        self.ymin2, self.ymax2 = -24.0,-16.0
        self.zmin2, self.zmax2 = 2.0, 4.0  #ycr
        self.dynamic_points_buff = [] # 动环点缓冲区
        self.dynamic_point = [] # 动环圆心点

        # EKF初始参数
        self.kf_dt = 1/30  # 点云频率间隔
        dt = self.kf_dt
        # 状态向量: [x, y, vx, vy, ax, ay]
        self.kf_state = np.zeros((6, 1))  # 初始全零
        # 状态转移矩阵 F（离散加速度模型）
        self.kf_F = np.array([
            [1, 0, dt,  0, 0.5*dt*dt, 0],
            [0, 1, 0, dt, 0, 0.5*dt*dt],
            [0, 0, 1,  0, dt, 0],
            [0, 0, 0,  1, 0, dt],
            [0, 0, 0,  0, 1, 0],
            [0, 0, 0,  0, 0, 1]])
        # 观测矩阵 H（我们只观测 x 和 y）
        self.kf_H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0]])
        # 初始协方差 P
        self.kf_P = np.eye(6) * 0.1
        # 过程噪声 Q
        q_pos = 0.05
        q_vel = 0.1
        q_acc = 0.2
        self.kf_Q = np.diag([q_pos, q_pos, q_vel, q_vel, q_acc, q_acc])
        # 观测噪声 R（点云精度）
        self.kf_R = np.diag([0.02, 0.02])

        # 动态降落相关参数
        if self.real_fly:
            print('使用实机参数')
            self.apriltag = None
                # 相机内参
            self.camera_matrix = np.array([
               	[355.414453730001,  0,  332.240684951786],
	            [0,  357.072163960496,  256.093028154310],
	            [0, 0, 1]])
            self.tag_size = 0.147  # meters
        else:
            print('使用仿真参数')
            self.apriltag = None
                # 相机内参
            self.camera_matrix = np.array([
                [554.254691191187, 0, 320.5],
                [0, 554.254691191187, 240.5],
                [0, 0, 1]])
            self.tag_size = 0.20  # meters

        # 订阅器
        self.mavros_state_sub = rospy.Subscriber(self.mavros_ns + "/state", State, self.mavros_state_callback)
        self.odom_sub = rospy.Subscriber("~drone_odometry", Odometry, self.odom_callback)
        self.cloud_sub = rospy.Subscriber("~cloud_registered", PointCloud2, self.cloud_callback)  

        # 发布器
        self.goal_pub = rospy.Publisher('~goal', PoseStamped, queue_size=10)
        self.state_machine_state_pub = rospy.Publisher("~state_machine", Int8, queue_size=10)
        self.mpc_pub = rospy.Publisher('~mpc_trajectory', Trajectory, queue_size=1)
        self.stop_super_pub = rospy.Publisher('~stop_super', Bool, queue_size=1)
        self.pub_downsampled_static = rospy.Publisher("~filtered_static_pcd", PointCloud2, queue_size=1)
        self.pub_downsampled_dynamic = rospy.Publisher("~filtered_dynamic_pcd", PointCloud2, queue_size=1)

        # 服务客户端
        self.arming_client = rospy.ServiceProxy(self.mavros_ns + "/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy(self.mavros_ns + "/set_mode", SetMode)
    
        # 比赛预设参数
        self.load_params()  # 加载参数
        
        # 初始位置参数（可通过参数服务器配置）
        initial_x = rospy.get_param('~initial_x', 0.0)
        initial_y = rospy.get_param('~initial_y', 0.0)
        initial_z = rospy.get_param('~initial_z', 0.0)
        
        # 起飞点设置到初始位置上方1.2m处
        self.takeoff_point = [initial_x, initial_y, initial_z + 1.2, 0.0]
        rospy.loginfo(f"[{self.uav_name}] Takeoff point set to: {self.takeoff_point}")
        
        # 保存最近3帧点云的numpy数组   //#ycr
        self.cloud_deque1 = deque(maxlen=5)
        self.cloud_deque2 = deque(maxlen=5)  #ycr
        
    def odom_callback(self, odom_msg):
        self.odom = odom_msg
        self.posess = odom_msg.pose.pose.position
        self.orientation = odom_msg.pose.pose.orientation 
        alpha = 0.8  # 越大越信任旧值，抗抖动越强
        self.current_velocity = alpha * self.current_velocity + (1 - alpha) * np.array([
            odom_msg.twist.twist.linear.x,
            odom_msg.twist.twist.linear.y,
            odom_msg.twist.twist.linear.z])

    def mavros_state_callback(self, msg):
        self.mavros_state = msg
        # rospy.loginfo_throttle(1.0, f"Connected: {msg.connected}, Mode: {msg.mode}, Armed: {msg.armed}")
    
    # Open3D → PointCloud2
    def convert_o3d_to_ros(self, pcd, frame_id="map"):
        points = np.asarray(pcd.points)
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame_id
        return pc2.create_cloud_xyz32(header, points)

    def cloud_callback(self, msg):
        tic = time.time()  #ycr
        if self.state == 4 and not self.left_circle_points1 and self.present_done:
            arr_points = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                arr_points.append([point[0], point[1], point[2]])
            np_points = np.array(arr_points, dtype=np.float32)
            # 转成 Open3D 点云
            cur_scan = o3d.geometry.PointCloud()
            cur_scan.points = o3d.utility.Vector3dVector(np_points)
            # 对点云进行范围限制（类似 PCL 的 CropBox）
            min_bound = np.array([self.xmin1, self.ymin1, self.zmin1])  # 范围最小值
            max_bound = np.array([self.xmax1, self.ymax1, self.zmax1])  # 范围最大值
            # 使用 Open3D 的 crop 方法进行范围限制
            cur_two_circle_scan = cur_scan.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound))
            # ✅ 体素下采样
            voxel_size = 0.05
            cur_two_circle_scan_down = cur_two_circle_scan.voxel_down_sample(voxel_size=voxel_size)
            # Open3D 点云 -> numpy
            cur_two_circle_scan_points = np.asarray(cur_two_circle_scan_down.points)
            filtered = [(x, y, z) for x, y, z in cur_two_circle_scan_points] ##ycr
            print(f"[DEBUG] filtered points count: {len(filtered)}")
            print(f"[DEBUG] current accumulated frames: {len(self.static_points_buff)}")
            # # 发布静环点云
            # ros_msg = self.convert_o3d_to_ros(cur_two_circle_scan_down)
            # self.pub_downsampled_static.publish(ros_msg)

            if not filtered or len(filtered) < 10:
                print("[DEBUG] Not enough valid points, skipping frame")
                return

            if len(self.static_points_buff) == 50:
                all_points = []
                for frame_points in self.static_points_buff:
                    all_points.extend(frame_points)
                points_np = np.array(all_points, dtype=np.float32)
                circle1_pcd = o3d.geometry.PointCloud()
                circle1_pcd.points = o3d.utility.Vector3dVector(points_np)
                # ✅ 体素下采样
                voxel_size = 0.05
                circle1_pcd = circle1_pcd.voxel_down_sample(voxel_size=voxel_size)
                # points_np_down=np.asarray(circle1_pcd.points)
                # avg_x = np.mean(points_np_down[:, 0])
                # avg_y = np.mean(points_np_down[:, 1])
                points_np_down = np.asarray(circle1_pcd.points)
                # === 只选取 y 最小10% 和最大10% 的点 ===
                y_values = points_np_down[:, 1]
                lower_thresh = np.percentile(y_values, 10)
                upper_thresh = np.percentile(y_values, 90)
                # 选择 y <= lower_thresh 或 y >= upper_thresh 的点
                mask = (y_values <= lower_thresh) | (y_values >= upper_thresh)
                selected_points = points_np_down[mask]
                # 如果选中点数量太少，回退到全部点
                if len(selected_points) < 10:
                    rospy.logwarn("[WARN] 选中的边缘点太少，使用全部点计算平均")
                    selected_points = points_np_down
                # 计算平均坐标
                avg_x = np.mean(selected_points[:, 0])
                avg_y = np.mean(selected_points[:, 1])
                # 发布静环点云<-25 75->
                selected_points_o3d = o3d.geometry.PointCloud()
                selected_points_o3d.points = o3d.utility.Vector3dVector(selected_points)
                ros_msg = self.convert_o3d_to_ros(selected_points_o3d)
                self.pub_downsampled_static.publish(ros_msg)

                rospy.loginfo(f"[RESULT] 平均坐标: x={avg_x:.3f}, y={avg_y:.3f}")
                # l2 -- r1              x
                # |      |              |
                # l1    r2       y __ __|
                self.left_circle_points1 = [avg_x - 1.0, avg_y + 0.7, 2.5, 180.0]
                self.left_circle_points2 = [avg_x + 1.0, avg_y + 0.7, 2.5, 180.0]
                self.right_circle_points1 = [avg_x + 1.0, avg_y - 0.7, 2.5, 180.0]
                self.right_circle_points2 = [avg_x - 1.0, avg_y - 0.7, 2.5, 180.0]
            else:
                self.static_points_buff.append(filtered)
                
        if self.state == 5 and self.present_done:
            arr_points = []
            for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
                arr_points.append([point[0], point[1], point[2]])
            np_points = np.array(arr_points, dtype=np.float32)
            # 转成 Open3D 点云
            cur_scan = o3d.geometry.PointCloud()
            cur_scan.points = o3d.utility.Vector3dVector(np_points)
            # 对点云进行范围限制（类似 PCL 的 CropBox）
            min_bound = np.array([self.xmin2, self.ymin2, self.zmin2])  # 范围最小值
            max_bound = np.array([self.xmax2, self.ymax2, self.zmax2])  # 范围最大值
            # 使用 Open3D 的 crop 方法进行范围限制
            cur_circle_scan = cur_scan.crop(o3d.geometry.AxisAlignedBoundingBox(min_bound, max_bound))
           
            #放进 deque
            self.cloud_deque2.append(cur_circle_scan)
            if len(self.cloud_deque2) == 0:
                rospy.loginfo("[RESULT] 没有有效帧")
                return
            # ✅ 直接合并最近3帧（Open3D 点云加法）
            circle_scan_merged_pcd = o3d.geometry.PointCloud()
            for pcd in self.cloud_deque2:
                circle_scan_merged_pcd += pcd  # 直接加
            # ✅ 体素下采样
            voxel_size = 0.05
            circle_scan_down_pcd = circle_scan_merged_pcd.voxel_down_sample(voxel_size=voxel_size)
            # Open3D 点云 -> numpy
            np_points_cur_circle = np.asarray(circle_scan_down_pcd.points)
            filtered = [(x, y) for x, y, z in np_points_cur_circle]
            # 发布动环点云
            ros_msg = self.convert_o3d_to_ros(circle_scan_down_pcd)
            self.pub_downsampled_dynamic.publish(ros_msg)

            if not filtered:
                return
            
            points_np = np.array(filtered)
            avg_x = np.mean(points_np[:, 0])
            avg_y = np.mean(points_np[:, 1])
            # rospy.loginfo(f"[RESULT] 下采样后点数: {np_points_cur_circle.shape[0]}")
            # rospy.loginfo(f"[RESULT] 平均坐标: x={avg_x:.3f}, y={avg_y:.3f}")
            # z 是测量值
            z = np.array([[avg_x], [avg_y]])

            # 预测
            self.kf_state = self.kf_F @ self.kf_state
            self.kf_P = self.kf_F @ self.kf_P @ self.kf_F.T + self.kf_Q

            # 更新
            S = self.kf_H @ self.kf_P @ self.kf_H.T + self.kf_R
            K = self.kf_P @ self.kf_H.T @ np.linalg.inv(S)
            y_residual = z - self.kf_H @ self.kf_state
            self.kf_state = self.kf_state + K @ y_residual
            self.kf_P = (np.eye(6) - K @ self.kf_H) @ self.kf_P

            # 获取滤波后位置
            filtered_x = self.kf_state[0, 0]
            filtered_y = self.kf_state[1, 0]

            # 更新动态点位置（如偏移修正）
            self.dynamic_point = [filtered_x - 1.5, filtered_y, 3.0]
            # rospy.loginfo(f"[KF] 原始 y={avg_y:.3f} | 滤波 y={filtered_y:.3f} | vy={self.kf_state[3, 0]:.2f}")

        toc = time.time()
        # rospy.loginfo('cloud_callback Time: {}'.format(toc - tic))

    # 判断当前位置是否接近目标位置
    def is_close(self, odom: Odometry, pose: list, xy,z):
        if not pose:
            return False
        # print(abs(odom.pose.pose.position.x - pose[0]), abs(odom.pose.pose.position.y - pose[1]), abs(odom.pose.pose.position.z - pose[2]))
        if abs(odom.pose.pose.position.x - pose[0]) < xy and abs(odom.pose.pose.position.y - pose[1]) < xy and abs(odom.pose.pose.position.z - pose[2]) < z:
            return True
        return False
    
    # 设置目标点
    def go_to(self, pose: list):
        # 设置目标点位置
        self.pose.pose.position.x = pose[0]
        self.pose.pose.position.y = pose[1]
        self.pose.pose.position.z = pose[2]
        # 设置目标点的朝向
        angle_rad = math.radians(pose[3])
        roll, pitch, yaw = 0.0, 0.0, angle_rad # 假设roll和pitch是0
        q = quaternion_from_euler(roll, pitch, yaw)
        self.pose.pose.orientation.x = q[0]
        self.pose.pose.orientation.y = q[1]
        self.pose.pose.orientation.z = q[2]
        self.pose.pose.orientation.w = q[3]

        self.pose.header.stamp = rospy.Time.now()
        self.goal_pub.publish(self.pose)

    # 清空航点和标志位
    def clean(self):
        self.point_count = 0
        self.present_done = False # 状态机标志位1
        self.present_done2 = False # 状态机标志位2
        self.present_done3 = False # 状态机标志位3
        self.present_done4 = False # 状态机标志位4
        self.present_done5 = False # 状态机标志位5
        self.present_done6 = False # 状态机标志位6
        self.static_points_buff = [] # 静环点缓冲区
        self.dynamic_points_buff = [] # 动环点缓冲区

    def angle_diff(self, a, b):
        """返回 [-pi, pi] 范围内的角度差"""
        return math.atan2(math.sin(a - b), math.cos(a - b))

    def normalize_angle(self, angle):
        """将角度归一化到 [-pi, pi)"""
        return (angle + math.pi) % (2 * math.pi) - math.pi
    # def normalize_angle(self, angle):
    #     """将角度归一化到 [0, 2π)"""
    #     return angle % (2 * math.pi)

    # 计算插值轨迹
    def caculate_trajectory(self, start, end, yaw_fixed_rad, dt=0.05):
        start = np.array(start)
        end = np.array(end)
        delta = end - start

        # 当前 UAV 线速度（已在 odom_callback 中缓存）
        v_mag = np.linalg.norm(self.current_velocity)  
        v_min = np.array([0.1, 0.1, 0.1])  # 起始速度限制_
        scale = np.clip(v_mag / 1.0, 0.0, 1.0)  # 假设 1.0m/s 是“跑满速度”，超过1就固定为1
        print(f"速度比例 scale: {scale:.2f}")

        # 每个轴的最大速度
        if self.state == 5:
            v_max_limit = np.array([1.2, 0.8, 0.4])  # 动环参数
        elif self.state == 6:
            v_max_limit = np.array([0.8, 0.8, 0.6])  # 降落参数

        v_max = v_min + scale * (v_max_limit - v_min)
        # 每个轴的插值时间
        time_needed = np.where(np.abs(delta) < 1e-6, 0.0, np.abs(delta) / v_max)

        # === 插值 yaw ===
        q = self.orientation
        roll, pitch, current_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # --- 平滑处理四元数跳变 ---
        if hasattr(self, 'last_yaw'):
            if abs(current_yaw - self.last_yaw) > np.pi:
                if self.last_yaw > current_yaw:
                    current_yaw += 2 * np.pi
                else:
                    current_yaw -= 2 * np.pi
        self.last_yaw = current_yaw  # 更新
        yaw_delta = self.angle_diff(yaw_fixed_rad, current_yaw)
  
        # 方案B：加阈值避免极小差值引起跳变
        if abs(yaw_delta) < 1e-3:
            yaw_delta = 0.0
            yaw_time = 0.0
        else:
            yaw_rate = np.deg2rad(20.0) # 固定角速度（rad/s）
            yaw_time = abs(yaw_delta) / yaw_rate

        # === 轨迹插值时间 = max(位置插值时间, yaw 插值时间) ===
        total_time = max(np.max(time_needed), yaw_time)
        num_steps = int(np.ceil(total_time / dt)) + 1
        t_array = np.linspace(0, total_time, num_steps)

        # 逐轴插值
        trajectory = []
        for t in t_array:
            coord = []
            for i in range(3):
                if time_needed[i] == 0:
                    coord.append(end[i])
                else:
                    ratio = min(t / time_needed[i], 1.0)
                    coord.append(start[i] + ratio * delta[i])
            trajectory.append(Point(*coord))

       # 方案A：比例插值，避免用符号加角速度累积
        yaw_list = []
        for t in t_array:
            if yaw_time > 1e-6:
                ratio = min(t / yaw_time, 1.0)
                yaw = self.normalize_angle(current_yaw + yaw_delta * ratio)
            else:
                yaw = self.normalize_angle(yaw_fixed_rad)
            yaw_list.append(yaw)
        
        # 方案C：打印调试信息
        print(f"[Yaw 插值] current: {np.rad2deg(current_yaw):.2f}°, target: {np.rad2deg(yaw_fixed_rad):.2f}°, delta: {np.rad2deg(yaw_delta):.2f}°")
        print("Yaw 插值序列 (deg):", [round(np.rad2deg(y), 1) for y in yaw_list])
        print(f"[Yaw 插值] 当前 yaw（rad）: {current_yaw:.3f}，目标 yaw（rad）: {yaw_fixed_rad:.3f}，差值 yaw_delta（rad）: {yaw_delta:.3f}")
        print("Yaw 插值序列 (rad):", [round(y, 1) for y in yaw_list])
        time_list = [float(t) for t in t_array]
        return trajectory, yaw_list, time_list
    
    # 发布mpc参考轨迹
    def pub_mpc_traj(self, traj_id, pos, yaw, time_pts):
        msg = Trajectory()
        msg.header.stamp = rospy.Time.now()
        msg.traj_id = traj_id
        msg.pos = pos
        msg.yaw = yaw
        msg.time = time_pts
        self.mpc_pub.publish(msg)
    
    # 发布super避障开关信号
    def super_change(self, super_stop):
        self.stop_super = super_stop
        msg = Bool()
        msg.data = self.stop_super
        self.stop_super_pub.publish(msg)
        # rospy.loginfo(f"发布super开关信号: {self.stop_super}")
    
    def run(self):
        # 发布当前状态
        state_now = Int8()
        state_now.data = self.state
        self.state_machine_state_pub.publish(state_now)

        # 状态0：等待进入OFFBOARD并解锁
        if self.state == 0:
            # '''切换到穿越圆环1'''
            if self.is_close(self.odom, self.takeoff_point, 0.5, 0.3):
                print("已到达起飞点，切换状态1")
                self.state = 1 #########
                return
            # '''切入offboard模式'''
            if self.mavros_state.mode != "OFFBOARD" and rospy.Time.now() - self.last_request > rospy.Duration(1.0):
                rospy.loginfo("Setting mode: OFFBOARD")
                # self.set_mode_client(0, "OFFBOARD")
                self.last_request = rospy.Time.now()
            # '''解锁电机'''
            elif not self.mavros_state.armed and rospy.Time.now() - self.last_request > rospy.Duration(1.0):
                rospy.loginfo("Arming drone")
                self.arming_client(True)
                self.last_request = rospy.Time.now()
            # '''起飞到指定高度'''
            elif self.mavros_state.mode == "OFFBOARD" and self.mavros_state.armed:
                self.go_to(self.takeoff_point)
                print(f"前往起飞点: {self.takeoff_point}")
            return
        
        # 通过树林
        elif self.state == 1:
            if self.is_close(self.odom, self.forest_points[self.point_count],1.5,0.5):
                if self.point_count == len(self.forest_points) - 1:
                    self.state = 2 #####
                    self.clean()
                    #print(self.point_count)
                    return
                self.state = 1
                self.point_count+=1
                return
            else:
                self.go_to(self.forest_points[self.point_count])
                print(f'飞往树林第{self.point_count+1}个点: {self.forest_points[self.point_count]}')
                return
            
        # 通过集装箱
        elif self.state == 2:
            # 改变xy接近阈值
            self.limit_xy = 0.5 if self.point_count == 0 else 1.0
            if self.is_close(self.odom, self.container_points[self.point_count],self.limit_xy,0.5):
                if self.point_count == len(self.container_points) - 1:
                    self.state = 3
                    self.clean()
                    return
                self.state = 2
                self.point_count += 1
                return
            else:
                self.go_to(self.container_points[self.point_count])
                print(f'飞往集装箱第{self.point_count+1}个点:{self.container_points[self.point_count]}')
                return
            
        # 通过迷宫
        elif self.state == 3:
            # 改变xy接近阈值
            self.limit_xy = 0.5 if self.point_count == 0 else 1.0
            if self.is_close(self.odom, self.maze_points[self.point_count],self.limit_xy,0.5):
                if self.point_count == len(self.maze_points) - 1:
                    self.state = 4
                    self.clean()
                    print("已通过迷宫，切换到静环")
                    return
                self.state = 3
                self.point_count += 1
                return
            else:
                self.go_to(self.maze_points[self.point_count])
                print(f'飞往迷宫第{self.point_count+1}个点:{self.maze_points[self.point_count]}')
                return
        
        # 通过静环
        elif self.state == 4:
            if not self.present_done:
                self.go_to(self.static_circle_points[self.point_count])
                print(f'飞往静环第{self.point_count+1}个点:{self.static_circle_points[self.point_count]}')
                time.sleep(1.0)
                self.present_done = True ##ycr
                return
            if not self.present_done2 and self.is_close(self.odom, self.static_circle_points[self.point_count],0.5,0.5):
                if not self.left_circle_points1:
                    return
                self.go_to(self.left_circle_points1)
                print(f'飞往静环左侧1点: {self.left_circle_points1}')
                self.present_done2 = True
                return
            if not self.present_done3 and self.is_close(self.odom, self.left_circle_points1,0.3, 0.3):
                self.go_to(self.left_circle_points2)
                print(f'飞往静环左侧2点: {self.left_circle_points2}')
                self.present_done3 = True
                return
            if not self.present_done4 and self.is_close(self.odom, self.left_circle_points2,0.5, 0.5):
                self.go_to(self.right_circle_points1)
                print(f'飞往静环右侧1点: {self.right_circle_points1}')
                self.present_done4 = True
                return
            if not self.present_done5 and self.is_close(self.odom, self.right_circle_points1,0.5, 0.5):
                self.go_to(self.right_circle_points2)
                print(f'飞往静环右侧2点: {self.right_circle_points2}')
                self.present_done5 = True
                return
            if not self.present_done6 and self.is_close(self.odom, self.right_circle_points2,0.5,0.5):
                self.state = 5  # 切换到状态5，表示完成静环任务
                print("已完成静环任务，切换到动态圆环")
                self.present_done6 = True
                self.clean()
                return
            return

        # 通过动环
        elif self.state == 5:
            if not self.present_done:
                self.go_to(self.dynamic_circle_points[self.point_count])
                print(f'飞往动环第{self.point_count+1}个点:{self.dynamic_circle_points[self.point_count]}')
                self.present_done = True
                return
            if not self.present_done2 and self.is_close(self.odom, self.dynamic_circle_points[self.point_count],0.2,0.2):
                self.present_done2 = True
                return
            if not self.present_done3 and self.dynamic_point and self.present_done2:
                self.super_change(True)  # 关闭super
                if not self.is_close(self.odom, self.dynamic_point,1.9,0.5):
                    start =  [self.posess.x, self.posess.y, self.posess.z]
                    self.end = self.dynamic_point
                    pos, yaw, time_pts = self.caculate_trajectory(start, self.end, math.radians(180.0),dt=0.05)
                    self.pub_mpc_traj(traj_id=1, pos=pos, yaw=yaw, time_pts=time_pts)
                if self.is_close(self.odom, self.end ,0.5,0.2):
                    self.state = 6  # 切换到状态6，表示完成动环任务
                    print("已完成动环任务，切换到动态降落")
                    self.present_done3 = True
                    self.super_change(False)  # 开启super
                    self.clean()
                return
            return
        
        # 前往降落区域
        elif self.state == 6:
            if not self.present_done:
                self.go_to(self.land_goal_points[self.point_count])
                print(f'飞往动环第{self.point_count+1}个点:{self.land_goal_points[self.point_count]}')
                self.present_done = True
                self.apriltag = AprilTagPoseEstimator(self.camera_matrix, self.tag_size, self.real_fly, is_debug=True)
                return
            if not self.present_done2:
                if self.is_close(self.odom, self.land_goal_points[self.point_count], 0.25, 0.25):
                    self.super_change(True)  # 关闭super
                    print("关闭super")
                    self.present_done2 = True
                return
            # 发布降落轨迹
            if self.apriltag.can_see and not self.present_done3:
                # 构造齐次变换矩阵（Body 到 World）
                T_world = tf_trans.concatenate_matrices(
                    tf_trans.translation_matrix([self.posess.x, self.posess.y, self.posess.z]),
                    tf_trans.quaternion_matrix([self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w]))
                # AprilTag 在 Body 坐标系下的位置 → 转换到 World 坐标系
                pos_body = self.apriltag.ros_pose.pose.position
                pt_body = np.array([[pos_body.x], [pos_body.y], [pos_body.z], [1.0]])
                pt_world = np.matmul(T_world, pt_body)
                rospy.loginfo(f"World Position: {pt_world[:3].flatten()}")
                # 发布插值轨迹
                start =  [self.posess.x, self.posess.y, self.posess.z]
                end = pt_world[:3].flatten() + np.array([0, 0, 0.1]) # 降落点z加上0.1m
                pos, yaw, time_pts = self.caculate_trajectory(start, end, math.radians(180.0),dt=0.05)
                self.pub_mpc_traj(traj_id=1, pos=pos, yaw=yaw, time_pts=time_pts)
             # 如果高度足够低,切换状态7
            if self.posess.z < 0.65:
                self.present_done3 = True
                self.state = 7
                rospy.loginfo(f"直接降落")
        
        elif self.state == 7:
            # 比赛结束
            rospy.loginfo(f"比赛结束")
            return

    def load_params(self):
        # 读取树林预设点
        forest_points = rospy.get_param("~forest")
        for i, point in enumerate(forest_points):
            self.forest_points.append(tuple(point))
        rospy.loginfo(f"Loaded forest points: {self.forest_points}")

        # 读取集装箱预设点
        container_points = rospy.get_param("~container")
        for i, point in enumerate(container_points):
            self.container_points.append(tuple(point))
        rospy.loginfo(f"Loaded container points: {self.container_points}")

        # 读取迷宫预设点
        maze_points = rospy.get_param("~maze")
        for i, point in enumerate(maze_points):
            self.maze_points.append(tuple(point))
        rospy.loginfo(f"Loaded maze points: {self.maze_points}")

        # 读取静环预设点
        static_circle_points = rospy.get_param("~static_circle")
        for i, point in enumerate(static_circle_points):
            self.static_circle_points.append(tuple(point))
        rospy.loginfo(f"Loaded static circle points: {self.static_circle_points}")

        # 读取动环预设点
        dynamic_circle_points = rospy.get_param("~dynamic_circle")
        for i, point in enumerate(dynamic_circle_points):
            self.dynamic_circle_points.append(tuple(point))
        rospy.loginfo(f"Loaded dynamic circle points: {self.dynamic_circle_points}")

        # 读取降落预设点
        land_goal_points = rospy.get_param("~land_goal")
        for i, point in enumerate(land_goal_points):
            self.land_goal_points.append(tuple(point))
        rospy.loginfo(f"Loaded land goal points: {self.land_goal_points}")
        

if __name__ == "__main__":
    state_machine = StateMachine()
    rate = rospy.Rate(30)  
    while not rospy.is_shutdown():
        state_machine.run()
        rate.sleep()