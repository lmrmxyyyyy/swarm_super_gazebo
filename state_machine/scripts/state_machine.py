#!/usr/bin/env python3
# coding=utf-8

import rospy
import time
import math
import numpy as np
from std_msgs.msg import Int8, Bool
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Point
from ius_msgs.msg import Trajectory 
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from scipy.interpolate import CubicSpline
from apriltag_robomaster import AprilTagPoseEstimator
import tf.transformations as tf_trans


class StateMachine:
    def __init__(self):
        rospy.init_node("state_machine")

        # 状态变量
        self.odom = Odometry()
        self.posess = None
        self.orientation = None
        self.mavros_state = State()
        self.state = 0
        self.last_request = rospy.Time.now()
        self.point =  []
        self.point_count = 0
        self.pose = PoseStamped()
       
        # 比赛预设参数
        self.load_params()  # 加载参数
        self.enable_detection = False  # yolo检测开关
        self.stop_super = False # super避障开关
        self.detect_through_time = False # yolo检测穿越时间开关
        self.allow_through = False  # 允许穿越旋转框标志

        self.preset_point = []  # 预先设置的环前目标点1
        self.set_preset_done = False
        self.preset_point2 = []  # 预先设置的环前目标点2
        self.set_preset_done2 = False 
        self.preset_point3 = []  # 预先设置的环前目标点3
        self.set_preset_done3 = False 
        self.preset_point4 = []  # 预先设置的环前目标点4
        self.set_preset_done4 = False 
        self.preset_point5 = []  # 预先设置的环前目标点5
        self.set_preset_done5 = False 
        self.align_point = [] # 与圆环中心点对齐的目标点
        self.set_align_done = False
        self.through_point = [] # 穿越圆环的目标点
        self.set_through_done = False

        self.circle_point = [] # 识别得到的圆环中心点  
        self.circle_point_buffer = [] # 圆环中心点缓存
        self.rotate_points_buffer = [] # 旋转框中心点缓存
        self.rotate_point = [] # 旋转框中心点
        self.dynamic_point = [] # 动环中心点
        self.flygooo = False  # 允许穿越动态圆环标志    
        
        # 预设的航点                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
        self.takeoff_point = [0.0, 0.0, 1.4, 0.0]  # 起飞点
        self.offset_1 = [0.5, 0.5, 0.0, 0.0]  # 前往圆环1前方的偏移量(角度是绝对的，不是相对的)
        self.offset_2 = [0.5, 0.5, 0.0, 0.0]  # 前往圆环2前方的偏移量(角度是绝对的，不是相对的)
        self.offset_31 = [2.5, 0.0, 1.8, 0.0]  # 前往圆环3前方的偏移量(角度是绝对的，不是相对的)
        self.offset_32 = [3.5, 0.0, 0.0, 0.0]  # 前往圆环3前方的偏移量(角度是绝对的，不是相对的)
        self.offset_41 = [3.0, -1.0, 0.3, 0.0]  # 前往圆环4前方的偏移量(角度是绝对的，不是相对的)
        self.offset_42 = [2.0, 0.0, 0.0, 90.0]  # 前往圆环4前方的偏移量(角度是绝对的，不是相对的)
        self.offset_5 =  [-1.5, 2.0, 0.0, 180.0]  # 前往圆环5前方的偏移量(角度是绝对的，不是相对的)
        self.offset_61 = [-3.5, 1.3, 0.0, 180.0]  # 前往圆环6前方的偏移量(角度是绝对的，不是相对的)
        # self.offset_61 = [0.0, -1.0, 0.0, 270.0]  # 前往圆环6前方的偏移量(角度是绝对的，不是相对的)  # 测试模式需要
        self.offset_62 = [-0.8, 0.0, -1.7, 180.0]  # 前往圆环6前方的偏移量(角度是绝对的，不是相对的)
        self.offset_63 = [-3.5, 0.0, 0.0, 270.0]  # 前往圆环6前方的偏移量(角度是绝对的，不是相对的)
        self.offset_71 = [1.3, 0.0, 0.0, 0.0]  # 前往圆环7前方的偏移量(角度是绝对的，不是相对的)
        self.offset_72 = [3.5, 0.0, 0.0, 0.0]  # 前往圆环7前方的偏移量(角度是绝对的，不是相对的)
        self.offset_73 = [0.0, -1.8, 0.0, 0.0]  # 前往圆环7前方的偏移量(角度是绝对的，不是相对的)
        self.offset_74 = [2.2, 0.0, 0.0, 0.0]  # 前往圆环7前方的偏移量(角度是绝对的，不是相对的)
        self.offset_75 = [2.5, 0.0, 0.0, 0.0]  # 前往圆环7前方的偏移量(角度是绝对的，不是相对的)
        self.offset_81 = [4.5, 0.0, 0.0, 90.0]  # 前往圆环8前方的偏移量(角度是绝对的，不是相对的)
        self.offset_82 = [0.0, 3.5, 0.0, 180.0]  # 前往圆环8前方的偏移量(角度是绝对的，不是相对的)
        self.offset_83 = [-1.0, 2.5, 0.0, 180.0]  # 前往圆环8前方的偏移量(角度是绝对的，不是相对的)
        self.offset_91 = [-7.0, 0.0, 0.0, 180.0]  # 前往圆环9前方的偏移量(角度是绝对的，不是相对的)
        self.offset_92 = [-0.6, 0.0, 0.0, 180.0]  # 前往圆环9前方的偏移量(角度是绝对的，不是相对的)
        self.offset_121 = [-3.0, -3.5, 0.0, 270.0]  # 前往降落平台偏移量(角度是绝对的，不是相对的)
        self.offset_122 = [-5.3, 0.0, 0.0, 270.0]  # 前往降落平台偏移量(角度是绝对的，不是相对的)
        self.offset_123 = [0.0, 4.0, 1.0, 0.0]  # 前往降落平台偏移量(角度是绝对的，不是相对的)
        # 从摆动环上方越过去
        # self.offset_121 = [0.0, 0.0, 1.5, 180.0]  # 前往降落平台偏移量(角度是绝对的，不是相对的)
        # self.offset_122 = [-7.5, 0.0, 0.0, 180.0]  # 前往降落平台偏移量(角度是绝对的，不是相对的)
        # self.offset_123 = [0.0, 0.0, -1.0, 180.0]  # 前往降落平台偏移量(角度是绝对的，不是相对的)
        

        # 与动态障碍物相关参数
        self.bridge = CvBridge()
        self.recent_avg_depths = []
        self.window_size = 10  # 连续稳定帧数要求
        self.stable_threshold = 0.3  # 深度变化范围小于此值视为稳定
        self.passable = False # 动障碍穿越标志

        # 参数读取
        odom_topic = rospy.get_param('~odom_topic', '/iris_0/mavros/local_position/odom')
        mavros_state_topic = rospy.get_param('~mavros_state_topic', '/iris_0/mavros/state')
        arming_client_service = rospy.get_param('~arming_client_service', '/iris_0/mavros/cmd/arming')
        set_mode_service = rospy.get_param('~set_mode_service', '/iris_0/mavros/set_mode')
        depth_image_topic = rospy.get_param('~depth_image_topic', '/iris_0/realsense/depth_camera/depth/image_raw')
        self.pass_dynamic_way = rospy.get_param('~pass_dynamic_way', False)  # 穿越动障碍方式
        self.test_mode = rospy.get_param('~test_mode', False)  # 是否为测试模式（直接到达降落处）
        self.real_fly = rospy.get_param('~real_fly','')
        
        if self.test_mode:
            rospy.loginfo("测试模式，直接到达动态圆环处")
        else:
            rospy.loginfo("比赛模式")

        if self.pass_dynamic_way:
            rospy.loginfo("使用深度图检测动障碍")
        else:
            rospy.loginfo("使用YOLO检测动障碍")

        # AprilTag检测
        if self.real_fly:
            print('使用实机参数')
            self.apriltag = None
            # 相机内参
            self.camera_matrix = np.array([
            [360.129106870628, 0, 321.244039022682],
            [0, 361.139325471772, 267.579746144584],
            [0, 0, 1]])
            self.tag_size = 0.135  # meters
        else:
            print('使用仿真参数')
            self.apriltag = None
            self.camera_matrix = np.array([
            [554.254691191187, 0, 320.5],
            [0, 554.254691191187, 240.5],
            [0, 0, 1]])
            self.tag_size = 0.2  # meters

        # 发布器
        self.goal_pub = rospy.Publisher('/goal', PoseStamped, queue_size=10)
        self.state_machine_state_pub = rospy.Publisher("/state_machine", Int8, queue_size=10)
        self.detect_enable_pub = rospy.Publisher('/yolo/detect_enable', Bool, queue_size=1)
        self.stop_super_pub = rospy.Publisher('/stop_super', Bool, queue_size=1)
        self.detect_through_time_pub = rospy.Publisher('/detect_through_time', Bool, queue_size=1)
        self.mpc_pub = rospy.Publisher('/mpc_trajectory', Trajectory, queue_size=1)

        # 服务客户端
        self.arming_client = rospy.ServiceProxy(arming_client_service, CommandBool)
        self.set_mode_client = rospy.ServiceProxy(set_mode_service, SetMode)

        # 订阅器
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        self.mavros_state_sub = rospy.Subscriber(mavros_state_topic, State, self.mavros_state_callback)
        self.circle_center_sub = rospy.Subscriber('/circle_center', Point, self.circle_center_callback)
        self.allow_through_sub = rospy.Subscriber('/allow_through', Bool, self.allow_through_callback)
        self.depth_image_sub = rospy.Subscriber(depth_image_topic, Image, self.depth_image_callback)
 
    def odom_callback(self, odom_msg):
        self.odom = odom_msg
        self.posess = odom_msg.pose.pose.position
        self.orientation = odom_msg.pose.pose.orientation  

    def mavros_state_callback(self, msg):
        self.mavros_state = msg
        # rospy.loginfo_throttle(1.0, f"Connected: {msg.connected}, Mode: {msg.mode}, Armed: {msg.armed}")

    def allow_through_callback(self, msg):
        self.allow_through = msg.data
        if self.state == 7:
            self.passable = self.allow_through

    def circle_center_callback(self, msg):
        # === 处理旋转框状态 ===
        if self.state == 6:
            self.rotate_points_buffer.append([msg.x, msg.y, msg.z])

            # 限制缓冲区最多10个点
            if len(self.rotate_points_buffer) > 10:
                self.rotate_points_buffer.pop(0)
            
            if len(self.rotate_points_buffer) == 10:
                points = np.array(self.rotate_points_buffer)
                median = np.median(points, axis=0)
                dists = np.linalg.norm(points - median, axis=1)
                threshold = 0.5
                filtered_points = points[dists < threshold]

                if len(filtered_points) >= 3:
                    avg_point = np.mean(filtered_points, axis=0)
                    self.rotate_point = avg_point.tolist()
                    rospy.loginfo(f"旋转框中心点: {self.rotate_point}")
                else:
                    rospy.logwarn("旋转框中心点不足3个，无法计算平均值")
            return

        # === 非旋转状态，处理圆环 ===
        self.rotate_points_buffer.clear()  # 离开旋转状态清空缓冲

        # 储存圆环中心点
        self.circle_point_buffer.append([msg.x, msg.y, msg.z])

        # 限制缓冲区最多75个点
        if len(self.circle_point_buffer) > 45:
            self.circle_point_buffer.pop(0)

        # === 平滑处理，使用最近10帧计算中值与均值 ===
        if len(self.circle_point_buffer) >= 10:
            recent_points = np.array(self.circle_point_buffer[-10:])
            median = np.median(recent_points, axis=0)
            dists = np.linalg.norm(recent_points - median, axis=1)
            threshold = 0.5
            filtered_points = recent_points[dists < threshold]

            if len(filtered_points) >= 3:
                avg_point = np.mean(filtered_points, axis=0)
                self.circle_point = avg_point.tolist()
                # rospy.loginfo(f"圆环中心点: {self.circle_point}")
        
        # === 计算动态圆环最左边点 ===
        if self.state in [10, 11] and not self.allow_through and len(self.circle_point_buffer) == 45:
            all_points = np.array(self.circle_point_buffer)
            min_y_idx = np.argmin(all_points[:, 1])  # 找到 y 坐标最小的点的索引
            leftmost_point = all_points[min_y_idx]
            self.dynamic_point = [leftmost_point[0], leftmost_point[1], leftmost_point[2]]
            rospy.loginfo(f"检测到最左边点: ({self.dynamic_point[0]:.2f}, {self.dynamic_point[1]:.2f}, {self.dynamic_point[2]:.2f}) → dynamic_point 设置完毕")

        # # === 判断圆环是否静止 ===
        # if self.state in [10, 11] and not self.allow_through and len(self.circle_point_buffer) == 30:
        #     all_points = np.array(self.circle_point_buffer)
        #     max_dist = 0.0
        #     for i in range(len(all_points)):
        #         for j in range(i + 1, len(all_points)):
        #             dist = np.linalg.norm(all_points[i] - all_points[j])
        #             if dist > max_dist:
        #                 max_dist = dist

        #     static_threshold = 0.075  # 根据实际场景可调
        #     if max_dist < static_threshold:
        #         self.allow_through = True
        #         rospy.loginfo("圆环已静止（最大距离 %.3f），允许通过" % max_dist)
        #     else:
        #         self.allow_through = False
        #         rospy.loginfo("圆环未静止（最大距离 %.3f），不允许通过" % max_dist)


    
    # 处理动态障碍物及动环
    def depth_image_callback(self, msg):
        if not hasattr(self, "frame_count"):
            self.frame_count = 0  # 初始化帧计数器
        self.frame_count += 1
        if self.frame_count % 2 != 0:
            return  # 每两帧处理一帧，跳过奇数帧
        
        if self.state == 7 and not self.passable and self.pass_dynamic_way:
            # 将ROS图像消息转换为OpenCV格式（假设是32FC1类型）
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")

            # 裁剪图像，保留高度480，居中裁剪宽度为320
            center_x = 640 // 2  # 320
            crop_w = 320
            x1 = center_x - crop_w // 2  # 320 - 160 = 160
            x2 = center_x + crop_w // 2  # 320 + 160 = 480

            cropped_depth = depth_image[:, x1:x2]  # 裁剪所有行（高），列范围为160:480

            # 筛选出大于0.1米的像素，并将所有超过6米的像素限制为6米（视为障碍物外），用于动态障碍物判断
            valid_pixels = np.clip(cropped_depth[(cropped_depth > 0.1)], 0, 6.0)
            if valid_pixels.size == 0:
                rospy.logwarn("No valid depth pixels found")
                return
            avg_depth = np.mean(valid_pixels)
            self.recent_avg_depths.append(avg_depth)

            # 限制滑动窗口大小
            if len(self.recent_avg_depths) > self.window_size:
                self.recent_avg_depths.pop(0)

            # 判断是否稳定变大（代表障碍物离开）
            if len(self.recent_avg_depths) == self.window_size:
                std_dev = np.std(self.recent_avg_depths[-3:])  # 只考虑最近3帧标准差
                depth_trend = self.recent_avg_depths[-1] - self.recent_avg_depths[0]
                print(self.recent_avg_depths)
                if std_dev < self.stable_threshold and depth_trend > 0.40:
                    self.passable = True
                    rospy.loginfo("✅ 动态障碍物已离开，可通过")
                else:
                    self.passable = False
                    rospy.loginfo("⛔ 动态障碍物仍在视野中")

        elif self.state in [10, 11] and not self.passable and self.flygooo:
            # 将ROS图像消息转换为OpenCV格式（假设是32FC1类型）
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            # 裁剪图像，保留高度480，靠右1/4
            x1,x2 = 480,640
            cropped_depth = depth_image[:, x1:x2]  # 裁剪所有行（高），列范围为480：640
            # 筛选出大于0.1米,且小于3米的像素
            valid_pixels = np.clip(cropped_depth[(cropped_depth > 0.1)], 0, 3.0)
            if valid_pixels.size == 0:
                rospy.logwarn("No valid depth pixels found")
                return
            avg_depth = np.mean(valid_pixels)
            self.recent_avg_depths.append(avg_depth)

            # 限制滑动窗口大小
            if len(self.recent_avg_depths) > self.window_size:
                self.recent_avg_depths.pop(0)
            
            if len(self.recent_avg_depths) >= 3:
                if self.recent_avg_depths[-1] < self.recent_avg_depths[-3] and not self.passable:
                    rospy.loginfo("Latest average depth is smaller than that of three frames ago")
                    self.passable = True

            
            ######


        else:
            return

    # 生成参考轨迹
    def generate_reference_trajectory(self, start, mid, end, t1, t2, yaw_fixed, dt=0.1):
        total_time = t1 + t2
        steps = int(np.ceil(total_time / dt)) + 1
        t = np.linspace(0, total_time, steps)

        # 控制点和对应时间
        ctrl_pts = np.array([start, mid, end])  # shape: (3, 3)
        ctrl_t = [0, t1, t1 + t2]

         # 添加边界速度为 0 的约束
        spl_x = CubicSpline(ctrl_t, ctrl_pts[:, 0], bc_type=((1, 0.0), (1, 0.0)))
        spl_y = CubicSpline(ctrl_t, ctrl_pts[:, 1], bc_type=((1, 0.0), (1, 0.0)))
        spl_z = CubicSpline(ctrl_t, ctrl_pts[:, 2], bc_type=((1, 0.0), (1, 0.0)))

        # 计算轨迹点
        x = spl_x(t)
        y = spl_y(t)
        z = spl_z(t)
        yaw = np.full_like(t, yaw_fixed)

        pos = [Point(x[i], y[i], z[i]) for i in range(len(t))]
        yaw_list = [float(yaw[i]) for i in range(len(t))]
        time_list = [float(t[i]) for i in range(len(t))]
        return pos, yaw_list, time_list
    
    # 计算降落轨迹
    def caculate_land_trajectory(self, start, end, yaw_fixed_rad, dt=0.05):
        start = np.array(start)
        end = np.array(end)
        delta = end - start

        # 每个轴的最大速度
        v_max = np.array([0.3, 0.3, 0.2])  # x, y, z 方向最大速度

        # 每个轴的插值时间
        time_needed = np.where(np.abs(delta) < 1e-6, 0.0, np.abs(delta) / v_max)

        # 总插值时间是所有轴中最大的
        total_time = np.max(time_needed)
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

        # 固定的 yaw 值和时间戳列表
        yaw_list = [float(yaw_fixed_rad)] * len(trajectory)
        time_list = [float(t) for t in t_array]

        return trajectory, yaw_list, time_list
    
    def generate_dynamic_traj(self, start_point, end_point, time=1.0, dt=0.05):
        """
        生成一段起点到终点的均匀直线轨迹。

        Args:
            start_point: list [x, y, z]
            end_point: list [x, y, z, yaw] 最后一位是 yaw 角（度）
            time: 总轨迹时间（秒）
            dt: 时间间隔（秒）

        Returns:
            pos: list of geometry_msgs.msg.Point
            yaw: list of float
            time_pts: list of float
        """
        steps = int(time / dt)
        start = np.array(start_point)
        end = np.array(end_point[:3])  # 仅取 x, y, z
        delta = end - start

        pos = []
        for i in range(steps):
            ratio = i / (steps - 1)
            point = start + ratio * delta
            pos.append(Point(*point))

        yaw = [end_point[3]] * steps  # 固定 yaw（角度）
        time_pts = [i * dt for i in range(steps)]

        return pos, yaw, time_pts
    
    # def generate_reference_trajectory(self,start, mid, end, t1, t2, yaw_fixed, dt=0.1):
    #     """
    #     生成供 MPC 使用的参考轨迹
    #     :param start: 起点坐标 (x, y, z)
    #     :param mid: 中点坐标 (x, y, z)
    #     :param end: 终点坐标 (x, y, z)
    #     :param t1: 起点到中点所需时间（秒）
    #     :param t2: 中点到终点所需时间（秒）
    #     :param yaw_fixed: 固定的 yaw 值（单位：弧度）
    #     :param dt: 采样时间间隔（默认 0.1s）
    #     :return: 包含每个时刻的轨迹点列表，每个点为 [pos, yaw, time]
    #     """
    #     # 生成时间序列
    #     t1_steps = int(np.ceil(t1 / dt))
    #     t2_steps = int(np.ceil(t2 / dt))
        
    #     # 插值函数
    #     def interpolate(p0, p1, steps):
    #         return [np.linspace(p0[i], p1[i], steps, endpoint=False) for i in range(3)]
        
    #     # 起点到中点插值
    #     x1, y1, z1 = interpolate(start, mid, t1_steps)
    #     time1 = np.linspace(0, t1, t1_steps, endpoint=False)

    #     # 中点到终点插值
    #     x2, y2, z2 = interpolate(mid, end, t2_steps + 1)  # +1 保证最终点包含
    #     time2 = np.linspace(t1, t1 + t2, t2_steps + 1)

    #     # 合并两个阶段
    #     x = np.concatenate([x1, x2])
    #     y = np.concatenate([y1, y2])
    #     z = np.concatenate([z1, z2])
    #     t = np.concatenate([time1, time2])
    #     yaw = np.full_like(t, yaw_fixed)

    #     # 构造 ROS 消息格式
    #     pos = [Point(x[i], y[i], z[i]) for i in range(len(t))]
    #     yaw_list = [float(yaw[i]) for i in range(len(t))]
    #     time_list = [float(t[i]) for i in range(len(t))]
    #     return pos, yaw_list, time_list
    
    # 发布mpc参考轨迹
    def pub_mpc_traj(self, traj_id, pos, yaw, time_pts):
        msg = Trajectory()
        msg.header.stamp = rospy.Time.now()
        msg.traj_id = traj_id
        msg.pos = pos
        msg.yaw = yaw
        msg.time = time_pts
        self.mpc_pub.publish(msg)
    
    # 判断当前位置是否接近目标位置
    def is_close(self, odom: Odometry, pose: list, xy,z):
        if not pose:
            return False
        # print(abs(odom.pose.pose.position.x - pose[0]), abs(odom.pose.pose.position.y - pose[1]), abs(odom.pose.pose.position.z - pose[2]))
        if abs(odom.pose.pose.position.x - pose[0]) < xy and abs(odom.pose.pose.position.y - pose[1]) < xy and abs(odom.pose.pose.position.z-pose[2]) < z:
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
    
    # 添加偏移量
    def add_offset(self, odom: Odometry, offset: list):
        # 提取位置和朝向
        self.position = [odom.pose.pose.position.x,
                         odom.pose.pose.position.y,
                         odom.pose.pose.position.z]
        orientation_q = odom.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation_q.x,
                                           orientation_q.y,
                                           orientation_q.z,
                                           orientation_q.w])
        self.yaw = yaw
        target_point = [self.position[0] + offset[0],
                        self.position[1] + offset[1],
                        self.position[2] + offset[2],
                        offset[3] ]
        return target_point
    
    # 发布检测控制信号
    def toggle_detection(self, enable):
        self.enable_detection = enable
        msg = Bool()
        msg.data = self.enable_detection
        self.detect_enable_pub.publish(msg)
        # rospy.loginfo(f"发布检测控制信号: {self.enable_detection}")

    # 发布super避障开关信号
    def super_change(self, super_stop):
        self.stop_super = super_stop
        msg = Bool()
        msg.data = self.stop_super
        self.stop_super_pub.publish(msg)
        # rospy.loginfo(f"发布super开关信号: {self.stop_super}")
    
    # 发布yolo检测穿越时间开关信号
    def detect_through(self, detect_through_time):
        self.detect_through_time = detect_through_time
        msg = Bool()
        msg.data = self.detect_through_time
        self.detect_through_time_pub.publish(msg)
        # rospy.loginfo(f"发布yolo检测穿越时间开关信号: {self.detect_through_time}")
    
    # 清空航点和标志位
    def clean(self):
        self.enable_detection = False  # yolo检测开关
        self.stop_super = False # super避障开关
        self.detect_through_time = False # yolo检测穿越时间开关
        self.allow_through = False  # 允许穿越旋转框标志
        self.passable = False # 动障碍穿越标志

        self.preset_point = []  # 预先设置的环前目标点
        self.set_preset_done = False
        self.preset_point2 = []  # 预先设置的环前目标点2(前5个环无需使用)
        self.set_preset_done2 = False  
        self.preset_point3 = []  # 预先设置的环前目标点3(前5个环无需使用)
        self.set_preset_done3 = False 
        self.preset_point4 = []  # 预先设置的环前目标点4
        self.set_preset_done4 = False 
        self.preset_point5 = []  # 预先设置的环前目标点4
        self.set_preset_done5 = False 
        self.align_point = [] # 与圆环中心点对齐的目标点
        self.set_align_done = False
        self.through_point = [] # 穿越圆环的目标点
        self.set_through_done = False

        self.circle_point = [] # 识别得到的圆环中心点  
        self.circle_point_buffer = [] # 圆环中心点缓存
        self.rotate_points_buffer = [] # 旋转框中心点缓存
        self.rotate_point = [] # 识别得到的旋转框中心点
        self.dynamic_point = [] # 动环中心点
        self.flygooo = False  # 允许穿越动态圆环标志 
    
    def run(self):
        # 发布当前状态
        state_now = Int8()
        state_now.data = self.state
        self.state_machine_state_pub.publish(state_now)

        # 状态0：等待进入OFFBOARD并解锁
        if self.state == 0:
            # '''切换到穿越圆环1'''
            if self.is_close(self.odom, self.takeoff_point, 0.25, 0.25):
                if self.test_mode:
                    print("测试模式，直接切换到状态10，准备通过动态圆环")
                    self.state = 10
                else:
                    print("已到达起飞点，切换状态1")
                    self.state = 1
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

        elif self.state == 1:
            # '''前往圆环1前方'''
            if not self.set_preset_done:
                self.toggle_detection(True)  # 开启检测
                self.preset_point = self.add_offset(self.odom, self.offset_1)
                self.go_to(self.preset_point)
                rospy.loginfo(f"前往圆环{self.state}前方点: {self.preset_point}, 开启检测")
                self.set_preset_done = True
                time.sleep(1)
                return
            # '''前往圆环1对齐点'''
            if not self.set_align_done and self.is_close(self.odom, self.preset_point, 0.25, 0.25) and len(self.circle_point) == 3:
                self.toggle_detection(False)  # 停止检测
                # x = self.odom.pose.pose.position.x
                self.x1, self.y1, self.z1 = self.circle_point[:3]
                self.align_point = [self.x1-1.0, self.y1, self.z1, 0.0]  # 对齐y和z坐标，x设置为圆环前1.0m
                self.go_to(self.align_point)
                rospy.loginfo(f"前往圆环{self.state}对齐点: {self.align_point}，关闭检测")
                self.set_align_done = True
                return
            # '''穿越圆环1'''
            if not self.set_through_done and self.is_close(self.odom, self.align_point, 0.25, 0.25):
                # x, y, z = self.circle_point[:3]
                self.through_point = [self.x1+ 0.4, self.y1, self.z1, 0.0]  # 飞到圆后方一点(x)
                self.go_to(self.through_point)
                rospy.loginfo(f"前往圆环{self.state}穿越点: {self.through_point}")
                self.set_through_done = True
                return
            # '''切换状态2'''
            if self.is_close(self.odom, self.through_point, 0.25, 0.25):
                self.clean()  # 清空航点和标志位
                rospy.loginfo(f"穿越圆环{self.state}完成，切换到状态{self.state+1}")
                self.state = 2 ###
                return
            return
            
        elif self.state == 2:
            # '''前往圆环2前方'''
            if not self.set_preset_done:
                self.toggle_detection(True)  # 开启检测
                self.preset_point = self.add_offset(self.odom, self.offset_2)
                self.go_to(self.preset_point)
                rospy.loginfo(f"前往圆环{self.state}前方点: {self.preset_point}, 开启检测")
                self.set_preset_done = True
                time.sleep(1)
                return
            # '''前往圆环2对齐点'''
            if not self.set_align_done and self.is_close(self.odom, self.preset_point, 0.25, 0.25) and len(self.circle_point) == 3:
                self.toggle_detection(False)  # 停止检测
                # x = self.odom.pose.pose.position.x
                x, y, z = self.circle_point[:3]
                self.align_point = [x-1.0, y, z, 0.0]   # 对齐y和z坐标，x设置为圆环前1.0m
                self.go_to(self.align_point)
                rospy.loginfo(f"前往圆环{self.state}对齐点: {self.align_point}，关闭检测")
                self.set_align_done = True
                return
            # '''穿越圆环2'''
            if not self.set_through_done and self.is_close(self.odom, self.align_point, 0.25, 0.25):
                x, y, z = self.circle_point[:3]
                self.through_point = [x + 1.0, y, z, 0.0]  # 飞到圆后方一点(x)
                self.go_to(self.through_point)
                rospy.loginfo(f"前往圆环{self.state}穿越点: {self.through_point}")
                self.set_through_done = True
                return
            # '''切换状态3'''
            if self.is_close(self.odom, self.through_point, 0.25, 0.25):
                self.clean()  # 清空航点和标志位
                rospy.loginfo(f"穿越圆环{self.state}完成，切换到状态{self.state+1}")
                self.state = 3
                return
            return
        
        elif self.state == 3:
            # '''前往圆环3前方第一个点'''
            if not self.set_preset_done:
                self.preset_point = self.add_offset(self.odom, self.offset_31)
                self.go_to(self.preset_point)
                rospy.loginfo(f"前往圆环{self.state}前方第一个点: {self.preset_point}, 开启检测")
                self.set_preset_done = True
                return
            # '''前往圆环3前方第二个点'''
            if not self.set_preset_done2 and self.is_close(self.odom, self.preset_point, 0.25, 0.15):
                self.toggle_detection(True)  # 开启检测
                self.preset_point2 = self.add_offset(self.odom, self.offset_32)
                self.go_to(self.preset_point2)
                rospy.loginfo(f"前往圆环{self.state}前方第二个点：{self.preset_point2}")
                self.set_preset_done2 = True
                time.sleep(1)
                return
            # '''前往圆环3对齐点'''
            if not self.set_align_done and self.is_close(self.odom, self.preset_point2, 0.25, 0.25) and len(self.circle_point) == 3:
                time.sleep(1.0)  # 环小，算准一点
                self.toggle_detection(False)  # 停止检测
                # x = self.odom.pose.pose.position.x
                x, y, z = self.circle_point[:3]
                self.align_point = [x-0.8, y, z, 0.0]   # 对齐y和z坐标，x设置为圆环前0.8m
                self.go_to(self.align_point)
                rospy.loginfo(f"前往圆环{self.state}对齐点: {self.align_point}，关闭检测")
                self.set_align_done = True
                return
            # '''穿越圆环3'''
            if not self.set_through_done and self.is_close(self.odom, self.align_point, 0.20, 0.15):
                x, y, z = self.circle_point[:3]
                self.through_point = [x + 0.5, y, z, 0.0]  # 飞到圆后方一点(x)
                self.go_to(self.through_point)
                rospy.loginfo(f"前往圆环{self.state}穿越点: {self.through_point}")
                self.set_through_done = True
                return
            # '''切换状态3'''
            if self.is_close(self.odom, self.through_point, 0.25, 0.25):
                self.clean()  # 清空航点和标志位 
                rospy.loginfo(f"穿越圆环{self.state}完成，切换到状态{self.state+1}")
                self.state = 4
                time.sleep(0.5) # 等待1秒，避免状态切换过快(rogmap建图)
                return
            return
        
        elif self.state == 4:
            # '''前往圆环4前方第一个点'''
            if not self.set_preset_done:
                self.toggle_detection(True)  # 开启检测
                self.preset_point = self.add_offset(self.odom, self.offset_41)
                self.go_to(self.preset_point)
                rospy.loginfo(f"前往圆环{self.state}前方第一个点: {self.preset_point}, 开启检测")
                self.set_preset_done = True
                return
            # '''前往圆环4前方第二个点'''
            if not self.set_preset_done2 and self.is_close(self.odom, self.preset_point, 0.25, 0.25):
                self.preset_point2 = self.add_offset(self.odom, self.offset_42)
                self.go_to(self.preset_point2)
                rospy.loginfo(f"前往圆环{self.state}前方第二个点：{self.preset_point2}")
                self.set_preset_done2 = True
                time.sleep(1)
                return
            # '''前往圆环4对齐点'''
            if not self.set_align_done and self.is_close(self.odom, self.preset_point2, 0.25, 0.25) and len(self.circle_point) == 3:
                time.sleep(1.5)  # 为了让检测结果稳定
                self.toggle_detection(False)  # 停止检测
                # y = self.odom.pose.pose.position.y
                x, y, z = self.circle_point[:3]
                self.align_point = [x, y-0.5, z+0.20, 90.0]  # 对齐x和z坐标，y设置为圆环前0.5m
                self.go_to(self.align_point)
                rospy.loginfo(f"前往圆环{self.state}对齐点: {self.align_point}，关闭检测")
                self.set_align_done = True
                return
            # '''穿越圆环4'''
            if not self.set_through_done and self.is_close(self.odom, self.align_point, 0.20, 0.10):
                x, y, z = self.circle_point[:3]
                self.through_point = [x, y+0.5, z+0.15, 90.0]  # 飞到圆后方一点(y)，并且在圆心上方(z)
                self.go_to(self.through_point)
                rospy.loginfo(f"前往圆环{self.state}穿越点: {self.through_point}")
                self.set_through_done = True
                return
            # '''切换状态5'''
            if self.is_close(self.odom, self.through_point, 0.25, 0.25):
                self.clean()  # 清空航点和标志位
                rospy.loginfo(f"穿越圆环{self.state}完成，切换到状态{self.state+1}")
                self.state = 5
                return
            return
        
        elif self.state == 5:
            # '''前往圆环5前方'''
            if not self.set_preset_done:
                self.toggle_detection(True)  # 开启检测
                self.preset_point = self.add_offset(self.odom, self.offset_5)
                self.go_to(self.preset_point)
                rospy.loginfo(f"前往圆环{self.state}前方点: {self.preset_point}, 开启检测")
                self.set_preset_done = True
                time.sleep(1)
                return
            # '''前往圆环5对齐点'''
            if not self.set_align_done and self.is_close(self.odom, self.preset_point, 0.25, 0.25) and len(self.circle_point) == 3:
                time.sleep(1.0) # 环小，算准一点
                self.toggle_detection(False)  # 停止检测
                # x = self.odom.pose.pose.position.x
                x, y, z = self.circle_point[:3]
                self.align_point = [x+0.8, y, z, 180.0]  # 对齐y和z坐标，x设置为圆环前1.0m
                self.go_to(self.align_point)
                rospy.loginfo(f"前往圆环{self.state}对齐点: {self.align_point}，关闭检测")
                self.set_align_done = True
                return
            # '''穿越圆环5'''
            if not self.set_through_done and self.is_close(self.odom, self.align_point, 0.20, 0.15):
                x, y, z = self.circle_point[:3]
                self.through_point = [x-0.5, y, z, 180.0]  # 飞到圆后方一点(x)
                self.go_to(self.through_point)
                rospy.loginfo(f"前往圆环{self.state}穿越点: {self.through_point}")
                self.set_through_done = True
                return
            # '''切换状态6'''
            if self.is_close(self.odom, self.through_point, 0.25, 0.25):
                self.clean()  # 清空航点和标志位
                rospy.loginfo(f"穿越圆环{self.state}完成，切换到状态{self.state+1}")
                self.state = 6
                return
            return
        
        # 处理旋转框，使用yolo检测
        elif self.state == 6:
            # '''前往旋转框6前方第一个点'''
            if not self.set_preset_done:
                self.preset_point = self.add_offset(self.odom, self.offset_61)
                self.go_to(self.preset_point)
                rospy.loginfo(f"前往旋转框{self.state}前方第一个点:{self.preset_point}")
                self.set_preset_done = True
                return
            # '''前往旋转框6前方第二个点'''
            if not self.set_preset_done2 and self.is_close(self.odom, self.preset_point, 0.25, 0.25):
                self.preset_point2 = self.add_offset(self.odom, self.offset_62)
                self.go_to(self.preset_point2)
                rospy.loginfo(f"前往旋转框{self.state}前方第二个点：{self.preset_point2}, 开启检测")
                self.set_preset_done2 = True
                return
            # '''前往旋转框6前方第三个点'''
            if not self.set_preset_done3 and self.is_close(self.odom, self.preset_point2, 0.25, 0.25):
                self.preset_point3 = self.add_offset(self.odom, self.offset_63)
                self.go_to(self.preset_point3)
                rospy.loginfo(f"前往旋转框{self.state}前方第三个点：{self.preset_point3}")
                self.set_preset_done3 = True
                time.sleep(1.0)
                self.toggle_detection(True)  # 开启检测
                return
            # '''前往旋转框6对齐点'''
            if not self.set_align_done and self.is_close(self.odom, self.preset_point3, 0.25, 0.25) and len(self.rotate_point) == 3:
                time.sleep(1.0)
                # self.toggle_detection(False)  # 停止检测
                x, y, z = self.rotate_point[:3]
                self.align_point = [x, y+2.5, z-0.1, 270.0]  # 对齐x和z坐标，y设置为旋转框前2.5m
                self.go_to(self.align_point)
                rospy.loginfo(f"前往旋转框{self.state}对齐点: {self.align_point}，关闭检测")
                self.set_align_done = True
                return
            # '''穿越旋转框6'''
            if not self.set_through_done and self.is_close(self.odom, self.align_point, 0.25, 0.20):
                self.super_change(True)  # 关闭super
                self.detect_through(True)  # 开启yolo检测穿越时间
                if self.allow_through:
                    time.sleep(3.5) #等待旋转框转到合适位置
                    self.toggle_detection(False)  # 停止检测
                    self.detect_through(False)  # 关闭yolo检测穿越时间
                    start_point = self.position = [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z]
                    mid_point = self.rotate_point
                    x, y, z = self.rotate_point[:3]
                    self.through_point = [x, y-2.5, z-0.1, 270.0]
                    pos, yaw, time_pts = self.generate_reference_trajectory(start_point, mid_point, self.through_point[:3], 3.5, 2.5, math.radians(270.0),dt=0.05)
                    self.pub_mpc_traj(traj_id=1, pos=pos, yaw=yaw, time_pts=time_pts)
                    rospy.loginfo(f"发布穿越旋转框{self.state}的固定轨迹")
                    self.set_through_done = True
                return
             # '''切换状态7'''
            if self.is_close(self.odom, self.through_point, 0.25, 0.25):
                self.super_change(False)  # 开启super
                self.clean()  # 清空航点和标志位
                rospy.loginfo(f"穿越旋转框{self.state}完成，切换到状态{self.state+1}")
                self.state = 7
                return
            return

        elif self.state == 7:
            # '''前往圆环7前方第一个点'''
            if not self.set_preset_done:
                self.preset_point = self.add_offset(self.odom, self.offset_71)
                self.go_to(self.preset_point)
                rospy.loginfo(f"前往圆环{self.state}前方第一个点: {self.preset_point}")
                self.set_preset_done = True
                return 
            # '''前往圆环7前方第二个点'''
            if not self.set_preset_done2 and self.is_close(self.odom, self.preset_point, 0.25, 0.25):
                self.preset_point2 = self.add_offset(self.odom, self.offset_72)
                self.go_to(self.preset_point2)
                rospy.loginfo(f"前往圆环{self.state}前方第二个点：{self.preset_point2}")
                self.set_preset_done2 = True
                return
            # '''前往圆环7前方第三个点'''
            if not self.set_preset_done3 and self.is_close(self.odom, self.preset_point2, 0.25, 0.25):
                self.toggle_detection(True)  # 开启检测
                self.preset_point3 = self.add_offset(self.odom, self.offset_73)
                self.go_to(self.preset_point3)
                rospy.loginfo(f"前往圆环{self.state}前方第三个点：{self.preset_point3}")
                self.set_preset_done3 = True
            # '''前往圆环7前方第四个点'''
            if not self.set_preset_done4 and self.is_close(self.odom, self.preset_point3, 0.25, 0.25):
                self.toggle_detection(True)  # 开启检测
                self.preset_point4 = self.add_offset(self.odom, self.offset_74)
                self.go_to(self.preset_point4)
                rospy.loginfo(f"前往圆环{self.state}前方第四个点：{self.preset_point3},开启检测")
                self.set_preset_done4 = True
             # '''开启动障碍检测'''
            if not self.set_preset_done5 and self.is_close(self.odom, self.preset_point4, 0.5, 0.25):
                if not self.pass_dynamic_way:
                    self.detect_through(True)  # 开启yolo检测穿越时间
                    self.set_preset_done5 = True
                return     
            # '''前往圆环7前方第五个点'''
            if self.passable:
                self.passable = False
                self.preset_point5 = self.add_offset(self.odom, self.offset_75)
                self.go_to(self.preset_point5)
                rospy.loginfo(f"前往圆环{self.state}前方第五个点: {self.preset_point5}")
                self.detect_through(False)  # 关闭yolo检测穿越时间
                self.toggle_detection(False)  # 开启检测
                time.sleep(1.0)
                self.toggle_detection(True)  # 开启检测
                return
            # '''前往圆环7对齐点'''
            if not self.set_align_done and self.is_close(self.odom, self.preset_point5, 0.25, 0.25) and len(self.circle_point) == 3:
                time.sleep(1.0)
                self.toggle_detection(False)  # 停止检测
                # x = self.odom.pose.pose.position.x
                x, y, z = self.circle_point[:3]
                self.align_point = [x-1.0, y, z, 0.0]  # 对齐y和z坐标，x设置为圆环前1.0m
                self.go_to(self.align_point)
                rospy.loginfo(f"前往圆环{self.state}对齐点: {self.align_point}，关闭检测")
                self.set_align_done = True
                return
            # '''穿越圆环7'''
            if not self.set_through_done and self.is_close(self.odom, self.align_point, 0.30, 0.25):
                x, y, z = self.circle_point[:3]
                self.through_point = [x+0.5, y, z, 0.0]  # 飞到圆后方一点(x)
                self.go_to(self.through_point)
                rospy.loginfo(f"前往圆环{self.state}穿越点: {self.through_point}")
                self.set_through_done = True
                return
             # '''切换状态8'''
            if self.is_close(self.odom, self.through_point, 0.25, 0.25):
                self.clean()  # 清空航点和标志位
                rospy.loginfo(f"穿越圆环{self.state}完成，切换到状态{self.state+1}")
                self.state = 8
                return
            return
        
        elif self.state == 8:
            # '''前往圆环8前方第一个点'''
            if not self.set_preset_done:
                self.preset_point = self.add_offset(self.odom, self.offset_81)
                self.go_to(self.preset_point)
                rospy.loginfo(f"前往圆环{self.state}前方第一个点: {self.preset_point}")
                self.set_preset_done = True
                return 
            # '''前往圆环8前方第二个点'''
            if not self.set_preset_done2 and self.is_close(self.odom, self.preset_point, 0.5, 0.25):
                self.preset_point2 = self.add_offset(self.odom, self.offset_82)
                self.go_to(self.preset_point2)
                rospy.loginfo(f"前往圆环{self.state}前方第二个点：{self.preset_point2}, 开启检测")
                self.set_preset_done2 = True
                return
            # '''前往圆环8前方第三个点'''
            if not self.set_preset_done3 and self.is_close(self.odom, self.preset_point2, 0.5, 0.25):
                self.toggle_detection(True)  # 开启检测
                self.preset_point3 = self.add_offset(self.odom, self.offset_83)
                self.go_to(self.preset_point3)
                rospy.loginfo(f"前往圆环{self.state}前方第二个点：{self.preset_point3}")
                self.set_preset_done2 = True
                time.sleep(1)
            # '''前往圆环8对齐点'''
            if not self.set_align_done and self.is_close(self.odom, self.preset_point3, 0.25, 0.25) and len(self.circle_point) == 3:
                time.sleep(1.5)
                self.toggle_detection(False)  # 停止检测
                # x = self.odom.pose.pose.position.x
                x, y, z = self.circle_point[:3]
                self.align_point = [x+1.0, y, z, 180.0]  # 对齐y和z坐标，x设置为圆环前1.0m
                self.go_to(self.align_point)
                rospy.loginfo(f"前往圆环{self.state}对齐点: {self.align_point}，关闭检测")
                self.set_align_done = True
                return
            # '''穿越圆环8'''
            if not self.set_through_done and self.is_close(self.odom, self.align_point, 0.30, 0.25):
                x, y, z = self.circle_point[:3]
                self.through_point = [x-0.5, y, z, 180.0]  # 飞到圆后方一点(x)
                self.go_to(self.through_point)
                rospy.loginfo(f"前往圆环{self.state}穿越点: {self.through_point}")
                self.set_through_done = True
                return
             # '''切换状态9'''
            if self.is_close(self.odom, self.through_point, 0.25, 0.25):
                self.clean()  # 清空航点和标志位
                rospy.loginfo(f"穿越圆环{self.state}完成，切换到状态{self.state+1}")
                self.state = 9
                return
            return
        
        elif self.state == 9:
            # '''前往圆环9前方第一个点'''
            if not self.set_preset_done:
                self.preset_point = self.add_offset(self.odom, self.offset_91)
                self.go_to(self.preset_point)
                rospy.loginfo(f"前往圆环{self.state}前方第一个点：{self.preset_point}")
                self.set_preset_done = True
                return 
            # '''前往圆环9前方第二个点'''
            if not self.set_preset_done2 and self.is_close(self.odom, self.preset_point, 0.5, 0.25):
                self.toggle_detection(True)  # 开启检测
                self.preset_point2 = self.add_offset(self.odom, self.offset_92)
                self.go_to(self.preset_point2)
                rospy.loginfo(f"前往圆环{self.state}前方第二个点：{self.preset_point2}")
                self.set_preset_done2 = True
                time.sleep(1)
                return
            # '''前往圆环9对齐点'''
            if not self.set_align_done and self.is_close(self.odom, self.preset_point2, 0.25, 0.25) and len(self.circle_point) == 3:
                time.sleep(0.5)
                self.toggle_detection(False)  # 停止检测
                # x = self.odom.pose.pose.position.x
                x, y, z = self.circle_point[:3]
                self.align_point = [x+1.0, y, z, 180.0]  # 对齐y和z坐标，x设置为圆环前1.0m
                self.go_to(self.align_point)
                rospy.loginfo(f"前往圆环{self.state}对齐点: {self.align_point}，关闭检测")
                self.set_align_done = True
                return
            # '''穿越圆环9'''
            if not self.set_through_done and self.is_close(self.odom, self.align_point, 0.30, 0.25):
                x, y, z = self.circle_point[:3]
                self.through_point = [x-1.0, y, z, 180.0]  # 飞到圆后方一点(x)
                self.go_to(self.through_point)
                rospy.loginfo(f"前往圆环{self.state}穿越点: {self.through_point}")
                self.set_through_done = True
                return
             # '''切换状态12, 前往降落点12'''
            if self.is_close(self.odom, self.through_point, 0.25, 0.25):
                self.clean()  # 清空航点和标志位
                rospy.loginfo(f"穿越圆环{self.state}完成，切换到状态{self.state+1}")
                self.state = 12
                return
            return
        
        elif self.state == 10:
            # ‘’‘测试模式，直接前往动环10前方点'''
            if self.test_mode:
                offset = [9.5, 5.8, 0.1, 180]
                preset_point = self.add_offset(self.odom, offset)
                self.go_to(preset_point)
                rospy.loginfo(f"直接前往动环前方点: {preset_point}")
                self.test_mode = False
                time.sleep(12)
                return 
            # ‘’‘检测并前往动环最左边点‘’‘
            if not self.set_align_done:
                self.toggle_detection(True) # 开启检测
                # 非空，已赋值
                if self.dynamic_point:
                    self.toggle_detection(False)  # 停止检测
                    self.x10, self.y10, self.z10 = self.dynamic_point[:3]
                    self.align_point = [self.x10+0.5, self.y10, self.z10-0.1, 180.0]  # 对齐x和z坐标，x设置为圆环右边0.2m
                    self.go_to(self.align_point)
                    rospy.loginfo(f"前往动态圆环{self.state}对齐点: {self.align_point}，关闭检测")
                    self.set_align_done = True
                return
            # '''穿越动环10'''
            if not self.set_through_done and self.is_close(self.odom, self.align_point, 0.25, 0.15):
                self.super_change(True)  # 关闭super
                self.flygooo = True  # 开启动态圆环飞行
                if self.passable:
                    start =  [self.posess.x, self.posess.y, self.posess.z]
                    self.through_point = [self.x10-0.5, self.y10, self.z10-0.1, 180.0]  # 飞到圆后方一点(x)
                    pos, yaw, time_pts = self.generate_dynamic_traj(start, self.through_point, time=0.8, dt=0.02)
                    self.pub_mpc_traj(traj_id=1, pos=pos, yaw=yaw, time_pts=time_pts)
                    rospy.loginfo(f"穿越动态圆环10")
                    self.set_through_done = True
                return
            # '''切换状态11'''
            if self.is_close(self.odom, self.through_point, 0.25, 0.25):
                self.clean()  # 清空航点和标志位
                rospy.loginfo(f"穿越动态圆环{self.state}完成，切换到状态{self.state+1}")
                self.state = 11
                return
            return
        
        elif self.state == 11:
            return
            # ‘’‘等待动环11停下，然后前往对齐点‘’‘
            if not self.set_align_done:
                self.toggle_detection(True) # 开启检测
                if self.allow_through:
                    self.toggle_detection(False)  # 停止检测
                    x, y, z = self.circle_point[:3]
                    self.align_point = [x+0.8, y, z, 180.0]  # 对齐y和z坐标，x设置为圆环前0.8m
                    self.go_to(self.align_point)
                    rospy.loginfo(f"前往动态圆环{self.state}对齐点: {self.align_point}，关闭检测")
                    self.set_align_done = True
                return
            # '''穿越动环11'''
            if not self.set_through_done and self.is_close(self.odom, self.align_point, 0.15, 0.15):
                x, y, z = self.circle_point[:3]
                self.through_point = [x-0.6, y, z, 180.0]  # 飞到圆后方一点(x)
                self.go_to(self.through_point)
                rospy.loginfo(f"前往动态圆环{self.state}穿越点: {self.through_point}")
                self.set_through_done = True
                return
            # '''切换状态12'''
            if self.is_close(self.odom, self.through_point, 0.25, 0.25):
                self.clean()  # 清空航点和标志位
                rospy.loginfo(f"穿越动态圆环{self.state}完成，切换到状态{self.state+1}")
                self.state = 12
                return
            return

        elif self.state == 12:
            # 前往降落等待点1
            if not self.set_preset_done:
                self.preset_point = self.add_offset(self.odom, self.offset_121)
                self.go_to(self.preset_point)
                rospy.loginfo(f"前往降落等待点1: {self.preset_point}")
                self.set_preset_done = True
                return 
            # 前往降落等待点2
            if not self.set_preset_done2 and self.is_close(self.odom, self.preset_point, 0.25, 0.25):
                self.preset_point2 = self.add_offset(self.odom, self.offset_122)
                self.go_to(self.preset_point2)
                rospy.loginfo(f"前往降落等待点2: {self.preset_point2}")
                self.set_preset_done2 = True
                return 
            # 前往降落等待点3
            if not self.set_preset_done3 and self.is_close(self.odom, self.preset_point2, 0.25, 0.25):
                self.apriltag = AprilTagPoseEstimator(self.camera_matrix, self.tag_size, self.real_fly, is_debug=True)
                self.preset_point3 = self.add_offset(self.odom, self.offset_123)
                self.go_to(self.preset_point3)
                rospy.loginfo(f"前往降落等待点3: {self.preset_point3}")
                self.set_preset_done3 = True
                return 
            # 关闭super
            if not self.set_preset_done4:
                if self.is_close(self.odom, self.preset_point3, 0.25, 0.25):
                    self.super_change(True)  # 关闭super
                    self.set_preset_done4 = True
                return
            # 发布降落轨迹
            if self.apriltag.can_see and not self.set_align_done:
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
                end = pt_world[:3].flatten() + np.array([0, 0, 0.3]) # 降落点z加上0.3m
                pos, yaw, time_pts = self.caculate_land_trajectory(start, end, math.radians(0.0),dt=0.05)
                self.pub_mpc_traj(traj_id=1, pos=pos, yaw=yaw, time_pts=time_pts)
            # 如果高度足够低,切换状态13
            if self.posess.z < 0.4:
                self.set_align_done = True
                # self.set_mode_client(0,'AUTO.LAND')
                self.state = 13
                rospy.loginfo(f"直接降落")
                # time.sleep(1)
                # self.arming_client(False)
            return

        elif self.state == 13:
            # 比赛结束
            rospy.loginfo(f"比赛结束")
            return
    
    

        # elif self.state == 1:
        #     if  self.is_close(self.odom, self.point[self.point_count],0.3,0.3):
        #         self.state = 2
        #         self.point_count+=1
        #         time.sleep(1.5)
        #         return
        #     else:
        #         self.go_to(self.point[self.point_count])
        #         print(f'飞往第{self.point_count+1}个点')
        #         print(self.point[self.point_count])
        #         return
            
    def load_params(self):
        # 读取先验数据
        self.point = rospy.get_param("~predefined_points", [])
        rospy.loginfo(f"Loaded {len(self.point)} points.")

        # self.point[0] = rospy.get_param('~Inital_pose')
        # self.point[1] = rospy.get_param('~point1')
        # self.point[2] = rospy.get_param('~point2')
        # self.point[3] = rospy.get_param('~point3')
        # self.point[4] = rospy.get_param('~point4')
        # self.point[5] = rospy.get_param('~point5')
        # self.point[6] = rospy.get_param('~point6')
        # self.point[7] = rospy.get_param('~point7')
        # self.point[8] = rospy.get_param('~point8')
        # self.point[9] = rospy.get_param('~point9')
        # self.point[10] = rospy.get_param('~point10')
        # self.point[11] = rospy.get_param('~point11')

if __name__ == "__main__":
    state_machine = StateMachine()
    rate = rospy.Rate(20)  
    while not rospy.is_shutdown():
        state_machine.run()
        rate.sleep()
