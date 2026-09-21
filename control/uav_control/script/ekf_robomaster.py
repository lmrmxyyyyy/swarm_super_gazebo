#! /usr/bin/python3.8
import rospy
import numpy as np
from queue import Queue
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist ,Point,Vector3,Quaternion,Vector3Stamped
from threading import Lock
import casadi as ca
from scipy.signal import butter, lfilter
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Int8
MAX_INIT_NUM=50

# Quaternion Multiplication
def quat_mult(q1,q2):
    ans = ca.vertcat(q2[0,:] * q1[0,:] - q2[1,:] * q1[1,:] - q2[2,:] * q1[2,:] - q2[3,:] * q1[3,:],
           q2[0,:] * q1[1,:] + q2[1,:] * q1[0,:] - q2[2,:] * q1[3,:] + q2[3,:] * q1[2,:],
           q2[0,:] * q1[2,:] + q2[2,:] * q1[0,:] + q2[1,:] * q1[3,:] - q2[3,:] * q1[1,:],
           q2[0,:] * q1[3,:] - q2[1,:] * q1[2,:] + q2[2,:] * q1[1,:] + q2[3,:] * q1[0,:])
    return ans

# Quaternion-Vector Rotation
def rotate_quat(q1,v1):
    ans = quat_mult(quat_mult(q1, ca.vertcat(0, v1)), ca.vertcat(q1[0,:],-q1[1,:], -q1[2,:], -q1[3,:]))
    return ca.vertcat(ans[1,:], ans[2,:], ans[3,:]) # to covert to 3x1 vec

class LowPassFilter:
    def __init__(self, cutoff_freq, sample_rate, order=2):
        nyquist = 0.5 * sample_rate  # 奈奎斯特频率
        normal_cutoff = cutoff_freq / nyquist
        self.b, self.a = butter(order, normal_cutoff, btype='low', analog=False)
        self.zi = np.zeros((max(len(self.b), len(self.a)) - 1,))  # 初始化状态

    def filter(self, data):
        data, self.zi = lfilter(self.b, self.a, [data], zi=self.zi)
        return data[0]


    
class EKF_Estimator:
    def __init__(self):
        # 初始化队列和锁
        self.odom_queue = Queue(maxsize=100)
        self.imu_queue = Queue(maxsize=100)
        self.lock = Lock()

        # 初始状态 [p_x, p_y, p_z, v_x, v_y, v_z, q_w, q_x, q_y, q_z, omega_x, omega_y, omega_z, b_a, b_w]
        self.state = np.zeros(19)
        self.state[6:10] = [1, 0, 0, 0]  # 初始四元数表示无旋转
        self.P = np.eye(19)  # 初始协方差

        # 噪声协方差
        self.Q = np.eye(19) * 0.001  # 过程噪声
        self.R = np.eye(3) * 0.001  # 观测噪声 (仅位置观测)

        

        # 上一次 IMU 的时间戳
        self.last_imu_time = None
        self.last_a_world =np.array([0,0,0])
        self.last_v_pred =np.array([0,0,0])
        self.last_gyro_corrected=np.array([0,0,0])

        sample_rate = 100  # 采样频率 (Hz)
        cutoff_freq = 40   # 截止频率 (Hz)
        # self.low_pass_filter = LowPassFilter(cutoff_freq, sample_rate)
        self.low_pass_filter_x = LowPassFilter(cutoff_freq, sample_rate)
        self.low_pass_filter_y = LowPassFilter(cutoff_freq, sample_rate)
        self.low_pass_filter_z = LowPassFilter(cutoff_freq, sample_rate)

        self.mean_acc = np.zeros(3)  # 加速度均值
        self.mean_gyr = np.zeros(3)  # 角速度均值
        self.cov_acc = np.zeros(3)   # 加速度协方差
        self.cov_gyr = np.zeros(3)   # 角速度协方差
        self.N = 1                   # 样本计数
        self.b_first_frame = True    # 标志，表示是否是第一帧IMU数据
        self.imu_need_init=False
        self.pose_init=False

        # 初始四元数
        self.q_initial = R.from_quat([0,0,0,1])

        # 初始位置
        self.initial_position = np.array([0,0,0])
        self.gnss_is_good =True

        # 订阅话题
        rospy.Subscriber("/airsim_node/drone_1/imu/imu", Imu, self.imu_callback)
        # rospy.Subscriber("/Odometry", Odometry, self.odom_callback)

        rospy.Subscriber("/airsim_node/drone_1/gps", PoseStamped, self.gnss_callback)
        rospy.Subscriber("/airsim_node/drone_1/debug/pose_gt", PoseStamped, self.debug_callback)

        rospy.Subscriber("/airsim_node/initial_pose", PoseStamped, self.init_callback)
        # 订阅风速话题
        rospy.Subscriber('/airsim_node/drone_1/debug/wind', Vector3Stamped, self.wind_callback)
        rospy.Subscriber('/state_machine', Int8,  self.state_callback)
        rospy.Subscriber('/localization_odom', Odometry, self.lidar_odom_callback)
        # 发布融合后的位姿
        self.pose_pub = rospy.Publisher("/fused_pose",Odometry, queue_size=1)
        self.debug_pub = rospy.Publisher("/debug_pose",Odometry, queue_size=10)
        self.debug_gps_pub = rospy.Publisher("/debug_gps_pose",Odometry, queue_size=10)
        # 发布转换后的风速消息
        self.wind_pub = rospy.Publisher('/wind_local', Vector3Stamped, queue_size=1)
        self.fused_pose = Odometry()
        self.state_machine=None
        self.pub_flag=0

    # def odom_callback(self, msg):
    #     with self.lock:
    #         if not self.odom_queue.full()and not self.imu_need_init:
    #             self.odom_queue.put(msg)
    def init_callback(self, msg):
        if self.pose_init==False and not msg==None:
                    # 初始四元数
            self.q_initial = R.from_quat([
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,

            ])

        # 初始位置
            self.initial_position = np.array([
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z,
            ])
            self.pose_init=True
            # print(self.initial_position)

    def wind_callback(self, msg):
        with self.lock:
            if self.pose_init:
                # 获取风速向量
                wind_global = np.array([msg.vector.x, msg.vector.y, msg.vector.z])
                
                # 将风速向量从全局坐标系转换到局部坐标系
                wind_local = self.q_initial.inv().apply(wind_global)
                 # 构造 Vector3Stamped 消息
                wind_local_msg = Vector3Stamped()
                wind_local_msg.header.stamp = rospy.Time.now()
                wind_local_msg.header.frame_id = "local_frame"  # 你可以更改为实际的局部坐标系名称
                wind_local_msg.vector.x = wind_local[0]
                wind_local_msg.vector.y = wind_local[1]
                wind_local_msg.vector.z = wind_local[2]

                # 发布转换后的风速消息
                self.wind_pub.publish(wind_local_msg)
                # 输出转换后的风速向量
                rospy.loginfo(f"Wind in local frame: {wind_local}")
                
    def gnss_callback(self, msg):
        with self.lock:
            if not self.odom_queue.full() and not self.imu_need_init and self.pose_init:
                if self.state_machine==None:
                    print("no state_machine input")
                elif self.state_machine.data==None or self.state_machine.data==26 or self.state_machine.data==27:
                    return
                elif self.state_machine.data==None or self.state_machine.data==36 or self.state_machine.data==37:
                    return
                if msg.pose.position.x<0.01 and msg.pose.position.y<0.01 and msg.pose.position.z<0.01:
                    print("no Gnss signal")
                    self.gnss_is_good=False
                    return
                self.gnss_is_good=True
                # print("GPS_callback:########################!")
                gnss_position = np.array([
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                ])
                # 计算局部坐标
                offset = gnss_position - self.initial_position
                corrected_offset = self.q_initial.inv().apply(offset)

                # 转换后的局部坐标
                # 构造 Odometry 消息
                local_pose = Odometry()
                local_pose.header.stamp = rospy.Time.now()
                local_pose.header.frame_id = "camera_init"  # 修改为实际坐标系

                # 设置位置
                local_pose.pose.pose.position = Point(
                    x=corrected_offset[0],
                    y=corrected_offset[1],
                    z=corrected_offset[2]
                )
                self.debug_gps_pub.publish(local_pose)
                # 将局部坐标放入队列
                self.odom_queue.put(local_pose)
                self.process_once()
                self.process_once()
                

    def lidar_odom_callback(self, msg):
        with self.lock:
            if not self.odom_queue.full() and not self.imu_need_init and self.pose_init:
                if self.state_machine.data!=23 and self.state_machine.data!=26 and self.state_machine.data!=27 \
                and self.state_machine.data!=33 and self.state_machine.data!=36 and self.state_machine.data!=37 :
                    return
                
                # elif self.gnss_is_good==True:
                #     return
                print("lidar_odom_callback:$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$!")
                lidar_position = np.array([
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                ])
                # 转换后的局部坐标
                # 构造 Odometry 消息
                local_pose = Odometry()
                local_pose.header.stamp = rospy.Time.now()
                local_pose.header.frame_id = "camera_init"  # 修改为实际坐标系

                # 设置位置
                local_pose.pose.pose.position = Point(
                    x=lidar_position[0],
                    y=lidar_position[1],
                    z=lidar_position[2]
                )
                self.debug_gps_pub.publish(local_pose)
                # 将局部坐标放入队列
                self.odom_queue.put(local_pose)
                self.process_once()
                self.process_once()
                
    def debug_callback(self, msg):
        with self.lock:
            if not self.odom_queue.full() and not self.imu_need_init and self.pose_init:
                debug_position = np.array([
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                ])
  
                # 计算局部坐标
                offset = debug_position - self.initial_position
                corrected_offset = self.q_initial.inv().apply(offset)
                corrected_offset = self.q_initial.inv().apply(offset)
                # 提取并转换四元数
                orientation = msg.pose.orientation
                input_quaternion = np.array([ orientation.x, orientation.y, orientation.z,orientation.w])  # wxyz格式
                local_quaternion = self.q_initial.inv() * R.from_quat(input_quaternion)
                # 转换后的局部坐标
                # 构造 Odometry 消息
                debug_pose = Odometry()
                debug_pose.header.stamp = rospy.Time.now()
                debug_pose.header.frame_id = "camera_init"  # 修改为实际坐标系

                # 设置位置
                debug_pose.pose.pose.position = Point(
                    x=corrected_offset[0],
                    y=corrected_offset[1],
                    z=corrected_offset[2]
                )
                # 设置四元数
                converted_quat = local_quaternion.as_quat()  # 返回xyzw格式
                debug_pose.pose.pose.orientation = Quaternion(
                    x=converted_quat[0],
                    y=converted_quat[1],
                    z=converted_quat[2],
                    w=converted_quat[3],
                )

                self.debug_pub.publish(debug_pose)


    def imu_callback(self, msg):
        # if self.imu_need_init:
        #     cur_acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        #     cur_gyr = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        #     # 如果是第一帧IMU数据，初始化
        #     if self.b_first_frame:
        #         self.mean_acc = cur_acc
        #         self.mean_gyr = cur_gyr
        #         self.N = 1
        #         self.b_first_frame = False

        #     # 更新加速度均值和角速度均值
        #     self.mean_acc += (cur_acc - self.mean_acc) / self.N
        #     self.mean_gyr += (cur_gyr - self.mean_gyr) / self.N

        #     # 更新加速度和角速度协方差
        #     self.cov_acc = self.cov_acc * (self.N - 1.0) / self.N + (cur_acc - self.mean_acc) * (cur_acc - self.mean_acc) * (self.N - 1.0) / (self.N * self.N)
        #     self.cov_gyr = self.cov_gyr * (self.N - 1.0) / self.N + (cur_gyr - self.mean_gyr) * (cur_gyr - self.mean_gyr) * (self.N - 1.0) / (self.N * self.N)
        #     # 更新加速度偏置b_a和角速度偏置b_w
        #     # self.ba = self.mean_acc  # 加速度偏置
        #     # self.bg = self.mean_gyr  # 角速度偏置
        #     self.state[13:16]=self.mean_acc
        #     self.state[16:19]=self.mean_gyr 
        #     # 估计重力向量
        #     self.grav = self.mean_acc / np.linalg.norm(self.mean_acc) * 9.81  # 重力矢量归一化，假设加速度方向即为重力方向
        #     self.P[13,13]=self.P[14,14]=self.P[15,15]=0.0001
        #     self.P[16,16]=self.P[17,17]=self.P[18,18]=0.001
        #     print(f"init:{self.N/MAX_INIT_NUM*100}%")
        #     self.N += 1
        #     if self.N >MAX_INIT_NUM:
        #         self.imu_need_init=False
        #     return
        # print("Imu debug")
        with self.lock:
            if not self.imu_queue.full() and not self.imu_need_init and self.pose_init:
                self.imu_queue.put(msg)
            self.process_once()
            self.process_once()
            
      

    

    def skew_symmetric(self,a):
        """
        Compute the skew-symmetric matrix of a vector.
        :param v: 3D vector
        :return: 3x3 skew-symmetric matrix
        """
        return np.array([
            [0, -a[2], a[1]],
            [a[2], 0, -a[0]],
            [-a[1], a[0], 0]
        ])

    def ekf_predict(self, dt, imu_data):
        """
        EKF 预测
        :param dt: 时间间隔
        :param imu_data: IMU 数据 (加速度和四元数)
        """
        if abs(dt)>1.0:
            print("Imu time errorr")
            return
        # 提取 IMU 数据
        rax, ray, raz = imu_data["linear_acceleration"]
        ax = self.low_pass_filter_x.filter(rax)
        ay = self.low_pass_filter_y.filter(ray)
        az = self.low_pass_filter_z.filter(raz)
        q_w, q_x, q_y, q_z = imu_data["orientation"]
        # 当前状态
        b_a = self.state[13:16]  # 加速度偏置
        b_g = self.state[16:19]
        # print(np.array([ax, ay, az]))
        # 校正后的加速度
        a_corrected = np.array([ax, ay, az]) -np.array(b_a)
        gyro_corrected= np.array(imu_data["angular_velocity"])-np.array(b_g)
        print("ra:",np.array([rax, ray, raz]))
        # print("ba:",np.array(b_a))
        # print( "g",self.grav)
        # print( "ba",b_a)
        # 预测四元数
        q_pred = ca.veccat(q_w,q_x,q_y,q_z)
        # conversion_matrix = np.diag([1, -1, -1])
        # 预测速度和位置
        a_world = np.array(rotate_quat(q_pred,a_corrected)).flatten()
        v_pred = self.state[3:6] + (a_world + np.array([0, 0, 9.81])) * dt
        # print(a_world+ np.array([0, 0, 9.81]) ,a_corrected)
        p_pred = self.state[:3] + self.state[3:6] * dt + 0.5 * (a_world+ np.array([0, 0, 9.81])) * dt**2
        # v_pred = self.state[3:6] + ((a_world+ self.last_a_world)/2 + np.array([0, 0, 9.81])) * dt
        # # print(a_world+ np.array([0, 0, 9.81]) ,a_corrected)
        # p_pred = self.state[:3] + (self.state[3:6]+v_pred)/2 * dt + 0.5 * ((a_world+ self.last_a_world)/2+ np.array([0, 0, 9.81])) * dt**2
        # print(p_pred)
        # 更新状态
        self.state[:3] = p_pred  # 位置
        self.state[3:6] = v_pred  # 速度
        self.state[6:10] = [q_w, q_x, q_y, q_z]  # 四元数
        self.state[10:13] = gyro_corrected   # 角速度

        self.last_a_world =a_world
        self.last_v_pred =v_pred
        self.last_gyro_corrected=gyro_corrected
        # 状态转移矩阵
        F = np.eye(19)
        F[:3, 3:6] = np.eye(3) * dt  # 位置依赖速度
        # F[3:6, 6:9] = -R_world @ self.skew_symmetric(a_corrected) * dt  # 速度依赖姿态

        # 更新协方差
        self.P = F @ self.P @ F.T + self.Q

    def ekf_update(self, measurement):
        """
        EKF 更新（校正）
        :param measurement: 来自 SLAM 的位置测量
        """
        H = np.zeros((3, 19))
        H[:3, :3] = np.eye(3)  # 仅观测位置
        #measurement[:-3]=self.state[3:6]
        # 计算残差
        print("gps:",measurement)
        y = measurement - self.state[:3]

        # 计算观测协方差
        S = H @ self.P @ H.T + self.R

        # 计算卡尔曼增益
        K = self.P @ H.T @ np.linalg.inv(S)

        # 更新状态和协方差
        self.state += K @ y
        I = np.eye(19)
        self.P = (I - K @ H) @ self.P

    # def process(self):
    #     rate = rospy.Rate(500)  # 100Hz 主循环
    #     while not rospy.is_shutdown():
    #         with self.lock:
                
    #             # 使用 IMU 数据预测
    #             if not self.imu_queue.empty():
    #                 imu_msg = self.imu_queue.get()
    #                 imu_data = {
    #                     "linear_acceleration": [
    #                         imu_msg.linear_acceleration.x,
    #                         imu_msg.linear_acceleration.y,
    #                         imu_msg.linear_acceleration.z,
    #                     ],
    #                     "angular_velocity": [
    #                         imu_msg.angular_velocity.x,
    #                         imu_msg.angular_velocity.y,
    #                         imu_msg.angular_velocity.z,
    #                     ],
    #                     "orientation": [
    #                         imu_msg.orientation.w,
    #                         imu_msg.orientation.x,
    #                         imu_msg.orientation.y,
    #                         imu_msg.orientation.z,
    #                     ],
    #                 }

    #                 now = imu_msg.header.stamp.to_sec() # rospy.Time.now().to_sec()
    #                 if self.last_imu_time is not None:
    #                     dt = now - self.last_imu_time
    #                     self.ekf_predict(dt, imu_data)
    #                     self.set_pose_pub()
    #                 self.last_imu_time = now

    #             # 使用 Odom 数据更新
    #             if not self.odom_queue.empty():
    #                 odom_msg = self.odom_queue.get()
    #                 z = np.array([
    #                     odom_msg.pose.pose.position.x,
    #                     odom_msg.pose.pose.position.y,
    #                     odom_msg.pose.pose.position.z,
    #                     # odom_msg.twist.twist.linear.x,
    #                     # odom_msg.twist.twist.linear.y,
    #                     # odom_msg.twist.twist.linear.z,
    #                 ])
    #                 self.ekf_update(z)
    #                 self.set_pose_pub()
    #         rate.sleep()
            
    def process_once(self):
        # with self.lock:
                    # 使用 IMU 数据预测
        if not self.imu_queue.empty():
            imu_msg = self.imu_queue.get()
            imu_data = {
                "linear_acceleration": [
                    imu_msg.linear_acceleration.x,
                    imu_msg.linear_acceleration.y,
                    imu_msg.linear_acceleration.z,
                ],
                "angular_velocity": [
                    imu_msg.angular_velocity.x,
                    imu_msg.angular_velocity.y,
                    imu_msg.angular_velocity.z,
                ],
                "orientation": [
                    imu_msg.orientation.w,
                    imu_msg.orientation.x,
                    imu_msg.orientation.y,
                    imu_msg.orientation.z,
                ],
            }

            now = imu_msg.header.stamp.to_sec() # rospy.Time.now().to_sec()
            if self.last_imu_time is not None:
                dt = now - self.last_imu_time
                if abs(dt) >1.0:
                    print("Imu time error")
                    self.last_imu_time = now
                    return
                self.ekf_predict(dt, imu_data)
                if self.pub_flag<99:
                    self.set_pose_pub(imu_msg)
                    self.pub_flag+=1
                else:
                    self.pub_flag=0
                    # self.set_pose_pub()
                    
            self.last_imu_time = now

        # 使用 Odom 数据更新
        if not self.odom_queue.empty():
            odom_msg = self.odom_queue.get()
            z = np.array([
                odom_msg.pose.pose.position.x,
                odom_msg.pose.pose.position.y,
                odom_msg.pose.pose.position.z,
                # odom_msg.twist.twist.linear.x,
                # odom_msg.twist.twist.linear.y,
                # odom_msg.twist.twist.linear.z,
            ])
            self.ekf_update(z)
            # self.set_pose_pub()



    def set_pose_pub(self,msg):
        self.fused_pose.header.stamp = msg.header.stamp
        self.fused_pose.header.frame_id = "camera_init"
        self.fused_pose.pose.pose.position.x = self.state[0]
        self.fused_pose.pose.pose.position.y = self.state[1]
        self.fused_pose.pose.pose.position.z = self.state[2]
        self.fused_pose.twist.twist.linear.x= self.state[3]
        self.fused_pose.twist.twist.linear.y= self.state[4]
        self.fused_pose.twist.twist.linear.z= self.state[5]
        self.fused_pose.pose.pose.orientation.w = self.state[6]
        self.fused_pose.pose.pose.orientation.x = self.state[7]
        self.fused_pose.pose.pose.orientation.y = self.state[8]
        self.fused_pose.pose.pose.orientation.z = self.state[9]
        self.fused_pose.twist.twist.angular.x= self.state[10]
        self.fused_pose.twist.twist.angular.y= self.state[11]
        self.fused_pose.twist.twist.angular.z= self.state[12]
         # 发布融合后的位姿
        print("publish fused_pose")
        self.pose_pub.publish(self.fused_pose)

    def state_callback(self, msg: Int8):
        self.state_machine = msg
        print("self.state_machine:",self.state_machine.data)

if __name__ == "__main__":
    rospy.init_node("pose_estimator")
    estimator = EKF_Estimator()
    rospy.spin() 
    # estimator.process()
