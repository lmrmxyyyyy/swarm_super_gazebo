#!/usr/bin/env python3
# coding=utf-8 
import rospy
import math
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from ius_msgs.msg import Trajectory
from tf.transformations import quaternion_from_euler

class UAVState:
    """单个无人机的状态信息"""
    def __init__(self, name):
        self.name = name
        self.odom = Odometry()
        self.mavros_state = State()
        self.goal_pub = None
        self.state_pub = None
        self.arming_client = None
        self.set_mode_client = None
        self.odom_sub = None
        self.mavros_state_sub = None
        self.trajectory_sub = None
        self.odom_received = False
        self.mavros_state_received = False
        self.trajectory_ready = False
        self.last_odom_receive_time = rospy.Time(0)

class StateMachine:
    def __init__(self):
        rospy.init_node("multi_uav_state_machine")

        # 状态变量（全局状态，用于同步所有无人机）
        self.global_state = 0
        self.last_request = rospy.Time.now()
        self.point_count = 0
        self.max_points = int(rospy.get_param("~max_points", 15))
        self.point_num = int(rospy.get_param("~point_num", 1))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 0.5))
        self.control_prestream_duration = float(
            rospy.get_param("~control_prestream_duration", 1.0)
        )
        self.goal_retry_period = float(rospy.get_param("~goal_retry_period", 1.0))
        if self.point_num < 1 or self.point_num > self.max_points:
            raise rospy.ROSException("~point_num must be in [1, ~max_points].")

        self.uav_names = self.load_uav_names()
        self.uavs = {}
        self.points = {}
        self.active_goal_key = None
        self.last_goal_publish_time = rospy.Time(0)
        self.trajectories_ready_since = None

        for uav_name in self.uav_names:
            self.uavs[uav_name] = self.create_uav_interface(uav_name)

        # 比赛预设参数
        self.load_params()  # 加载参数
        self.takeoff_point = [0.0, 0.0, 1.2, 0.0]  # 起飞点
        rospy.loginfo("multi_uav_state_machine controls UAVs: %s", ", ".join(self.uav_names))

    def load_uav_names(self):
        uav_names = rospy.get_param("~uav_names", None)
        if uav_names:
            if isinstance(uav_names, str):
                names = [name.strip() for name in uav_names.split(",") if name.strip()]
            else:
                names = list(uav_names)
            if not names:
                raise rospy.ROSException("~uav_names cannot be empty.")
            return names

        uav_count = int(rospy.get_param("~uav_count", 2))
        if uav_count < 1:
            raise rospy.ROSException("~uav_count must be at least 1.")
        return [f"uav{i}" for i in range(1, uav_count + 1)]

    def create_uav_interface(self, uav_name):
        uav = UAVState(uav_name)
        # Register the state object before creating subscribers.  A latched or
        # already-active topic may invoke its callback immediately, before this
        # function returns to the caller.
        self.uavs[uav_name] = uav
        uav.mavros_state_sub = rospy.Subscriber(
            f"/{uav_name}/mavros/state",
            State,
            lambda msg, name=uav_name: self.mavros_state_callback(name, msg),
        )
        uav.odom_sub = rospy.Subscriber(
            f"/{uav_name}/drone_odometry",
            Odometry,
            lambda msg, name=uav_name: self.odom_callback(name, msg),
        )
        uav.goal_pub = rospy.Publisher(f"/{uav_name}/goal", PoseStamped, queue_size=10)
        uav.state_pub = rospy.Publisher(f"/{uav_name}/state_machine", Int8, queue_size=10)
        uav.trajectory_sub = rospy.Subscriber(
            f"/{uav_name}/mpc_trajectory",
            Trajectory,
            lambda msg, name=uav_name: self.trajectory_callback(name, msg),
        )
        uav.arming_client = rospy.ServiceProxy(f"/{uav_name}/mavros/cmd/arming", CommandBool)
        uav.set_mode_client = rospy.ServiceProxy(f"/{uav_name}/mavros/set_mode", SetMode)
        return uav

    def odom_callback(self, uav_name, msg):
        self.uavs[uav_name].odom = msg
        self.uavs[uav_name].odom_received = True
        self.uavs[uav_name].last_odom_receive_time = rospy.Time.now()

    def mavros_state_callback(self, uav_name, msg):
        self.uavs[uav_name].mavros_state = msg
        self.uavs[uav_name].mavros_state_received = True

    def trajectory_callback(self, uav_name, msg):
        valid = (
            len(msg.pos) >= 2
            and len(msg.time) == len(msg.pos)
            and all(
                math.isfinite(value)
                for point in msg.pos
                for value in (point.x, point.y, point.z)
            )
        )
        if valid:
            self.uavs[uav_name].trajectory_ready = True

    # 判断当前位置是否接近目标位置
    def is_close(self, odom: Odometry, pose: list, xy, z):
        if not pose:
            return False
        if abs(odom.pose.pose.position.x - pose[0]) < xy and abs(odom.pose.pose.position.y - pose[1]) < xy and abs(odom.pose.pose.position.z - pose[2]) < z:
            return True
        return False
    
    # 设置目标点（通用函数）
    def create_pose(self, pose: list):
        new_pose = PoseStamped()
        new_pose.pose.position.x = pose[0]
        new_pose.pose.position.y = pose[1]
        new_pose.pose.position.z = pose[2]
        angle_rad = math.radians(pose[3])
        roll, pitch, yaw = 0.0, 0.0, angle_rad
        q = quaternion_from_euler(roll, pitch, yaw)
        new_pose.pose.orientation.x = q[0]
        new_pose.pose.orientation.y = q[1]
        new_pose.pose.orientation.z = q[2]
        new_pose.pose.orientation.w = q[3]
        new_pose.header.stamp = rospy.Time.now()
        return new_pose
    
    # 同时给所有无人机发送同一个目标点
    def go_to_all(self, pose: list):
        goal_pose = self.create_pose(pose)
        for uav in self.uavs.values():
            uav.goal_pub.publish(goal_pose)

    # 分别给每架无人机发送不同目标点
    def go_to_each(self, poses_by_uav: dict):
        for uav_name, pose in poses_by_uav.items():
            self.uavs[uav_name].goal_pub.publish(self.create_pose(pose))

    def publish_goal_set(self, goal_key, poses_by_uav):
        """Publish a new target once, retrying slowly until all planners respond."""
        now = rospy.Time.now()
        new_goal = goal_key != self.active_goal_key
        retry_due = now - self.last_goal_publish_time >= rospy.Duration(
            self.goal_retry_period
        )
        waiting_for_trajectory = not self.all_trajectories_ready()
        if not new_goal and not (waiting_for_trajectory and retry_due):
            return

        if new_goal:
            self.active_goal_key = goal_key
            self.trajectories_ready_since = None
            for uav in self.uavs.values():
                uav.trajectory_ready = False

        self.go_to_each(poses_by_uav)
        self.last_goal_publish_time = now
    
    # 发布全局状态到所有无人机
    def publish_global_state(self):
        state_now = Int8()
        state_now.data = self.global_state
        for uav in self.uavs.values():
            uav.state_pub.publish(state_now)
    
    # 检查所有无人机是否都满足条件
    def all_armed(self):
        return all(uav.mavros_state.armed for uav in self.uavs.values())
    
    def all_offboard(self):
        return all(uav.mavros_state.mode == "OFFBOARD" for uav in self.uavs.values())

    def all_inputs_ready(self):
        now = rospy.Time.now()
        return all(
            uav.mavros_state_received
            and uav.mavros_state.connected
            and uav.odom_received
            and now - uav.last_odom_receive_time < rospy.Duration(self.odom_timeout)
            for uav in self.uavs.values()
        )

    def all_trajectories_ready(self):
        return all(uav.trajectory_ready for uav in self.uavs.values())
    
    def all_close(self, pose, xy, z):
        return all(self.is_close(uav.odom, pose, xy, z) for uav in self.uavs.values())

    def all_close_each(self, poses_by_uav, xy, z):
        return all(
            self.is_close(self.uavs[uav_name].odom, pose, xy, z)
            for uav_name, pose in poses_by_uav.items()
        )

    def set_offboard_for_all(self):
        for uav in self.uavs.values():
            if uav.mavros_state.mode == "OFFBOARD":
                continue
            try:
                uav.set_mode_client(0, "OFFBOARD")
            except rospy.ServiceException as exc:
                rospy.logwarn("Failed to set OFFBOARD for %s: %s", uav.name, exc)

    def arm_all(self):
        for uav in self.uavs.values():
            if uav.mavros_state.armed:
                continue
            try:
                uav.arming_client(True)
            except rospy.ServiceException as exc:
                rospy.logwarn("Failed to arm %s: %s", uav.name, exc)
    
    def run(self):
        # 发布当前状态到所有无人机
        self.publish_global_state()

        # 状态0：等待进入OFFBOARD并解锁
        if self.global_state == 0:
            takeoff_points = {
                uav_name: self.takeoff_point for uav_name in self.uav_names
            }
            # Generate and pre-stream a valid takeoff command before PX4 is
            # allowed to enter OFFBOARD or arm.
            self.publish_goal_set("takeoff", takeoff_points)

            if not self.all_inputs_ready():
                rospy.logwarn_throttle(1.0, "Waiting for fresh odometry and MAVROS")
                return

            if not self.all_trajectories_ready():
                rospy.logwarn_throttle(1.0, "Waiting for both takeoff trajectories")
                return

            if self.trajectories_ready_since is None:
                self.trajectories_ready_since = rospy.Time.now()
                rospy.loginfo("Both takeoff trajectories are ready; pre-streaming control")
                return

            if rospy.Time.now() - self.trajectories_ready_since < rospy.Duration(
                self.control_prestream_duration
            ):
                return

            # 检查是否都到达起飞点
            if self.all_offboard() and self.all_armed() and self.all_close(self.takeoff_point, 0.2, 0.2):
                print("所有无人机都已到达起飞点，切换状态1")
                self.global_state = 1
                return
            
            # 切入offboard模式（同时尝试）
            if not self.all_offboard() and rospy.Time.now() - self.last_request > rospy.Duration(1.0):
                rospy.loginfo("Setting mode: OFFBOARD for all UAVs")
                self.set_offboard_for_all()
                self.last_request = rospy.Time.now()
            # 解锁电机（同时尝试）
            elif not self.all_armed() and rospy.Time.now() - self.last_request > rospy.Duration(1.0):
                rospy.loginfo("Arming all drones")
                self.arm_all()
                self.last_request = rospy.Time.now()
            return
            
        elif self.global_state == 1:
            current_points = self.get_current_points()
            self.publish_goal_set(f"point_{self.point_count}", current_points)
            if self.all_close_each(current_points, 0.5, 0.5):
                if self.point_count == self.point_num - 1:
                    self.global_state = 2
                    return
                self.point_count += 1
                print(f"切换到第{self.point_count+1}个点")
                return
            else:
                return
            
        elif self.global_state == 2:
            print("完成飞行")
            return
            
    def load_params(self):
        # 读取先验数据
        for uav_name in self.uav_names:
            self.points[uav_name] = [None] * self.max_points

        for i in range(self.max_points):
            common_point = rospy.get_param(f'~point{i}', None)
            for uav_name in self.uav_names:
                self.points[uav_name][i] = rospy.get_param(f'~point{i}_{uav_name}', common_point)

        for i in range(self.point_num):
            missing_uavs = [name for name in self.uav_names if self.points[name][i] is None]
            if missing_uavs:
                raise rospy.ROSException(
                    f"point{i} is missing for UAVs: {', '.join(missing_uavs)}. "
                    f"Set ~point{i} or ~point{i}_<uav_name>."
                )

    def get_current_points(self):
        return {
            uav_name: self.points[uav_name][self.point_count]
            for uav_name in self.uav_names
        }

if __name__ == "__main__":
    state_machine = StateMachine()
    rate = rospy.Rate(20)  
    while not rospy.is_shutdown():
        state_machine.run()
        rate.sleep()
