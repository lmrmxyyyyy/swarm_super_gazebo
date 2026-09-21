#!/usr/bin/env python3
# coding=utf-8 
import rospy
import time
import math
from std_msgs.msg import Int8, Bool
from nav_msgs.msg import Odometry
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped, Point
from tf.transformations import quaternion_from_euler, euler_from_quaternion

class StateMachine:
    def __init__(self):
        rospy.init_node("state_machine")

        # 状态变量
        self.odom = Odometry()
        self.mavros_state = State()
        self.state = 0
        self.last_request = rospy.Time.now()
        self.point = [None] * 15 
        self.point_count = 0
        self.pose = PoseStamped()
        self.point_num = 1
    
        # 订阅器
        self.mavros_state_sub = rospy.Subscriber("~mavros/state", State, self.mavros_state_callback)
        self.odom_sub = rospy.Subscriber("~drone_odometry", Odometry, self.odom_callback)

        # 发布器
        self.goal_pub = rospy.Publisher('~goal', PoseStamped, queue_size=10)
        self.state_machine_state_pub = rospy.Publisher("~state_machine", Int8, queue_size=10)

        # 服务客户端
        self.arming_client = rospy.ServiceProxy("~mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("~mavros/set_mode", SetMode)
    
        # 比赛预设参数
        self.load_params()  # 加载参数
        self.takeoff_point = [0.0, 0.0, 1.2, 0.0]  # 起飞点

    def odom_callback(self, msg):
        self.odom = msg

    def mavros_state_callback(self, msg):
        self.mavros_state = msg
        # rospy.loginfo_throttle(1.0, f"Connected: {msg.connected}, Mode: {msg.mode}, Armed: {msg.armed}")

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

    
    def run(self):
        # 发布当前状态
        state_now = Int8()
        state_now.data = self.state
        self.state_machine_state_pub.publish(state_now)

        # 状态0：等待进入OFFBOARD并解锁
        if self.state == 0:
            # '''切换到穿越圆环1'''
            if self.is_close(self.odom, self.takeoff_point, 0.2, 0.2):
                print("已到达起飞点，切换状态1")
                self.state = 1  #########
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
            if self.is_close(self.odom, self.point[self.point_count],0.5,0.5):
                if self.point_count == self.point_num-1:
                    self.state = 2 #####
                    #print(self.point_count)
                    return
                # time.sleep(1)
                self.state = 1
                self.point_count+=1
                return
            else:
                self.go_to(self.point[self.point_count])
                print(f'飞往第{self.point_count+1}个点')
                print(self.point[self.point_count])
                return
            
        elif self.state == 2:
            print("完成飞行")
            return
        
            
    def load_params(self):
        # 读取先验数据
        self.point[0] = rospy.get_param('~point0')
        self.point[1] = rospy.get_param('~point1')
        self.point[2] = rospy.get_param('~point2')
        self.point[3] = rospy.get_param('~point3')
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