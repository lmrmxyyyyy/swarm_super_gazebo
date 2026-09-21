#! /usr/bin/env python3
#coding=utf-8
import rospy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from ius_msgs.msg import Trajectory 
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import Point

class StateMachine:
    def __init__(self):
        rospy.init_node("state_machine")
        self.state = 0
        self.odom = Odometry()
        self.mavros_state = State()

        self.mavros_state_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)
        self.odom_sub = rospy.Subscriber("/mavros/local_position/odom", Odometry, self.odom_callback)
        self.mpc_pub = rospy.Publisher('/mpc_trajectory', Trajectory, queue_size=1)
        self.arming_client = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self.set_mode_client = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        self.takeoff = [[0.0, 0.0, 0.1, 0, 0.0], 
                        [0.0, 0.0, 0.2, 0, 0.1],
                        [0.0, 0.0, 0.3, 0, 0.2], 
                        [0.0, 0.0, 0.4, 0, 0.3],
                        [0.0, 0.0, 0.5, 0, 0.4], 
                        [0.0, 0.0, 0.6, 0, 0.5],
                        [0.0, 0.0, 0.7, 0, 0.6], 
                        [0.0, 0.0, 0.8, 0, 0.7],
                        [0.0, 0.0, 0.9, 0, 0.8], 
                        [0.0, 0.0, 1.0, 0, 1.0]]
        
        self.takeoff_flag = False
        self.trajectory = self.load_txt_to_trajectory("/home/fractal/flm_ws/src/state_machine/config/uav_16.txt")
        self.trajectory_flag = False


    def mavros_state_callback(self, msg):
        self.mavros_state = msg

    def odom_callback(self, msg):
        self.odom = msg

    def is_close(self, odom: Odometry, pose: list, z):
        # print(abs(odom.pose.pose.position.x - pose[0]), abs(odom.pose.pose.position.y - pose[1]), abs(odom.pose.pose.position.z - pose[2]))
        if abs(odom.pose.pose.position.x - pose[0]) < 0.1 and abs(odom.pose.pose.position.y - pose[1]) < 0.1 and abs(odom.pose.pose.position.z - pose[2]) < z:
            return True
        return False
    
    def pub_mpc_traj(self, traj_id, pos, yaw, time_pts):
        msg = Trajectory()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id="map"
        msg.traj_id = traj_id
        msg.pos = pos
        msg.yaw = yaw
        msg.time = time_pts
        self.mpc_pub.publish(msg)

    def pack_trajectory_msgs(self,points_5d):
        """
        输入:
            points_5d: list of [x, y, z, yaw, time]
        输出:
            pos: List[geometry_msgs/Point]
            yaw: List[float]
            time_pts: List[float]
        """
        pos = []
        yaw = []
        time_pts = []

        for pt in points_5d:
            if len(pt) != 5:
                raise ValueError("每个点必须是 [x, y, z, yaw, time] 格式")
            
            point = Point()
            point.x = pt[0]
            point.y = pt[1]
            point.z = pt[2]
            pos.append(point)

            yaw.append(pt[3])
            time_pts.append(pt[4])

        return pos, yaw, time_pts

    def run(self):
        # Arming
        if self.state == 0:
            if self.mavros_state.mode == "OFFBOARD" :
                if self.mavros_state.armed == True :
                    self.state = 1
                    return
                self.arming_client.call(True)
                return
            else:
                print('waiting offboard')
                return
        
        # takeoff    
        elif self.state == 1:
            if not self.takeoff_flag:
                pos, yaw, time_pts = self.pack_trajectory_msgs(self.takeoff)
                self.pub_mpc_traj(traj_id=1, pos=pos, yaw=yaw, time_pts=time_pts)
                self.takeoff_flag = True
                print("发布起飞轨迹")
            else:
                if self.is_close(self.odom, self.takeoff[0], 0.1):
                    self.state = 2
                    print("起飞完成")
                else:
                    print("等待起飞")
            return

        # publish trajectory
        elif self.state == 2:
            if not self.trajectory_flag:
                pos, yaw, time_pts = self.pack_trajectory_msgs(self.trajectory)
                self.pub_mpc_traj(traj_id=1, pos=pos, yaw=yaw, time_pts=time_pts)
                self.trajectory_flag = True
                print("发布无人机固定轨迹")
            else:
                if self.is_close(self.odom, self.trajectory[-1], 0.1):
                    print("执行完成无人机固定轨迹")
                else:
                    print("执行轨迹")
            return
        
    def load_txt_to_trajectory(self,filepath):
        with open(filepath, 'r') as f:
            data = f.read()

        # 把数据拆成数字
        nums = list(map(float, data.strip().split()))
        n = len(nums)
        if n % 3 != 0:
            raise ValueError("数字总数不是3的倍数，无法拆分为x/y/z")

        N = n // 3
        x_list = nums[:N]
        y_list = nums[N:2*N]
        z_list = nums[2*N:]

        trajectory = []
        for i in range(N):
            x = x_list[i]
            y = y_list[i]
            z = z_list[i]
            yaw = 0.0
            t = i * 0.02
            trajectory.append([x, y, z, yaw, t])
        
        return trajectory


if __name__ == "__main__":
    state_machine = StateMachine()
    rate = rospy.Rate(20)  
    while not rospy.is_shutdown():
        state_machine.run()
        rate.sleep()