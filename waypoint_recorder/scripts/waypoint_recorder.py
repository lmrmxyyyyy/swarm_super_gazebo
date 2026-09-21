#! /usr/bin/env python3
#coding=utf-8


import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
import time
from std_msgs.msg import Int8
import yaml
import os

class WaypointMem:
    def __init__(self):
        rospy.init_node("arcuo_det")
        self.odom = Odometry()
        self.point_num = 10
        self.debug_jump_to = 0
        self.points = {}
        self.state = 0
        self.land_flag = 0
        # self.odom_sub = rospy.Subscriber("/Odometry", Odometry, self.odom_callback)
        self.odom_sub = rospy.Subscriber("/airsim_node/drone_1/debug/pose_gt", PoseStamped, self.odom_callback)

        self.filename = os.path.expanduser('~/IntelligentUAVChampionshipBase/flm_dev/src/waypoint_recorder/waypoints.yaml')
        self.fin=0
        # 删除之前的文件（如果存在）
        if os.path.exists(self.filename):
            os.remove(self.filename)
    def odom_callback(self, msg):
        self.odom = msg

    def record_point(self, name):
        # position = self.odom.pose.pose.position
        position = self.odom.pose.position
        point = [round(position.x, 2), round(position.y, 2), round(position.z, 2)]#z自己定义
        self.points[name] = point
        rospy.loginfo(f"Recorded point {name}: {point}")
        self.save_to_yaml(name, point)

    def save_to_yaml(self, name, point):
        if os.path.exists(self.filename):
            with open(self.filename, 'r') as yaml_file:
                data = yaml.safe_load(yaml_file) or {}
        else:
            data = {}

        data[name] = point

        with open(self.filename, 'w') as yaml_file:
            yaml.dump(data, yaml_file, default_flow_style=True)
        rospy.loginfo(f"Point {name} saved to {self.filename}")

    def run(self):
        while not rospy.is_shutdown():
            print("\n\n-----------------------------------------------------------------------------")
            rospy.loginfo("输入 Name+回车 to record a point or type 'exit' to save and exit.")
            user_input = input()
            if user_input.lower() == 'exit':
                self.fin=1#结束
                rospy.loginfo("Exiting...")
                break
            else:
                self.record_point(user_input)

if __name__ == "__main__":
    waypoint_mem = WaypointMem()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        waypoint_mem.run()
        if waypoint_mem.fin:
            break
        rate.sleep()
