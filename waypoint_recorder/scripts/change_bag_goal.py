#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class GoalPublisher:
    def __init__(self):
        rospy.init_node('goal_from_path_node')
        
        self.sub = rospy.Subscriber('/waypoint_generator/waypoints', Path, self.path_callback)
        self.pub = rospy.Publisher('/goal', PoseStamped, queue_size=10)

    def path_callback(self, path_msg):
        if not path_msg.poses:
            rospy.logwarn("Received empty path.")
            return

        # 获取最后一个点
        last_pose = path_msg.poses[-1]

        # 设置时间戳为当前时间
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = last_pose.header.frame_id
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id="camera_init"
        goal_pose.pose = last_pose.pose

        # 发布到 /goal
        self.pub.publish(goal_pose)
        rospy.loginfo("Published goal to /goal.")

if __name__ == '__main__':
    try:
        GoalPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
