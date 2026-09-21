#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, PoseStamped, Twist

def pose_publisher():
    model_state_pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1)
    circle_pose_pub = rospy.Publisher('/circle1_real_pose', PoseStamped, queue_size=1)
    rate = rospy.Rate(100)

    # === erweima 参数 ===
    initial_x_erweima = 17.0
    range_erweima = 3.0       # 移动范围
    speed_erweima = 0.4       # 移动速度

    # === circle_1 参数 ===
    initial_x_circle = 5.0
    #initial_x_circle = 27.0
    range_circle = 4.0        # 移动范围
    speed_circle = 0.7        # 移动速度

    t0 = rospy.get_time()

    while not rospy.is_shutdown():
        now = rospy.get_time()
        t = now - t0

        # === erweima 移动逻辑 ===
        cycle_time_e = 2 * range_erweima / speed_erweima
        t_e = t % cycle_time_e
        if t_e <= range_erweima / speed_erweima:
            x_erweima = initial_x_erweima + speed_erweima * t_e
        else:
            x_erweima = initial_x_erweima + speed_erweima * (cycle_time_e - t_e)

        # === circle_1 移动逻辑 ===
        cycle_time_c = 2 * range_circle / speed_circle
        t_c = t % cycle_time_c
        if t_c <= range_circle / speed_circle:
            x_circle = initial_x_circle - speed_circle * t_c
        else:
            x_circle = initial_x_circle - speed_circle * (cycle_time_c - t_c)

        # 发布 erweima 的状态
        erweima_state = ModelState()
        erweima_state.model_name = "erweima"  # 注意属性名是 model_name，不是 name
        erweima_state.pose = Pose()
        erweima_state.pose.position.x = x_erweima
        erweima_state.pose.position.y = 1.0
        erweima_state.pose.position.z = 0.0
        erweima_state.twist = Twist()
        model_state_pub.publish(erweima_state)

        # 发布 circle_1 的状态
        circle_state = ModelState()
        circle_state.model_name = "circle_1"  # 注意属性名是 model_name，不是 name
        circle_state.pose = Pose()
        circle_state.pose.position.x = x_circle
        #circle_state.pose.position.y = 10.0
        circle_state.pose.position.y = 3.0
        circle_state.pose.position.z = 1.0
        circle_state.twist = Twist()
        model_state_pub.publish(circle_state)

        # 发布 circle_1 的真实位置话题（坐标偏移）
        circle_pose_msg = PoseStamped()
        circle_pose_msg.header.stamp = rospy.Time.now()
        circle_pose_msg.header.frame_id = "world"
        circle_pose_msg.pose = Pose()
        circle_pose_msg.pose.position.x = circle_state.pose.position.x
        circle_pose_msg.pose.position.y = circle_state.pose.position.y
        circle_pose_msg.pose.position.z = circle_state.pose.position.z
        circle_pose_pub.publish(circle_pose_msg)

        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('control_move')
    pose_publisher()
