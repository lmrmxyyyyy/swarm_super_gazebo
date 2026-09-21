#!/usr/bin/env python3
# coding=utf-8

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from pupil_apriltags import Detector
from scipy.spatial.transform import Rotation as R


class AprilTagPoseEstimator:
    def __init__(self, camera_matrix, tag_size, real_fly, is_debug=False):
        # 相机参数和标签大小
        self.fx = camera_matrix[0, 0]
        self.fy = camera_matrix[1, 1]
        self.cx = camera_matrix[0, 2]
        self.cy = camera_matrix[1, 2]
        self.tag_size = tag_size
        self.camera_matrix = camera_matrix
        self.dist_coeffs = np.zeros((4, 1))  # 假设无畸变，如有畸变请填写

        self.debug = is_debug
        self.ros_pose = PoseStamped()

        # AprilTag 检测器
        self.detector = Detector(families='tag36h11')
        self.need_det_id=1

        # EKF 初始化
        self.state = np.zeros((6, 1))  # [x, y, z, vx, vy, vz]
        self.P = np.eye(6) * 0.1
        self.Q = np.eye(6) * 0.05
        self.R = np.eye(3) * 0.001
        self.dt = 1.0 / 15.0
        self.last_time = None  # 初始化
        
        # RDF -> FLU 坐标变换矩阵
        if(real_fly):
            usb_cam_topic = "/usb_cam/image_raw"
            self.T_cam_to_drone = np.array([
            [0, -1, 0],   # x_cam -> y_drone
            [-1, 0, 0],   # y_cam -> x_drone
            [0, -0, -1]   # z_cam -> -z_drone
            ])
            self.off_cam_to_drone =np.array([0, 0, 0])
        else:
            usb_cam_topic = "/neverlost/usb_cam/image_raw"
            self.T_cam_to_drone = np.array([
            [0, -1, 0],   # x_cam -> y_drone
            [-1, 0, 0],   # y_cam -> x_drone
            [0, -0, -1]   # z_cam -> -z_drone
            ])
            self.off_cam_to_drone =np.array([0, 0, 0])

        # ROS 初始化
        self.bridge = CvBridge()
        self.pose_pub = rospy.Publisher("/apriltag/pose", PoseStamped, queue_size=1)
        self.debug_pub = rospy.Publisher("/apriltag/debug_img", Image, queue_size=1)
        rospy.Subscriber(usb_cam_topic, Image, self.image_callback)
        # self.latest_frame = cv2.imread("/home/fractal/图片/2025-06-19 14-40-44 的屏幕截图.png")
        self.latest_frame = None
        self.can_see = False

        self.timer = rospy.Timer(rospy.Duration(self.dt), self.timer_callback)  # 15Hz 固定检测
        rospy.loginfo("AprilTagPoseEstimator initialized and listening to /usb_cam/image_raw")

    def predict(self):
        now = rospy.Time.now().to_sec()
        if self.last_time is None:
            self.last_time = now
        dt = now - self.last_time
        self.last_time = now
        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt
        print("dt  ",dt)
        self.state = F @ self.state
        self.P = F @ self.P @ F.T + self.Q

    def update(self, measurement):
        H = np.zeros((3, 6))
        H[0, 0] = 1
        H[1, 1] = 1
        H[2, 2] = 1
        z = measurement.reshape(3, 1)
        y = z - H @ self.state
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.state = self.state + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

    def detect_and_estimate(self, gray_img, vis_img=None):
        detections = self.detector.detect(
            gray_img,
            estimate_tag_pose=True,
            camera_params=[self.fx, self.fy, self.cx, self.cy],
            tag_size=self.tag_size
        )

        if len(detections) == 0:
            return None, vis_img

        det = detections[0]  # 取第一个
        for det in detections:
            tag_id = det.tag_id  # 标签编号
            if tag_id ==self.need_det_id:
                self.can_see=True
                tvec = det.pose_t.reshape(3, 1)  # 相机系位置
                rmat = det.pose_R                # 相机系方向
                # 转换到无人机坐标系
                tvec_drone = (self.T_cam_to_drone @ tvec).reshape(3, 1)+self.off_cam_to_drone.reshape(3, 1)
                break
        if  not self.can_see:
            return None, vis_img
        # rmat_drone = self.T_cam_to_drone @ rmat

        self.predict()
        self.update(tvec_drone)

        pose = {
            'id': tag_id,
            'position': self.state[:3].ravel(),
            # 'rotation_matrix': rmat,
            # 'quaternion': R.from_matrix(rmat).as_quat()
        }

        print(f"Detected tag ID: {tag_id}")
        print(f"Filtered Position: {pose['position']}")

        # 如果可视化，仅绘制角点
        if self.debug and vis_img is not None:
            for det_i in detections:
                color_need=(0, 0, 255)
                if det_i.tag_id ==self.need_det_id:
                    color_need=(0, 255, 0)
                corners = det_i.corners.astype(np.float32).reshape(-1, 1, 2)
                cv2.polylines(vis_img, [corners.astype(np.int32)], True, color_need, 2)
                # 在角点中心显示ID
                center = np.mean(det_i.corners, axis=0).astype(int)
                cv2.putText(vis_img, f"ID:{det_i.tag_id}", tuple(center), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color_need, 2)

        return pose, vis_img

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # height, width, _ = self.latest_frame.shape
            # rospy.loginfo(f"Received image of size: width={width}, height={height}")
        except Exception as e:
            rospy.logerr(f"Image conversion failed: {e}")
            
    def timer_callback(self, event):
        if self.latest_frame is None and not self.can_see:
            return
        # print("11")
        frame = self.latest_frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.can_see=False
        pose, vis_img = self.detect_and_estimate(gray, frame)

        if  not self.can_see:
            return

        if pose:
            self.ros_pose.header.stamp = rospy.Time.now()
            self.ros_pose.header.frame_id = "camera"
            self.ros_pose.pose.position.x = pose['position'][0]
            self.ros_pose.pose.position.y = pose['position'][1]
            self.ros_pose.pose.position.z = pose['position'][2]
            self.pose_pub.publish(self.ros_pose)
            
        if self.debug and vis_img is not None:
            print("22")
            debug_msg = self.bridge.cv2_to_imgmsg(vis_img, encoding="bgr8")
            self.debug_pub.publish(debug_msg)


if __name__ == "__main__":
    rospy.init_node("apriltag_ekf_node")
    camera_matrix = np.array([
        [355.092415190087, 0, 299.727990720535],
        [0, 356.601678448605, 240.266512432898],
        [0, 0, 1]
    ])
    tag_size = 0.147  # meters

    # debug = rospy.get_param("~debug", True)

    estimator = AprilTagPoseEstimator(camera_matrix, tag_size, is_debug=True,real_fly=True)
    rospy.spin()
