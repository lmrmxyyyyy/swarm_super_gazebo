#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <Eigen/Eigen>
#include<cmath>
 #include <queue>
 #include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
Eigen::Vector3d p_lidar_body, p_enu;  // 存储激光雷达相对机体坐标系下的位置信息和ENU坐标系下的位置信息
Eigen::Quaterniond q_mav;  // 存储来自VINS的四元数姿态信息
Eigen::Quaterniond q_px4_odom;  // 存储来自PX4飞控的四元数姿态信息
Eigen::Quaterniond init_q(1,0,0,0);

// 创建 PoseStamped 消息
    geometry_msgs::PoseStamped pose_msg;
// 创建 PoseWithCovarianceStamped 消息
    geometry_msgs::PoseWithCovarianceStamped pose_cov_msg;
// 创建 TwistWithCovarianceStamped 消息
    geometry_msgs::TwistWithCovarianceStamped twist_cov_msg;

// 滑动窗口平均类定义
 class SlidingWindowAverage {
public:
    SlidingWindowAverage(int windowSize) : windowSize(windowSize), windowSum(0.0) {}
    // 向滑动窗口添加新数据，并返回窗口内数据的平均值
    double addData(double newData) {
        if(!dataQueue.empty()&&fabs(newData-dataQueue.back())>0.01){
            dataQueue = std::queue<double>();
            windowSum = 0.0;
            dataQueue.push(newData);
            windowSum += newData;
        }
        else{            
            dataQueue.push(newData);
            windowSum += newData;
        }

        // 如果队列大小超过窗口大小，弹出队列头部元素并更新窗口和队列和
        if (dataQueue.size() > windowSize) {
            windowSum -= dataQueue.front();
            dataQueue.pop();
        }
        windowAvg = windowSum / dataQueue.size();
        // 返回当前窗口内的平均值
        return windowAvg;
    }
    // 获取队列大小
    int get_size(){
        return dataQueue.size();
    }
    // 获取窗口平均值
    double get_avg(){
        return windowAvg;
    }

private:
    int windowSize;  // 窗口大小
    double windowSum;  // 窗口和
    double windowAvg;  // 窗口平均值
    std::queue<double> dataQueue;  // 数据队列
};

int windowSize = 8;// 滑动窗口大小
SlidingWindowAverage swa=SlidingWindowAverage(windowSize); // 创建滑动窗口平均对象

// 将四元数转换为偏航角
double fromQuaternion2yaw(Eigen::Quaterniond q)
{
  double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
  return yaw;
}

// VINS回调函数，用于获取VINS的位姿信息
void lio_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{

    p_lidar_body = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);

    q_mav = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);

    pose_msg.header = msg->header;
    pose_msg.header.frame_id = "odom"; // 或者其他的参考坐标系名称
    
    pose_cov_msg.header = msg->header;
    pose_cov_msg.header.frame_id = "odom"; // 或者其他的参考坐标系名称
    
    twist_cov_msg.header = msg->header;
    twist_cov_msg.header.frame_id = "odom"; // 或者其他的参考坐标系名称


    p_enu = init_q*p_lidar_body;
    // 填充视觉位姿消息的位置信息
    pose_msg.pose.position.x = p_enu[0];
    pose_msg.pose.position.y = p_enu[1];
    pose_msg.pose.position.z = p_enu[2];
// 填充视觉位姿消息的姿态信息，使用来自VINS的姿态信息
    pose_msg.pose.orientation.x = q_mav.x();
    pose_msg.pose.orientation.y = q_mav.y();
    pose_msg.pose.orientation.z = q_mav.z();
    pose_msg.pose.orientation.w = q_mav.w();
    //vision.header.stamp = ros::Time::now();
    pose_cov_msg.pose = msg->pose; // 包含位置和姿态协方差
    twist_cov_msg.twist = msg->twist; // 包含速度和协方差
    


}

// PX4 odom回调函数，用于获取PX4飞控的位姿信息
void px4_odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    q_px4_odom = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    swa.addData(fromQuaternion2yaw(q_px4_odom));
} 


void pub_external_pose(const ros::Publisher & vision_pub ,const ros::Publisher & pose_cov_pub ,const ros::Publisher & twist_cov_pub)
{
    vision_pub.publish(pose_msg);
    pose_cov_pub.publish(pose_cov_msg);
    // 发布 TwistWithCovarianceStamped 消息
    twist_cov_pub.publish(twist_cov_msg);
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lio_to_mavros");// 初始化ROS节点
    ros::NodeHandle nh("~");

    // 订阅VINS的位姿信息话题
    ros::Subscriber slam_sub = nh.subscribe<nav_msgs::Odometry>("/Odometry", 100, lio_odom_callback);
    ros::Subscriber px4_odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 5, px4_odom_callback);
 
    ros::Publisher vision_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    ros::Publisher pose_cov_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/mavros/vision_pose/pose_cov", 10);
    ros::Publisher twist_cov_pub = nh.advertise<geometry_msgs::TwistWithCovarianceStamped>("/mavros/vision_speed/speed_twist_cov", 10);
 
    // the setpoint publishing rate MUST be faster than 2Hz
    // 设置发布速率
    ros::Rate rate(30.0);
 
    ros::Time last_request = ros::Time::now();
    float init_yaw = 0.0;
    bool init_flag = 0;
    
    

    while(ros::ok()){
        // if(swa.get_size()==windowSize&&!init_flag){ // 如果滑动窗口内数据已经达到窗口大小，并且未进行初始化
        //     init_yaw = swa.get_avg();
        //     init_flag = 1;
        //     // 使用初始偏航角初始化四元数，其他轴角度为0
        //     init_q = Eigen::AngleAxisd(init_yaw,Eigen::Vector3d::UnitZ())//des.yaw// 使用初始偏航角初始化四元数
        //                 * Eigen::AngleAxisd(0.0,Eigen::Vector3d::UnitY())
        //                 * Eigen::AngleAxisd(0.0,Eigen::Vector3d::UnitX());
        // // delete swa;
        // }

        if(1){
            pub_external_pose( vision_pub ,pose_cov_pub , twist_cov_pub);
            ROS_INFO("\nposition in enu:\n   x: %.18f\n   y: %.18f\n   z: %.18f\norientation of lidar:\n   x: %.18f\n   y: %.18f\n   z: %.18f\n   w: %.18f", \
            p_enu[0],p_enu[1],p_enu[2],q_mav.x(),q_mav.y(),q_mav.z(),q_mav.w());
        }
        nav_msgs::Odometry odomAftMappedd;
        odomAftMappedd.header.frame_id = "camera_init";
        odomAftMappedd.child_frame_id = "world";
        odomAftMappedd.header.stamp = ros::Time::now();
        odomAftMappedd.pose.pose.position.x = 0;
        odomAftMappedd.pose.pose.position.y = 0;
        odomAftMappedd.pose.pose.position.z = 0;
        odomAftMappedd.pose.pose.orientation.x = 0;
        odomAftMappedd.pose.pose.orientation.y = 0;
        odomAftMappedd.pose.pose.orientation.z = 0;
        odomAftMappedd.pose.pose.orientation.w = 1; 
        static tf::TransformBroadcaster brr, brrr,brrrr;
        tf::Transform                   transformm;
        tf::Quaternion                  qq;
        transformm.setOrigin(tf::Vector3(0,0,0));
        qq.setW(1);
        qq.setX(0);
        qq.setY(0);
        qq.setZ(0);
        transformm.setRotation( qq );
        brr.sendTransform( tf::StampedTransform( transformm, odomAftMappedd.header.stamp, "camera_init", "world" ) );

        odomAftMappedd.header.frame_id = "world";
        odomAftMappedd.child_frame_id = "odom";
        odomAftMappedd.header.stamp = ros::Time::now();
        brrr.sendTransform( tf::StampedTransform( transformm, odomAftMappedd.header.stamp, "world", "odom" ) );

        odomAftMappedd.header.frame_id = "world";
        odomAftMappedd.child_frame_id = "map";
        odomAftMappedd.header.stamp = ros::Time::now();
        brrrr.sendTransform( tf::StampedTransform( transformm, odomAftMappedd.header.stamp, "world", "map" ) );

 
        ros::spinOnce();
        rate.sleep();
    }
 
    return 0;
}
