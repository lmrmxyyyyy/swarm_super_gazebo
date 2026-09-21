/**
* This file is part of ROG-Map
*
* Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/ROG-Map>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* ROG-Map is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ROG-Map is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with ROG-Map. If not, see <http://www.gnu.org/licenses/>.
*/
#define USE_ROS1///////////
#ifndef USE_ROS1
#ifndef USE_ROS2
#error "Please define either USE_ROS1 or USE_ROS2, but not both."
#endif
#endif

#ifdef USE_ROS1
#ifdef USE_ROS2
#error "Cannot use both USE_ROS1 and USE_ROS2 at the same time. Please define only one."
#endif
#endif


#ifdef USE_ROS1
#ifndef ROG_MAP_ROS_HPP
#define ROG_MAP_ROS_HPP
#include <rog_map/rog_map.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <super_utils/color_msg_utils.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>  // Include for pcl::transformPointCloud
#include "std_msgs/Int8.h"
#include <deque>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>

namespace rog_map {
    using namespace super_utils;

    class ROGMapROS :public ROGMap {
        ros::NodeHandle nh_;
        std::deque<nav_msgs::Odometry> odom_queue;
        const int max_odom_queue_size = 20;

        const double getSystemWalltimeNow() override {
            return ros::Time::now().toSec();
        }

        struct VisualizeMap {
            ros::Publisher occ_pub, unknown_pub,
                           occ_inf_pub, unknown_inf_pub,
                           mkr_arr_pub, frontier_pub,
                           esdf_pub, esdf_neg_pub, esdf_occ_pub;
            ros::Timer viz_timer;
        } vm_;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>
                SyncPolicyImageOdom;
        struct ROSCallback {
            ros::Subscriber odom_sub, cloud_sub,cloud_f_sub,state_sub;
            // message_filters::Subscriber<sensor_msgs::Image> depth_sub;
            // message_filters::Subscriber<nav_msgs::Odometry> odom_depth_sub;
            
            shared_ptr<message_filters::Synchronizer<SyncPolicyImageOdom>> sync_image_odom_;
            int unfinished_frame_cnt{0};
            Pose pc_pose;
            PointCloud pc;
            ros::Timer update_timer,occ_timer_;
            mutex updete_lock;
        } rc_;
        struct MappingData {
            // main map data, occupancy of each voxel and Euclidean distance

            std::vector<double> occupancy_buffer_;
            std::vector<char> occupancy_buffer_inflate_;

            // camera position and pose data

            Eigen::Vector3d camera_pos_, last_camera_pos_;
            Eigen::Quaterniond camera_q_, last_camera_q_;

            // depth image data

            cv::Mat depth_image_, last_depth_image_;
            int image_cnt_;

            Eigen::Matrix4d cam2body_;

            // flags of map state

            bool occ_need_update_, local_updated_;
            bool has_first_depth_;
            bool has_odom_, has_cloud_;

            // depth image projected point cloud

            vector<Eigen::Vector3d> proj_points_;
            int proj_points_cnt;

            // flag buffers for speeding up raycasting

            vector<short> count_hit_, count_hit_and_miss_;
            vector<char> flag_traverse_, flag_rayend_;
            char raycast_num_;
            queue<Eigen::Vector3i> cache_voxel_;

            // range of updating grid

            Eigen::Vector3i local_bound_min_, local_bound_max_;

            // computation time

            double fuse_time_, max_fuse_time_;
            int update_num_;
            double k_depth_scaling_factor_;
        }md_;

        std_msgs::Int8 state,state_last;

        void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg) {
            updateRobotState(std::make_pair(
                Vec3f(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
                      odom_msg->pose.pose.position.z),
                Quatf(odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
                      odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z)));
            // 将新的 odom 数据加入队列
            odom_queue.push_back(*odom_msg);

            // 如果队列大小超过最大限制，移除最早的帧
            if (odom_queue.size() > max_odom_queue_size) {
                odom_queue.pop_front();
            }

            static tf2_ros::TransformBroadcaster br_map_ego;
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.frame_id = "world";
            transformStamped.child_frame_id = "drone";
            transformStamped.transform.translation.x = odom_msg->pose.pose.position.x;
            transformStamped.transform.translation.y = odom_msg->pose.pose.position.y;
            transformStamped.transform.translation.z = odom_msg->pose.pose.position.z;
            transformStamped.transform.rotation.x = odom_msg->pose.pose.orientation.x;
            transformStamped.transform.rotation.y = odom_msg->pose.pose.orientation.y;
            transformStamped.transform.rotation.z = odom_msg->pose.pose.orientation.z;
            transformStamped.transform.rotation.w = odom_msg->pose.pose.orientation.w;
            br_map_ego.sendTransform(transformStamped);
            
            static tf2_ros::TransformBroadcaster br_map_body;
            geometry_msgs::TransformStamped transformStamped_body;
            transformStamped_body.header.stamp = ros::Time::now();
            transformStamped_body.header.frame_id = "world";
            transformStamped_body.child_frame_id = "body";
            transformStamped_body.transform.translation.x = odom_msg->pose.pose.position.x;
            transformStamped_body.transform.translation.y = odom_msg->pose.pose.position.y;
            transformStamped_body.transform.translation.z = odom_msg->pose.pose.position.z;
            transformStamped_body.transform.rotation.x = odom_msg->pose.pose.orientation.x;
            transformStamped_body.transform.rotation.y = odom_msg->pose.pose.orientation.y;
            transformStamped_body.transform.rotation.z = odom_msg->pose.pose.orientation.z;
            transformStamped_body.transform.rotation.w = odom_msg->pose.pose.orientation.w;
            br_map_body.sendTransform(transformStamped_body);

            // static tf2_ros::TransformBroadcaster br_vis_camera_init;
            // geometry_msgs::TransformStamped transform_camera_init;
            // transform_camera_init.header.stamp = ros::Time::now();
            // transform_camera_init.header.frame_id = "world";
            // transform_camera_init.child_frame_id = "camera_init";
            // transform_camera_init.transform.translation.x = 0;
            // transform_camera_init.transform.translation.y = 0;
            // transform_camera_init.transform.translation.z = 0;
            // transform_camera_init.transform.rotation.x = 0;
            // transform_camera_init.transform.rotation.y = 0;
            // transform_camera_init.transform.rotation.z = 0;
            // transform_camera_init.transform.rotation.w = 1;
            // br_vis_camera_init.sendTransform(transform_camera_init);
            
            // static tf2_ros::TransformBroadcaster br_world_map;
            // geometry_msgs::TransformStamped transform_world_map;
            // transform_world_map.header.stamp = ros::Time::now();
            // transform_world_map.header.frame_id = "world";
            // transform_world_map.child_frame_id = "map";
            // transform_world_map.transform.translation.x = 0;
            // transform_world_map.transform.translation.y = 0;
            // transform_world_map.transform.translation.z = 0;
            // transform_world_map.transform.rotation.x = 0;
            // transform_world_map.transform.rotation.y = 0;
            // transform_world_map.transform.rotation.z = 0;
            // transform_world_map.transform.rotation.w = 1;
            // br_world_map.sendTransform(transform_world_map);

            // static tf2_ros::TransformBroadcaster br_vis_world;
            // geometry_msgs::TransformStamped transform_robomaster;
            // transform_robomaster.header.stamp = ros::Time::now();
            // transform_robomaster.header.frame_id = "world";
            // transform_robomaster.child_frame_id = "world_vis_robomaster";
            // transform_robomaster.transform.translation.x = 0;
            // transform_robomaster.transform.translation.y = 0;
            // transform_robomaster.transform.translation.z = 0;
            // transform_robomaster.transform.rotation.x = 1;
            // transform_robomaster.transform.rotation.y = 0;
            // transform_robomaster.transform.rotation.z = 0;
            // transform_robomaster.transform.rotation.w = 0;
            // br_vis_world.sendTransform(transform_robomaster);

        }
        // void depthOdomCallback(const sensor_msgs::ImageConstPtr &img, // from ego
        //                         const nav_msgs::OdometryConstPtr &odom)
        // {
        //     // std::cout << YELLOW << " -- [ROS] Odom depthOdomCallback callback.!!!!!!" << RESET << std::endl;
        //     /* get pose */
        //     Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
        //                                                     odom->pose.pose.orientation.x,
        //                                                     odom->pose.pose.orientation.y,
        //                                                     odom->pose.pose.orientation.z);    
        //     Eigen::Matrix3d body_r_m = body_q.toRotationMatrix();   
        //     Eigen::Matrix4d body2world;
        //     body2world.block<3, 3>(0, 0) = body_r_m;
        //     body2world(0, 3) = odom->pose.pose.position.x;
        //     body2world(1, 3) = odom->pose.pose.position.y;
        //     body2world(2, 3) = odom->pose.pose.position.z;
        //     body2world(3, 3) = 1.0;

            
        //     md_.cam2body_ << 0.0, 0.0, 1.0, 0.1,
        //                     1.0, 0.0, 0.0, 0.0,
        //                     0.0, 1.0, 0.0, 0.0,
        //                     0.0, 0.0, 0.0, 1.0;
            
        //     Eigen::Matrix4d cam_T = body2world * md_.cam2body_;;
        //     md_.camera_pos_(0) = cam_T(0, 3);
        //     md_.camera_pos_(1) = cam_T(1, 3);
        //     md_.camera_pos_(2) = cam_T(2, 3);
        //     md_.camera_q_ = Eigen::Quaterniond(cam_T.block<3, 3>(0, 0));

        //     /* get depth image */
        //     cv_bridge::CvImagePtr cv_ptr;
        //     cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
        //     md_.k_depth_scaling_factor_=1000.0;
        //     if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
        //     {
        //         (cv_ptr->image).convertTo(cv_ptr->image, CV_16UC1, md_.k_depth_scaling_factor_);
        //     }
        //     // 使用 rotate 进行 180° 旋转
        //     // cv::Mat rotated_image;
        //     // cv::rotate(cv_ptr->image, rotated_image, cv::ROTATE_180);
        //     // rotated_image.copyTo(md_.depth_image_);
        //     cv_ptr->image.copyTo(md_.depth_image_);

        //     md_.occ_need_update_ = true;
        //     // std::cout << YELLOW << " -- aaaaaaaaaaaaaaaaaaaaaaaaaaaarc_.." << RESET << std::endl;
        // }
        
        // void projectDepthImage(const ros::TimerEvent & /*event*/) //
        // {
        //     if (!md_.occ_need_update_)
        //         return;
        //     if (!robot_state_.rcv) {
        //         return;
        //     }
        //     double cbk_t = ros::Time::now().toSec();
        //     if (cbk_t - robot_state_.rcv_time > cfg_.odom_timeout) {
        //         std::cout << YELLOW << " -- [ROS] Odom timeout, skip depth callback." << RESET << std::endl;
        //         return;
        //     }
        //     // md_.proj_points_.clear();
        //     md_.proj_points_cnt = 0;

        //     uint16_t *row_ptr;
        //     // int cols = current_img_.cols, rows = current_img_.rows;
        //     int cols = md_.depth_image_.cols;
        //     int rows = md_.depth_image_.rows;

        //     double depth;

        //     Eigen::Matrix3d camera_r = md_.camera_q_.toRotationMatrix();

        //     // cout << "rotate: " << md_.camera_q_.toRotationMatrix() << endl;
        //     // std::cout << "pos in proj: " << md_.camera_pos_ << std::endl;
        //     //过边缘的部分（根据 depth_filter_margin_），并且每隔一定的像素（根据 skip_pixel_）处理一个像素。
        //     md_.k_depth_scaling_factor_=1000.0;
        //     int depth_filter_margin_=1;
        //     int skip_pixel_=3;
        //     double depth_filter_mindist_=0.1;
        //     double depth_filter_maxdist_=4.5;

        //     // 实机上的深度相机参数
        //     // double cx_=327.6851806640625;
        //     // double cy_=235.88491821289062;
        //     // double fx_=390.87896728515625;
        //     // double fy_=390.87896728515625;   1
           
        //     // double cx_=317.986328125;
        //     // double cy_=239.7025604248047;
        //     // double fx_=387.89385986328125;
        //     // double fy_=387.89385986328125; //2 
        //     double cx_= 320.4765625 ;
        //     double cy_= 238.01258850097656 ;
        //     double fx_= 390.60772705078125 ;
        //     double fy_= 390.60772705078125 ;  //3
            
        //     // 仿真中的深度相机参数
        //     // double cx_=320.5;
        //     // double cy_=240.5;
        //     // double fx_=554.254691191187;
        //     // double fy_=554.254691191187;

        //     /* use depth filter */
           
        //     if (!md_.has_first_depth_)
        //     md_.has_first_depth_ = true;
        //     else
        //     {
        //         PointCloud tmp_pc;
        //         Eigen::Vector3d pt_cur, pt_world, pt_reproj;

        //         Eigen::Matrix3d last_camera_r_inv;
        //         last_camera_r_inv = md_.last_camera_q_.inverse();
        //         const double inv_factor = 1.0 / md_.k_depth_scaling_factor_;

        //         for (int v = depth_filter_margin_; v < rows - depth_filter_margin_; v += skip_pixel_)
        //         {
        //             row_ptr = md_.depth_image_.ptr<uint16_t>(v) + depth_filter_margin_;

        //             for (int u = depth_filter_margin_; u < cols - depth_filter_margin_; u += skip_pixel_)
        //             {

        //             depth = (*row_ptr) * inv_factor;
        //             row_ptr = row_ptr + skip_pixel_;

        //             // filter depth
        //             // depth += rand_noise_(eng_);
        //             // if (depth > 0.01) depth += rand_noise2_(eng_);
        //             //// depth过滤
        //             if (*row_ptr == 0 )
        //             {
        //                 // depth = cfg_.raycast_range_max+ 0.1; 
        //                 continue;
        //             }
        //             else if (depth < depth_filter_mindist_)
        //             {
        //                 continue;
        //             }
        //             else if (depth > depth_filter_maxdist_)
        //             {
        //                 // continue;
        //                 depth = cfg_.raycast_range_max+ 0.1;
        //             }
        //             // else if (depth > depth_filter_maxdist_)
        //             // {
        //             //     // continue;
        //             //     depth = cfg_.raycast_range_max+ 0.1;
        //             // }

        //             // project to world frame
        //             pt_cur(0) = (u - cx_) * depth / fx_;
        //             pt_cur(1) = (v - cy_) * depth / fy_;
        //             pt_cur(2) = depth;

        //             pt_world = camera_r * pt_cur + md_.camera_pos_;
        //             // if (!isInMap(pt_world)) {
        //             //   pt_world = closetPointInMap(pt_world, md_.camera_pos_);
        //             // }
        //             // 将有效的3D点加入到点云中
        //             pcl::PointXYZI point;
        //             point.x = pt_world(0);  // x坐标
        //             point.y = pt_world(1);  // y坐标
        //             point.z = pt_world(2);  // z坐标
        //             point.intensity = 0;  // 这里我们可以使用深度值作为点的强度

        //             tmp_pc.push_back(point);  // 将点加入点云

                   
                    
        //             }
        //         }
        //         // std::cout << YELLOW << " -- bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb_.." << RESET << std::endl;
        //         rc_.updete_lock.lock();
        //         rc_.pc = tmp_pc;
        //         rc_.pc_pose = std::make_pair(robot_state_.p, robot_state_.q);
        //         rc_.unfinished_frame_cnt++;
        //         map_empty_ = false;
        //         rc_.updete_lock.unlock();
        //     }
            

        //     /* maintain camera pose for consistency check */

        //     md_.last_camera_pos_ = md_.camera_pos_;
        //     md_.last_camera_q_ = md_.camera_q_;
        //     md_.last_depth_image_ = md_.depth_image_;
        //     md_.occ_need_update_ = false;
            
        // }

        void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {//接收点云
            if (!robot_state_.rcv) {
                return;
            }
            double cbk_t = ros::Time::now().toSec();
            if (cbk_t - robot_state_.rcv_time > cfg_.odom_timeout) {
                std::cout << YELLOW << " -- [ROS] Odom timeout, skip cloud callback." << RESET << std::endl;
                return;
            }
            // if(state.data==26||state.data==27||state.data==23||state.data==28)
            // {
            //         return;
            // }
            // else if(state.data==36||state.data==37||state.data==33||state.data==38)
            // {
            //     return;
            // }
            // else if(state.data>=8 && state.data<=14)
            // {
            //     return;
            // }
            // else if(state.data==3)
            // {
            //     return;
            // }
             // 将ROS消息转换为PCL点云
             // 遍历 odom 队列，找到与 LiDAR 时间戳最接近的 odom
            double min_diff = 1000;
            nav_msgs::Odometry closest_odom;
            for (const auto& odom : odom_queue) {
                double time_diff = std::abs(cloud_msg->header.stamp.toSec() - odom.header.stamp.toSec());
                if (time_diff < min_diff) {
                    min_diff = time_diff;
                    closest_odom = odom;
                }
            }
            PointCloud tmp_pc;
            pcl::fromROSMsg(*cloud_msg, tmp_pc);

            if (min_diff < cfg_.odom_timeout) {
             // 获取无人机的位置和姿态
            // 获取无人机的位置和姿态
            // Eigen::Vector3d drone_position(closest_odom.pose.pose.position.x,
            //                            closest_odom.pose.pose.position.y,
            //                            closest_odom.pose.pose.position.z);
            // Eigen::Quaterniond drone_orientation(closest_odom.pose.pose.orientation.w,
            //                                   closest_odom.pose.pose.orientation.x,
            //                                   closest_odom.pose.pose.orientation.y,
            //                                   closest_odom.pose.pose.orientation.z);
            Eigen::Vector3d drone_position(0,
                                       0,
                                       0);
            Eigen::Quaterniond drone_orientation(1,
                                              0,
                                              0,
                                              0);
            // LiDAR 相对于无人机的偏移量，假设 LiDAR 位于无人机上方 10cm
            Eigen::Vector3d lidar_offset(0, 0, 0.1);  // Z轴上方 10 cm
            // 计算 LiDAR 在世界坐标系中的位置
            Eigen::Vector3d lidar_position_world = drone_orientation * lidar_offset + drone_position;
            // 创建变换矩阵：旋转 + 平移
            Eigen::Affine3d lidar_to_world_transform = Eigen::Affine3d::Identity();
            lidar_to_world_transform.translation() = lidar_position_world;  // 平移
            lidar_to_world_transform.linear() = drone_orientation.toRotationMatrix();  // 旋转

            // 使用 pcl::transformPointCloud 将点云从 LiDAR 坐标系转换到世界坐标系
            pcl::transformPointCloud(tmp_pc, tmp_pc, lidar_to_world_transform);


            rc_.updete_lock.lock();
            rc_.pc = tmp_pc;
            rc_.pc_pose = std::make_pair(robot_state_.p, robot_state_.q);
            rc_.unfinished_frame_cnt++;
            map_empty_ = false;
            rc_.updete_lock.unlock();
            }
        }

        void stateMachineCallback(const std_msgs::Int8::ConstPtr& msg)
        {
            state = *msg;
            ROS_INFO("ROG Map Received state machine state: [%d]", state.data);
            // if (state.data==3||state.data==103||state.data==4||state.data==5)
            // {
            //     cfg_.point_filt_num=50;
            // }
            // else if(state.data==77)
            // {
            //     cfg_.point_filt_num=10;
            // }
            // else 
            // {
            //     cfg_.point_filt_num=cfg_.point_filt_num_init;
            // }
        }
        // void cloudCallback_f(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {//接收点云
        //     if (!robot_state_.rcv) {
        //         return;
        //     }
        //     if(!(state.data==7||state.data==2))
        //     {
        //         return;
        //     }
        //      if(true) //不用
        //     {
        //         return;
        //     }

        //     double cbk_t = ros::Time::now().toSec();
        //     if (cbk_t - robot_state_.rcv_time > cfg_.odom_timeout) {
        //         std::cout << YELLOW << " -- [ROS] Odom timeout, skip cloud callback." << RESET << std::endl;
        //         return;
        //     }
        //      // 将ROS消息转换为PCL点云
        //     PointCloud tmp_pc;
        //     pcl::fromROSMsg(*cloud_msg, tmp_pc);

        //     //  // 获取无人机的位置和姿态
        //     // Eigen::Vector3d drone_position(robot_state_.p.x(), robot_state_.p.y(), robot_state_.p.z());
        //     // Eigen::Quaterniond drone_orientation(robot_state_.q.w(), robot_state_.q.x(), robot_state_.q.y(), robot_state_.q.z());
        //     // // LiDAR 相对于无人机的偏移量，假设 LiDAR 位于无人机上方 5cm
        //     // Eigen::Vector3d lidar_offset(0, 0, -0.05);  // Z轴上方 5 cm
        //     // // 计算 LiDAR 在世界坐标系中的位置
        //     // Eigen::Vector3d lidar_position_world = drone_orientation * lidar_offset + drone_position;
        //     // // 创建变换矩阵：旋转 + 平移
        //     // Eigen::Affine3d lidar_to_world_transform = Eigen::Affine3d::Identity();
        //     // lidar_to_world_transform.translation() = lidar_position_world;  // 平移
        //     // lidar_to_world_transform.linear() = drone_orientation.toRotationMatrix();  // 旋转

        //     // // 使用 pcl::transformPointCloud 将点云从 LiDAR 坐标系转换到世界坐标系
        //     // pcl::transformPointCloud(tmp_pc, tmp_pc, lidar_to_world_transform);


        //     rc_.updete_lock.lock();
        //     rc_.pc = tmp_pc;
        //     rc_.pc_pose = std::make_pair(robot_state_.p, robot_state_.q);
        //     rc_.unfinished_frame_cnt++;
        //     map_empty_ = false;
        //     rc_.updete_lock.unlock();
        // }

        void updateCallback(const ros::TimerEvent& event) {
            if (map_empty_) {
                static double last_print_t = ros::Time::now().toSec();
                double cur_t = ros::Time::now().toSec();
                if (cfg_.ros_callback_en && (cur_t - last_print_t > 1.0)) {
                    std::cout << YELLOW << " -- [ROG WARN] No point cloud input, check the topic name." << RESET <<
                        std::endl;
                    last_print_t = cur_t;
                }
                return;
            }
            if (rc_.unfinished_frame_cnt == 0) {
                return;
            }

            if (rc_.unfinished_frame_cnt > 1) {
                std::cout << YELLOW <<
                    " -- [ROG WARN] Unfinished frame cnt > 1, the map may not work in real-time" << RESET
                    << std::endl;
            }
            static PointCloud temp_pc;
            static Pose temp_pose;

            rc_.updete_lock.lock();
            temp_pc = rc_.pc;
            temp_pose = rc_.pc_pose;
            rc_.unfinished_frame_cnt = 0;
            rc_.updete_lock.unlock();

            updateProbMap(temp_pc, temp_pose);

            writeTimeConsumingToLog(time_log_file_);
        }


        void vizCallback(const ros::TimerEvent& event) {
            TimeConsuming ssss("vizCallback", false);

            if (!cfg_.visualization_en) {
                return;
            }
            if (map_empty_) {
                return;
            }

            Vec3f box_max = robot_state_.p + cfg_.visualization_range / 2;
            Vec3f box_min = robot_state_.p - cfg_.visualization_range / 2;

            boundBoxByLocalMap(box_min, box_max);
            if ((box_max - box_min).minCoeff() <= 0) {
                cout << YELLOW << " -- [ROGMap] Visualization range is too small." << RESET << endl;
                return;
            }

            if (cfg_.pub_unknown_map_en && vm_.unknown_pub.getNumSubscribers() >= 1) {
                vec_E<Vec3f> unknown_map, inf_unknown_map;
                boxSearch(box_min, box_max, UNKNOWN, unknown_map);
                sensor_msgs::PointCloud2 cloud_msg;
                vecEVec3fToPC2(unknown_map, cloud_msg);
                cloud_msg.header.stamp = ros::Time::now();
                vm_.unknown_pub.publish(cloud_msg);
                if (cfg_.unk_inflation_en && vm_.unknown_inf_pub.getNumSubscribers() >= 1) {
                    boxSearchInflate(box_min, box_max, UNKNOWN, inf_unknown_map);
                    vecEVec3fToPC2(inf_unknown_map, cloud_msg);
                    cloud_msg.header.stamp = ros::Time::now();
                    vm_.unknown_inf_pub.publish(cloud_msg);
                }
            }

            if (cfg_.frontier_extraction_en && vm_.frontier_pub.getNumSubscribers() >= 1) {
                vec_E<Vec3f> frontier_map;
                boxSearch(box_min, box_max, FRONTIER, frontier_map);
                sensor_msgs::PointCloud2 cloud_msg;
                vecEVec3fToPC2(frontier_map, cloud_msg);
                cloud_msg.header.stamp = ros::Time::now();
                vm_.frontier_pub.publish(cloud_msg);
            }

            vec_E<Vec3f> occ_map, inf_occ_map;
            sensor_msgs::PointCloud2 cloud_msg;
            if (vm_.occ_pub.getNumSubscribers() >= 1) {
                boxSearch(box_min, box_max, OCCUPIED, occ_map);
                vecEVec3fToPC2(occ_map, cloud_msg);
                vm_.occ_pub.publish(cloud_msg);
            }

            if (vm_.occ_inf_pub.getNumSubscribers() >= 1) {
                boxSearchInflate(box_min, box_max, OCCUPIED, inf_occ_map);
                vecEVec3fToPC2(inf_occ_map, cloud_msg);
                cloud_msg.header.stamp = ros::Time::now();
                vm_.occ_inf_pub.publish(cloud_msg);
            }

            /* visualize ESDF Map*/
            if (cfg_.esdf_en) {
                if (vm_.esdf_pub.getNumSubscribers() >= 1) {
                    PointCloud tmp_cloud;
                    esdf_map_->getPositiveESDFPointCloud(box_min, box_max, robot_state_.p.z() - 0.5, tmp_cloud);
                    pcl::toROSMsg(tmp_cloud, cloud_msg);
                    cloud_msg.header.stamp = ros::Time::now();
                    vm_.esdf_pub.publish(cloud_msg);
                }

                if (vm_.esdf_neg_pub.getNumSubscribers() >= 1) {
                    PointCloud tmp_cloud;
                    esdf_map_->getNegativeESDFPointCloud(box_min, box_max, robot_state_.p.z() - 0.5, tmp_cloud);
                    pcl::toROSMsg(tmp_cloud, cloud_msg);
                    cloud_msg.header.stamp = ros::Time::now();
                    vm_.esdf_neg_pub.publish(cloud_msg);
                }

#ifdef ESDF_MAP_DEBUG
        esdf_map_->getESDFOccPC2(box_min, box_max,cloud_msg);
        cloud_msg.header.stamp = ros::Time::now();
        vm_.esdf_occ_pub.publish(cloud_msg);
#endif
            }


            /* Publish visualization range */
            visualization_msgs::MarkerArray mkr_arr;
            visualizeBoundingBox(mkr_arr, box_min, box_max, "Visualization Range", Color::Purple());
            visualizeText(mkr_arr, "Visualization Range Text", "Visualization Range", box_max + Vec3f(0, 0, 0.5),
                          Color::Purple(), 0.6, 0);

            /* Publish local map range */
            Vec3f local_map_max(999, 999, 999), local_map_min(-999, -999, -999);
            boundBoxByLocalMap(local_map_min, local_map_max);
            visualizeBoundingBox(mkr_arr, local_map_min, local_map_max, "Local Map Range",
                                 Color::Orange());
            visualizeText(mkr_arr, "Local Map Range Text", "Local Map Range", local_map_max + Vec3f(0, 0, 1.0),
                          Color::Orange(),
                          0.6, 0);

            /* Publish Ray-casting range */
            visualizeBoundingBox(mkr_arr, raycast_data_.cache_box_min, raycast_data_.cache_box_max,
                                 "Updating Range",
                                 Color::Green());
            visualizeText(mkr_arr, "Updating Range Text", "Updating Range",
                          raycast_data_.cache_box_max + Vec3f(0, 0, 0.5),
                          Color::Green(), 0.6, 0);

            /* Publish Local map origin */
            visualizePoint(mkr_arr, local_map_origin_d_, Color::Red(), "Local Map Origin", 0.2, 0);

            if (cfg_.esdf_en) {
                Vec3f esdf_box_max, esdf_box_min;
                esdf_map_->getUpdatedBbox(esdf_box_min, esdf_box_max);
                visualizeText(mkr_arr, "ESDF Map Text", "ESDF Map", esdf_box_max + Vec3f(0, 0, 1.0),
                              Color::Blue(),
                              0.6, 0);
                visualizeBoundingBox(mkr_arr, esdf_box_min, esdf_box_max, "ESDF Updating Range",
                                     Color::Blue());
            }

            vm_.mkr_arr_pub.publish(mkr_arr);
        }

        void vecEVec3fToPC2(const vec_E<Vec3f>& points, sensor_msgs::PointCloud2& cloud) {
            // 设置header信息
            pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
            pcl_cloud.resize(points.size());
            for (long unsigned int i = 0; i < points.size(); i++) {
                pcl_cloud[i].x = static_cast<float>(points[i][0]);
                pcl_cloud[i].y = static_cast<float>(points[i][1]);
                pcl_cloud[i].z = static_cast<float>(points[i][2]);
            }
            pcl::toROSMsg(pcl_cloud, cloud);
            cloud.header.stamp = ros::Time::now();
            cloud.header.frame_id = "world";
        }

    public:
        typedef shared_ptr<ROGMapROS> Ptr;

        ROGMapROS(const ros::NodeHandle& nh, const std::string& cfg_path) :nh_(nh){
            cfg_ = rog_map::Config(cfg_path);
            init(); //初始化
            
            // Get UAV name from parameter server, default to "uav1"
            std::string uav_name = "uav1";
            nh_.param<std::string>("uav_name", uav_name, "uav1");
            std::string map_prefix = "/" + uav_name + "/rog_map";
            
            /// Initialize visualization module
            if (cfg_.visualization_en) {
                vm_.occ_pub = nh_.advertise<sensor_msgs::PointCloud2>(map_prefix + "/occ", 1);
                vm_.unknown_pub = nh_.advertise<sensor_msgs::PointCloud2>(map_prefix + "/unk", 1);
                vm_.occ_inf_pub = nh_.advertise<sensor_msgs::PointCloud2>(map_prefix + "/inf_occ", 1);
                vm_.unknown_inf_pub = nh_.advertise<sensor_msgs::PointCloud2>(map_prefix + "/inf_unk", 1);

                if (cfg_.frontier_extraction_en) {
                    vm_.frontier_pub = nh_.advertise<sensor_msgs::PointCloud2>(map_prefix + "/frontier", 1);
                }

                if (cfg_.esdf_en) {
                    vm_.esdf_pub = nh_.advertise<sensor_msgs::PointCloud2>(map_prefix + "/esdf", 1);
                    vm_.esdf_neg_pub = nh_.advertise<sensor_msgs::PointCloud2>(map_prefix + "/esdf/neg", 1);
                    vm_.esdf_occ_pub = nh_.advertise<sensor_msgs::PointCloud2>(map_prefix + "/esdf/occ", 1);
                }

                if (cfg_.viz_time_rate > 0) {
                    vm_.viz_timer = nh_.createTimer(ros::Duration(1.0 / cfg_.viz_time_rate), &ROGMapROS::vizCallback,
                                                    this);
                }
            }
            vm_.mkr_arr_pub = nh_.advertise<visualization_msgs::MarkerArray>(map_prefix + "/map_bound", 1);
            state.data=-1;
            if (cfg_.ros_callback_en) { //#开启回调  odom cloud timer_update
                rc_.odom_sub = nh_.subscribe(cfg_.odom_topic, 1, &ROGMapROS::odomCallback, this);
                rc_.cloud_sub = nh_.subscribe(cfg_.cloud_topic, 1, &ROGMapROS::cloudCallback, this);
                // rc_.cloud_f_sub = nh_.subscribe(cfg_.cloud_f_topic, 1, &ROGMapROS::cloudCallback_f, this);
                // rc_.depth_sub.subscribe(nh_, cfg_.depth_topic, 1,ros::TransportHints().tcpNoDelay());
                // rc_.odom_depth_sub.subscribe(nh_, cfg_.odom_topic, 1,ros::TransportHints().tcpNoDelay());
                // rc_.sync_image_odom_.reset(new message_filters::Synchronizer<SyncPolicyImageOdom>(
                //     SyncPolicyImageOdom(10), rc_.depth_sub, rc_.odom_depth_sub));
                // rc_.sync_image_odom_->setInterMessageLowerBound(ros::Duration(0.1));
                // rc_.sync_image_odom_->registerCallback(boost::bind(&ROGMapROS::depthOdomCallback, this, _1, _2));
                // rc_.occ_timer_ = nh_.createTimer(ros::Duration(0.01), &ROGMapROS::projectDepthImage, this);
                
                rc_.state_sub = nh_.subscribe<std_msgs::Int8>("/state_machine", 1, &ROGMapROS::stateMachineCallback, this, ros::TransportHints().tcpNoDelay());
                rc_.update_timer = nh_.createTimer(ros::Duration(0.001), &ROGMapROS::updateCallback, this);
            }
        }

    private:
        static void visualizeBoundingBox(visualization_msgs::MarkerArray& mkrarr,
                                         const Vec3f& box_min,
                                         const Vec3f& box_max,
                                         const string& ns,
                                         const Color& color,
                                         const double& size_x = 0.1,
                                         const double& alpha = 1.0,
                                         const bool& print_ns = true) {
            Vec3f size = (box_max - box_min) / 2;
            Vec3f vis_pos_world = (box_min + box_max) / 2;
            double width = size.x();
            double length = size.y();
            double hight = size.z();

            //Publish Bounding box
            int id = 0;
            visualization_msgs::Marker line_strip;
            line_strip.header.stamp = ros::Time::now();
            line_strip.header.frame_id = "world";
            line_strip.action = visualization_msgs::Marker::ADD;
            line_strip.ns = ns;
            line_strip.pose.orientation.w = 1.0;
            line_strip.id = id++; //unique id, useful when multiple markers exist.
            line_strip.type = visualization_msgs::Marker::LINE_STRIP; //marker type
            line_strip.scale.x = size_x;


            line_strip.color = color;
            line_strip.color.a = alpha; //不透明度，设0则全透明
            geometry_msgs::Point p[8];

            //vis_pos_world是目标物的坐标
            p[0].x = vis_pos_world(0) - width;
            p[0].y = vis_pos_world(1) + length;
            p[0].z = vis_pos_world(2) + hight;
            p[1].x = vis_pos_world(0) - width;
            p[1].y = vis_pos_world(1) - length;
            p[1].z = vis_pos_world(2) + hight;
            p[2].x = vis_pos_world(0) - width;
            p[2].y = vis_pos_world(1) - length;
            p[2].z = vis_pos_world(2) - hight;
            p[3].x = vis_pos_world(0) - width;
            p[3].y = vis_pos_world(1) + length;
            p[3].z = vis_pos_world(2) - hight;
            p[4].x = vis_pos_world(0) + width;
            p[4].y = vis_pos_world(1) + length;
            p[4].z = vis_pos_world(2) - hight;
            p[5].x = vis_pos_world(0) + width;
            p[5].y = vis_pos_world(1) - length;
            p[5].z = vis_pos_world(2) - hight;
            p[6].x = vis_pos_world(0) + width;
            p[6].y = vis_pos_world(1) - length;
            p[6].z = vis_pos_world(2) + hight;
            p[7].x = vis_pos_world(0) + width;
            p[7].y = vis_pos_world(1) + length;
            p[7].z = vis_pos_world(2) + hight;
            //LINE_STRIP类型仅仅将line_strip.points中相邻的两个点相连，如0和1，1和2，2和3
            for (int i = 0; i < 8; i++) {
                line_strip.points.push_back(p[i]);
            }
            //为了保证矩形框的八条边都存在：
            line_strip.points.push_back(p[0]);
            line_strip.points.push_back(p[3]);
            line_strip.points.push_back(p[2]);
            line_strip.points.push_back(p[5]);
            line_strip.points.push_back(p[6]);
            line_strip.points.push_back(p[1]);
            line_strip.points.push_back(p[0]);
            line_strip.points.push_back(p[7]);
            line_strip.points.push_back(p[4]);
            mkrarr.markers.push_back(line_strip);
        }

        static void visualizeText(visualization_msgs::MarkerArray& mkr_arr,
                                  const std::string& ns,
                                  const std::string& text,
                                  const Vec3f& position,
                                  const Color& c = Color::White(),
                                  const double& size = 0.6,
                                  const int& id = -1) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.ns = ns.c_str();
            if (id >= 0) {
                marker.id = id;
            }
            else {
                static int id = 0;
                marker.id = id++;
            }
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.scale.z = size;
            marker.color = c;
            marker.text = text;
            marker.pose.position.x = position.x();
            marker.pose.position.y = position.y();
            marker.pose.position.z = position.z();
            marker.pose.orientation.w = 1.0;
            mkr_arr.markers.push_back(marker);
        };

        static void visualizePoint(visualization_msgs::MarkerArray& mkr_arr,
                                   const Vec3f& pt,
                                   Color color = Color::Pink(),
                                   std::string ns = "pt",
                                   double size = 0.1, int id = -1,
                                   const bool& print_ns = true) {
            visualization_msgs::Marker marker_ball;
            static int cnt = 0;
            Vec3f cur_pos = pt;
            if (isnan(pt.x()) || isnan(pt.y()) || isnan(pt.z())) {
                return;
            }
            marker_ball.header.frame_id = "world";
            marker_ball.header.stamp = ros::Time::now();
            marker_ball.ns = ns.c_str();
            marker_ball.id = id >= 0 ? id : cnt++;
            marker_ball.action = visualization_msgs::Marker::ADD;
            marker_ball.pose.orientation.w = 1.0;
            marker_ball.type = visualization_msgs::Marker::SPHERE;
            marker_ball.scale.x = size;
            marker_ball.scale.y = size;
            marker_ball.scale.z = size;
            marker_ball.color = color;

            geometry_msgs::Point p;
            p.x = cur_pos.x();
            p.y = cur_pos.y();
            p.z = cur_pos.z();

            marker_ball.pose.position = p;
            mkr_arr.markers.push_back(marker_ball);

            // add test
            if (print_ns) {
                visualization_msgs::Marker marker;
                marker.header.frame_id = "world";
                marker.header.stamp = ros::Time::now();
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.orientation.w = 1.0;
                marker.ns = ns + "_text";
                if (id >= 0) {
                    marker.id = id;
                }
                else {
                    static int id = 0;
                    marker.id = id++;
                }
                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                marker.scale.z = 0.6;
                marker.color = color;
                marker.text = ns;
                marker.pose.position.x = cur_pos.x();
                marker.pose.position.y = cur_pos.y();
                marker.pose.position.z = cur_pos.z() + 0.5;
                marker.pose.orientation.w = 1.0;
                mkr_arr.markers.push_back(marker);
            }
        }
    };
}
#endif // ROG_MAP_ROS_HPP
#endif // USE_ROS1