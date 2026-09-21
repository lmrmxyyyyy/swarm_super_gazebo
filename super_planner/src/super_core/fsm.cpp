/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
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

#include <fsm/fsm.h>
#include <memory>
#include "std_msgs/Int8.h"

using namespace super_utils;

namespace fsm {
    Fsm::~Fsm() {
        write_time_.close();
    }

    void Fsm::WriteTimeToLog() {
        write_time_ << (ros_ptr_->getSimTime() - system_start_time_) << ", ";
        for (long unsigned int i = 0; i < log_module_time.size(); i++) {
            write_time_ << log_module_time[i];
            if (i != log_module_time.size() - 1) {
                write_time_ << ", ";
            }
        }
        write_time_ << endl;
    }

    void Fsm::callReplanOnce() {
        if (stop) {
            return;
        }

        if (machine_state_ != FOLLOW_TRAJ) {//必须处于 FOLLOW_TRAJ（跟随轨迹状态），否则不进行重规划。
            return;
        }

        if (finish_plan) {//规划已经完成，则不需要再进行重规划，直接返回
            return;
        }

        if (plan_from_rest_) {//最近刚进行过 PlanFromRest 规划，不需要立即重新规划
            plan_from_rest_ = false;
            return;
        }

        std_msgs::Int8 end_in_obs_flag;
        end_in_obs_flag.data=0;
        bool obs_flag_;
        //确保目标点 gi_.goal_p 是 可行驶的，如果目标点在障碍物中，则寻找最近的 非占据（GridType::OCCUPIED 之外）网格点。
        obs_flag_=planner_ptr_->getMap()->getNearestInfCellNot(GridType::OCCUPIED, gi_.goal_p, gi_.goal_p, 1.5);//搜索范围为 3.0 米
        if (obs_flag_== false) {end_in_obs_flag.data=1;}
        // target_in_obs_pub.publish(end_in_obs_flag);


        TimeConsuming replan_once_time("replan_once_time", false);// 计时器
        //进行路径重规划
        RET_CODE ret_code = planner_ptr_->ReplanOnce(gi_.goal_p, gi_.goal_yaw, gi_.new_goal);
        if (ret_code == FAILED) {//处理规划结果
//            cout << YELLOW << " -- [Fsm] ReplanOnce failed." << RESET << endl;
        } else { cout << GREEN << " -- [Fsm] ReplanOnce succeed." << RESET << endl; }

        if (ret_code == EMER) {
            ChangeState("ReplanTimerCallback", EMER_STOP);
        } else if (ret_code == NEW_TRAJ) {//生成轨迹状态
            ChangeState("ReplanTimerCallback", GENERATE_TRAJ);
        } else if (ret_code == SUCCESS || ret_code == FINISH) {//成功完成重规划
            gi_.new_goal = false;
            publishPolyTraj();
        }

        planner_ptr_->getModuleTimeConsuming(log_module_time);
        log_module_time[log_module_time.size() - 2] = replan_once_time.stop();
        // save on log
        replan_logs_.push_back(planner_ptr_->getLatestReplanLog());
        WriteTimeToLog();
    }

    void Fsm::callMainFsmOnce() {  //状态机时间回调函数 mainFsmTimerCallback
        if (stop) {//如果 stop 标志为真，直接退出函数
            return;
        }
        static double fsm_start_time = ros_ptr_->getSimTime();//记录状态机开始的时间
        double cur_t = (ros_ptr_->getSimTime() - fsm_start_time);//return ros::Time::now().toSec();
        static double last_print_t = 0.0;//用于控制日志输出的时间间隔，确保日志不是每次都输出，而是间隔一段时间后输出。
        planner_ptr_->getRobotState(robot_state_); //planner 获取机器人状态


        if (cur_t - last_print_t > 1.0) {  //打印
            last_print_t = cur_t;
            if ((!robot_state_.rcv || (ros_ptr_->getSimTime() - robot_state_.rcv_time) > 0.1)) {  //初始没odom或者 odom超时
                cout << YELLOW << " -- [Fsm] No odom." << RESET << endl;
                return;
            }
            if (!started_) { //没开始
                cout << YELLOW << " -- [Fsm] Wait for goal." << RESET << endl;
            }
            cout << std::fixed << std::setprecision(3);
            cout << GREEN << " -- [Fsm " << cur_t << "] Current state: " << MACHINE_STATE_STR[machine_state_]
                 << RESET << endl;
        }

        switch (machine_state_) {
            case INIT: {
                if (!started_) {
                    return;
                }
                if ((!robot_state_.rcv || (ros_ptr_->getSimTime() - robot_state_.rcv_time) > 0.1)) {
                    cout << YELLOW << " -- [Fsm] No odom." << RESET << endl;
                }
                ChangeState("MainFsmCallback", WAIT_GOAL);  //状态到 WAIT_GOAL
                break;
            }
            case WAIT_GOAL: {
                if (!gi_.new_goal) {  //没收到新目标
                    return;
                } else {
                    ChangeState("MainFsmCallback", GENERATE_TRAJ);  //到生成轨迹状态
                }
                resetVisualizedPath();  //重置可视化path
                break;
            }
            case GENERATE_TRAJ: {
                if (closeToGoal(0.1)) {  //足够接近目标
                    ChangeState("MainFsmCallback", WAIT_GOAL);  //到WAIT_GOAL状态
                    gi_.new_goal = false;
                    finish_plan = true;
                    return;
                }
                int retcode = planner_ptr_->PlanFromRest(gi_.goal_p, gi_.goal_yaw, gi_.new_goal); //新点来了，从reset开始规划轨迹
                if (!planner_ptr_->goalValid()) {  //如果goal不可用，skip
                    cout << YELLOW << " -- [Fsm] Goal is invalid, skip this goal." << RESET << endl;
                    ChangeState("MainFsmCallback", WAIT_GOAL);
                    return;
                }
                if (retcode == SUCCESS || retcode == FINISH) {  //规划成功
                    gi_.new_goal = false;
                    plan_from_rest_ = true; //从reset开始plan
                    finish_plan = false;
                    if (retcode == FINISH) {
                        finish_plan = true;
                    }

                    publishPolyTraj();  //publish 多项式轨迹

                    ChangeState("MainFsmCallback", FOLLOW_TRAJ);  //状态到FOLLOW_TRAJ
                } else { //规划不成功
                    cout << YELLOW << " -- [Fsm] PlanFromRest failed, try replan." << RESET << endl;
                    // ros::Duration(0.1).sleep();
                }
                replan_logs_.push_back(planner_ptr_->getLatestReplanLog());//记录replan日志（）sfc和time
                break;
            }
            case FOLLOW_TRAJ: {
                publishCurPoseToPath(); //发布robot_state (pos,q)和path
                break;
            }
            case EMER_STOP: {
                ChangeState("MainFsmCallback", WAIT_GOAL);
                break;
            }
            default:
                break;
        }
    }

    bool Fsm::closeToGoal(const double &thresh_dis) {
        /// The close to goal should consider the the local shift
        /// All goal should be in the known free on inf map.
        /// The intermedia points should be in free space.
        double dis = (robot_state_.p - gi_.goal_p).norm();
        return dis < thresh_dis;
    }

    void Fsm::setGoalPosiAndYaw(const Vec3f &p, const Quatf &q) {

        auto click_point = p;
        // if (cfg_.click_height > -5) {
        //     click_point.z() = cfg_.click_height;
        // }
        std_msgs::Int8 end_in_obs_flag;
        end_in_obs_flag.data=0;
        if (planner_ptr_->getMap()->getNearestInfCellNot(GridType::OCCUPIED, click_point, gi_.goal_p, 1.5)) {
            cout << GREEN << " -- [Fsm] Get goal at " << RESET << gi_.goal_p.transpose() << endl;
        } else {
            fmt::print(fg(fmt::color::indian_red), "Goal is deeply occupied, skip this goal.\n");
            end_in_obs_flag.data=1;
            // target_in_obs_pub.publish(end_in_obs_flag);
            return;
        }

        if ((robot_state_.p - gi_.goal_p).norm() <
            0.1) {
            //                print(fg(color::gray), " -- [Rviz] Too close to goal, skip this target.\n");
            return;
        }

        if (cfg_.click_yaw_en) {
            if (isnan(q.w()) || isnan(q.x()) || isnan(q.y()) || isnan(q.z())) {
                gi_.goal_yaw = NAN;
                ros_ptr_->info(" -- [Fsm] Receive click goal at: [{}, {}, {}]; goal yaw disabled",
                               gi_.goal_p.x(), gi_.goal_p.y(), gi_.goal_p.z());
            } else {
                gi_.goal_yaw = geometry_utils::get_yaw_from_quaternion(q);
                cout << GREEN << " -- [Fsm] Receive click goal at: [" << gi_.goal_p.transpose() << "]; goal yaw: "
                     << gi_.goal_yaw * 57.3 << " deg" << RESET << endl;
            }

        } else {
            gi_.goal_yaw = NAN;
            cout << GREEN << " -- [Fsm] Receive click goal at: [" << gi_.goal_p.transpose() << "]; goal yaw disabled"
                 << RESET << endl;
        }

        started_ = true;
        gi_.new_goal = true;
    }

    void Fsm::ChangeState(const string &call_func, const MACHINE_STATE &new_state) {
        fmt::print(fg(fmt::color::green), " -- [Fsm]: [{}] change state from [{}] to [{}].\n", call_func,
                   MACHINE_STATE_STR[int(machine_state_)], MACHINE_STATE_STR[int(new_state)]);
        machine_state_ = new_state;
    }
}
