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


#ifndef SUPER_PLANNER_CONFIG_HPP
#define SUPER_PLANNER_CONFIG_HPP

#include <rog_map/rog_map_core/config.hpp>
#include <traj_opt/config.hpp>
#include <utils/header/yaml_loader.hpp>

namespace super_planner {
    using namespace traj_opt;
    using std::cout;
    using std::endl;

    class Config {
    public:
        enum YawMode{
            YAW_TO_VEL = 1,
            YAW_TO_GOAL = 2
        };

        traj_opt::Config exp_traj_cfg, back_traj_cfg;

        // Bool Params
        bool visualization_en{true};
        bool detailed_log_en{false};
        bool backup_traj_en;
        bool use_fov_cut, print_log;
        bool goal_vel_en,goal_yaw_en;
        bool visual_process;
        bool frontend_in_known_free;
        bool use_dstar_lite_frontend{true};
        bool tracking_drift_recovery_en{true};
        bool backup_traj_risk_gate_en{false};

        double resolution;
        double planning_horizon;
        double receding_dis;
        double safe_corridor_line_max_length;
        // for fov cut
        double sensing_horizon;

        // Planning Params
        int obs_skip_num;
        double corridor_bound_dis, corridor_line_max_length;
        double replan_forward_dt;
        double max_replan_time;
        double sample_traj_dt;
        double robot_r;
        double backup_traj_trigger_clearance{0.0};
        double backup_traj_trigger_lookahead_time{0.0};
        int iris_iter_num;

        int mpc_horizon{};

        double yaw_dot_max;
        // Yaw mode: 1 heading to velocity, 2 heading to goal
        int yaw_mode = YAW_TO_VEL;

        rog_map::vec_E<rog_map::Vec3i> seed_line_neighbour;//rogmap的膨胀表


        Config() = default;
        Config(const std::string & cfg_path) {
            yaml_loader::YamlLoader loader(cfg_path);
            exp_traj_cfg = traj_opt::Config(cfg_path, "exp_traj");//加载探索轨迹参数
            back_traj_cfg = traj_opt::Config(cfg_path, "backup_traj");//加载备份轨迹参数
            loader.LoadParam("super_planner/print_log", print_log, false);  //打印log
            loader.LoadParam("super_planner/detailed_log_en", detailed_log_en, false);//#详细log
            loader.LoadParam("super_planner/visualization_en", visualization_en, false); //开启可视化
            loader.LoadParam("super_planner/backup_traj_en", backup_traj_en, false);  //#开启备份轨迹
            loader.LoadParam("super_planner/goal_vel_en", goal_vel_en, false);    //目标速度开启
            loader.LoadParam("super_planner/goal_yaw_en", goal_yaw_en, false);   //目标yaw开启
            loader.LoadParam("super_planner/visual_process", visual_process, false);  //可视化处理
            loader.LoadParam("super_planner/use_fov_cut", use_fov_cut, false);  //#裁减fov？
            loader.LoadParam("super_planner/frontend_in_known_free", frontend_in_known_free, false);//#前端在未知边界内？
            loader.LoadParam("super_planner/use_dstar_lite_frontend", use_dstar_lite_frontend, true);
            loader.LoadParam("super_planner/tracking_drift_recovery_en", tracking_drift_recovery_en, true);
            loader.LoadParam("super_planner/backup_traj_risk_gate_en", backup_traj_risk_gate_en, false);
            loader.LoadParam("super_planner/safe_corridor_line_max_length", safe_corridor_line_max_length, 3.0);//安全走廊线最大长度
            loader.LoadParam("super_planner/sensing_horizon", sensing_horizon, 3.0);//#感知界限
            loader.LoadParam("super_planner/obs_skip_num", obs_skip_num, 1); //#障碍物躲避数量？
            loader.LoadParam("super_planner/replan_forward_dt", replan_forward_dt, 0.3);//#重规划向前时间
            loader.LoadParam("super_planner/max_replan_time", max_replan_time, std::max(1.0, replan_forward_dt));
            loader.LoadParam("super_planner/corridor_bound_dis", corridor_bound_dis, 3.0);  //走廊边界距离
            loader.LoadParam("super_planner/corridor_line_max_length", corridor_line_max_length, 3.0); //走廊线最大长度
            loader.LoadParam("super_planner/planning_horizon", planning_horizon, 10.0);   //#规划界限
            loader.LoadParam("super_planner/receding_dis", receding_dis, 5.0);  //后退距离
            loader.LoadParam("super_planner/robot_r", robot_r, 0.3);  //#机器人半径
            loader.LoadParam("super_planner/backup_traj_trigger_clearance", backup_traj_trigger_clearance,
                             std::max(0.0, robot_r));
            loader.LoadParam("super_planner/backup_traj_trigger_lookahead_time",
                             backup_traj_trigger_lookahead_time, 0.0);
            loader.LoadParam("super_planner/iris_iter_num", iris_iter_num, 1);  //iris迭代次数
            loader.LoadParam("super_planner/yaw_mode", yaw_mode, 1);  //# Yaw mode: 1 heading to velocity, 2 heading to goal
            loader.LoadParam("super_planner/mpc_horizon", mpc_horizon, 1);  //#mpc步数
            loader.LoadParam("super_planner/yaw_dot_max", yaw_dot_max, 3.14);  //最大yaw角速度

            loader.LoadParam("rog_map/resolution", resolution, 0.01, true);  //rog map 分辨率

            sample_traj_dt = resolution / exp_traj_cfg.max_vel;

            int step = ceil(robot_r / resolution);//robotr步数
            for (int x = -step; x <= step; x++) {
                for (int y = -step; y <= step; y++) {
                    for (int z = -step; z <= step; z++) {
                        if (x * x + y * y + z * z <= step * step) {
                            seed_line_neighbour.push_back({x, y, z});//种子线邻居
                        }
                    }
                }
            }
            std::sort(seed_line_neighbour.begin(), seed_line_neighbour.end(),
                      [](const auto& a, const auto& b) {
                          return a[0] * a[0] + a[1] * a[1] + a[2] * a[2] < b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
                      });
        }


    };
}

#endif
