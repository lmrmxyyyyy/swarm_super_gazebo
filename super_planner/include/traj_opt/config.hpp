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

#pragma once

#include <string>
#include <utils/geometry/quadrotor_flatness.hpp>
#include <utils/header/yaml_loader.hpp>
#define DEBUG_FILE_DIR(name) (string(string(ROOT_DIR) + "log/"+(name)))

namespace traj_opt {
    using std::string;
    using std::vector;

    enum PosConstrainType {
        WAYPOINT = 1,
        CORRIDOR = 2,
    };

    class Config {
    public:
        flatness::FlatnessMap quadrotot_flatness;

        bool print_optimizer_log{false};

        /// Param for flatness
        double mass, dh, dv, grav, cp, v_eps;

        // if save the optimization problem to log
        bool save_log_en{false};

        int pos_constraint_type{CORRIDOR};
        // Set to true for only min time.
        bool block_energy_cost{false};
        // Limit conditions.
        double max_vel{0}, max_acc{0}, max_jerk{0}, max_omg{0}, max_acc_thr{0}, min_acc_thr{0};
        // Penalty cost.
        double penna_scale{-1}, penna_vel{0}, penna_acc{0}, penna_jerk{0}, penna_omg{0}, penna_thr{0};
        // penna_t; penna_pos only for corridor based method.
        double penna_t{0}, penna_pos{0}, penna_attract{0};
        // penna_ts only for backupTraj;
        double penna_ts{0};
        // for backup traj piece num
        int piece_num{0};

        double penna_margin{0.05};

        double smooth_eps{0};
        int integral_reso{0};
        double opt_accuracy{0};
        bool local_density_en{false};
        double local_density_radius_x{0.5};
        double local_density_radius_y{0.5};
        double local_density_radius_z{0.35};
        double local_density_smooth_eps{0.1};
        double local_density_time_inflation{0.1};
        double local_density_weight{0.0};
        double trigger_density_weight{0.0};
        double trigger_static_clearance{0.8};
        double trigger_static_weight{1.0};
        double trigger_swarm_weight{1.0};
        double ts_delay_weight{0.0};
        double ts_delay_high_order_weight{0.0};
        double ts_delay_deadband{0.0};
        double ts_anchor_weight{0.0};
        double ts_anchor_deadband{0.0};
        double tail_anchor_weight{0.0};
        double tail_anchor_deadband{0.0};
        double time_balance_weight{0.0};

        Config() = default;

        Config(const std::string & cfg_path, string ns) {  //轨迹参数加载
            yaml_loader::YamlLoader loader(cfg_path);
            if (ns.empty()) {
                ns = "/";
            }
            else {
                ns = "/" + ns + "/";
            }

            loader.LoadParam("traj_opt/switch/print_optimizer_log", print_optimizer_log, false);//print优化器日志
            /// Load Param for Flatness
            loader.LoadParam("traj_opt/flatness/mass", mass, 1.0); //平滑化参数
            loader.LoadParam("traj_opt/flatness/dh", dh, 0.7);
            loader.LoadParam("traj_opt/flatness/dv", dv, 0.8);
            loader.LoadParam("traj_opt/flatness/grav", grav, 1.0);
            loader.LoadParam("traj_opt/flatness/cp", cp, 0.01);
            loader.LoadParam("traj_opt/flatness/v_eps", v_eps, 0.0001);

            loader.LoadParam("traj_opt/switch/save_log_en", save_log_en, false);// 保存日志
            loader.LoadParam("traj_opt" + ns + "pos_constraint_type", pos_constraint_type, 2);//位置约束类型
            loader.LoadParam("traj_opt" + ns + "piece_num", piece_num, 1);
            loader.LoadParam("traj_opt" + ns + "block_energy_cost", block_energy_cost, false);  //
            loader.LoadParam("traj_opt" + ns + "opt_accuracy", opt_accuracy, 1.0e-5);  //求解精度
            loader.LoadParam("traj_opt" + ns + "integral_reso", integral_reso, 10);
            loader.LoadParam("traj_opt" + ns + "smooth_eps", smooth_eps, 0.01);
            loader.LoadParam("traj_opt" + ns + "local_density_en", local_density_en, false);
            loader.LoadParam("traj_opt" + ns + "local_density_radius_x", local_density_radius_x, 0.5);
            loader.LoadParam("traj_opt" + ns + "local_density_radius_y", local_density_radius_y, 0.5);
            loader.LoadParam("traj_opt" + ns + "local_density_radius_z", local_density_radius_z, 0.35);
            loader.LoadParam("traj_opt" + ns + "local_density_smooth_eps", local_density_smooth_eps, 0.1);
            loader.LoadParam("traj_opt" + ns + "local_density_time_inflation", local_density_time_inflation, 0.1);
            loader.LoadParam("traj_opt" + ns + "local_density_weight", local_density_weight, 0.0);
            loader.LoadParam("traj_opt" + ns + "trigger_density_weight", trigger_density_weight, 0.0);
            loader.LoadParam("traj_opt" + ns + "trigger_static_clearance", trigger_static_clearance, 0.8);
            loader.LoadParam("traj_opt" + ns + "trigger_static_weight", trigger_static_weight, 1.0);
            loader.LoadParam("traj_opt" + ns + "trigger_swarm_weight", trigger_swarm_weight, 1.0);
            loader.LoadParam("traj_opt" + ns + "ts_delay_weight", ts_delay_weight, 0.0);
            loader.LoadParam("traj_opt" + ns + "ts_delay_high_order_weight", ts_delay_high_order_weight, 0.0);
            loader.LoadParam("traj_opt" + ns + "ts_delay_deadband", ts_delay_deadband, 0.0);
            loader.LoadParam("traj_opt" + ns + "ts_anchor_weight", ts_anchor_weight, 0.0);
            loader.LoadParam("traj_opt" + ns + "ts_anchor_deadband", ts_anchor_deadband, 0.0);
            loader.LoadParam("traj_opt" + ns + "tail_anchor_weight", tail_anchor_weight, 0.0);
            loader.LoadParam("traj_opt" + ns + "tail_anchor_deadband", tail_anchor_deadband, 0.0);
            loader.LoadParam("traj_opt" + ns + "time_balance_weight", time_balance_weight, 0.0);
            loader.LoadParam("traj_opt/boundary/max_vel", max_vel, -1.0);///边界参数
            loader.LoadParam("traj_opt/boundary/max_acc", max_acc, -1.0);
            loader.LoadParam("traj_opt/boundary/max_jerk", max_jerk, -1.0);
            loader.LoadParam("traj_opt/boundary/max_omg", max_omg, -1.0);
            loader.LoadParam("traj_opt/boundary/max_acc_thr", max_acc_thr, -1.0);
            loader.LoadParam("traj_opt/boundary/min_acc_thr", min_acc_thr, -1.0);
            loader.LoadParam("traj_opt/boundary/penna_margin", penna_margin, 0.05);

            loader.LoadParam("traj_opt" + ns + "penna_scale", penna_scale, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_t", penna_t, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_ts", penna_ts, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_pos", penna_pos, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_vel", penna_vel, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_acc", penna_acc, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_jerk", penna_jerk, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_attract", penna_attract, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_omg", penna_omg, -1.0);
            loader.LoadParam("traj_opt" + ns + "penna_thr", penna_thr, -1.0);

            if (penna_scale > 0) {
                penna_t = penna_t * penna_scale;
                penna_ts = penna_ts * penna_scale;
                penna_pos = penna_pos * penna_scale;
                penna_vel = penna_vel * penna_scale;
                penna_acc = penna_acc * penna_scale;
                penna_jerk = penna_jerk * penna_scale;
                penna_attract = penna_attract * penna_scale;
                penna_omg = penna_omg * penna_scale;
                penna_thr = penna_thr * penna_scale;
            }

            quadrotot_flatness.reset(mass, grav, dh, dv, cp, v_eps);
        }
    };
}
