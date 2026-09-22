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

#include <super_core/super_planner.h>
#include <memory>
#include <super_utils/scope_timer.hpp>
#include <fmt/color.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>

#include <boost/bind.hpp>

using namespace super_utils;

namespace super_planner {
    namespace {
        bool samePoint(const Vec3f &a, const Vec3f &b) {
            return (a - b).squaredNorm() < 1.0e-8;
        }

        bool frontendLineFree(const Vec3f &from,
                              const Vec3f &to,
                              const rog_map::ROGMapROS::Ptr &map_ptr,
                              const bool unknown_as_occupied) {
            return map_ptr && map_ptr->isLineFree(from, to, true, unknown_as_occupied);
        }

        double cornerCost(const Vec3f &prev, const Vec3f &cur, const Vec3f &next) {
            const Vec3f in = cur - prev;
            const Vec3f out = next - cur;
            const double in_norm = in.norm();
            const double out_norm = out.norm();
            if (in_norm < 1.0e-6 || out_norm < 1.0e-6) {
                return 0.0;
            }
            const double cos_angle = std::max(-1.0, std::min(1.0, in.dot(out) / (in_norm * out_norm)));
            return 1.0 - cos_angle;
        }

        void removeRedundantCorners(vec_Vec3f &path,
                                    const rog_map::ROGMapROS::Ptr &map_ptr,
                                    const bool unknown_as_occupied) {
            if (path.size() <= 2 || !map_ptr) {
                return;
            }

            constexpr double kNearlyStraightTurnCost = 0.035; // about 15 deg
            constexpr double kShortSegmentLength = 0.35;
            constexpr double kShortCornerTurnCost = 0.18; // about 35 deg

            bool changed = true;
            int pass = 0;
            while (changed && pass++ < 3 && path.size() > 2) {
                changed = false;
                vec_Vec3f sparse;
                sparse.reserve(path.size());
                sparse.push_back(path.front());

                for (size_t i = 1; i + 1 < path.size(); ++i) {
                    const Vec3f &prev = sparse.back();
                    const Vec3f &cur = path[i];
                    const Vec3f &next = path[i + 1];
                    const double in_len = (cur - prev).norm();
                    const double out_len = (next - cur).norm();
                    const double turn_cost = cornerCost(prev, cur, next);
                    const bool weak_corner = turn_cost < kNearlyStraightTurnCost;
                    const bool short_corner = std::min(in_len, out_len) < kShortSegmentLength &&
                                              turn_cost < kShortCornerTurnCost;

                    if ((weak_corner || short_corner) &&
                        frontendLineFree(prev, next, map_ptr, unknown_as_occupied)) {
                        changed = true;
                        continue;
                    }

                    sparse.push_back(cur);
                }

                sparse.push_back(path.back());
                path.swap(sparse);
            }
        }

        void softenSharpCorners(vec_Vec3f &path,
                                const rog_map::ROGMapROS::Ptr &map_ptr,
                                const bool unknown_as_occupied) {
            if (path.size() <= 2 || !map_ptr) {
                return;
            }

            constexpr double kSharpTurnCost = 0.5; // about 60 deg
            constexpr double kMinRoundSegment = 0.25;
            constexpr double kMaxRoundSegment = 0.8;
            constexpr double kRoundRatio = 0.35;

            vec_Vec3f rounded;
            rounded.reserve(path.size() * 2);
            rounded.push_back(path.front());

            for (size_t i = 1; i + 1 < path.size(); ++i) {
                const Vec3f &prev = rounded.back();
                const Vec3f &cur = path[i];
                const Vec3f &next = path[i + 1];
                const double in_len = (cur - prev).norm();
                const double out_len = (next - cur).norm();
                const double turn_cost = cornerCost(prev, cur, next);

                if (turn_cost < kSharpTurnCost ||
                    in_len < kMinRoundSegment * 2.0 ||
                    out_len < kMinRoundSegment * 2.0) {
                    rounded.push_back(cur);
                    continue;
                }

                const double round_len = std::min(kMaxRoundSegment,
                                                  std::max(kMinRoundSegment,
                                                           kRoundRatio * std::min(in_len, out_len)));
                const Vec3f entry = cur - (cur - prev).normalized() * round_len;
                const Vec3f exit = cur + (next - cur).normalized() * round_len;

                if (frontendLineFree(entry, exit, map_ptr, unknown_as_occupied) &&
                    frontendLineFree(prev, entry, map_ptr, unknown_as_occupied) &&
                    frontendLineFree(exit, next, map_ptr, unknown_as_occupied)) {
                    if (!samePoint(rounded.back(), entry)) {
                        rounded.push_back(entry);
                    }
                    if (!samePoint(rounded.back(), exit)) {
                        rounded.push_back(exit);
                    }
                } else {
                    rounded.push_back(cur);
                }
            }

            rounded.push_back(path.back());
            path.swap(rounded);
        }

        void greedyShortcutOnce(vec_Vec3f &path,
                                const rog_map::ROGMapROS::Ptr &map_ptr,
                                const bool unknown_as_occupied) {
            if (path.size() <= 2 || !map_ptr) {
                return;
            }

            vec_Vec3f shortcut;
            shortcut.reserve(path.size());
            shortcut.push_back(path.front());

            size_t anchor = 0;
            while (anchor + 1 < path.size()) {
                size_t next = anchor + 1;
                for (size_t candidate = path.size() - 1; candidate > anchor + 1; --candidate) {
                    if (frontendLineFree(path[anchor], path[candidate], map_ptr, unknown_as_occupied)) {
                        next = candidate;
                        break;
                    }
                }

                shortcut.push_back(path[next]);
                anchor = next;
            }

            path.swap(shortcut);
        }

        void shortcutFrontendPath(vec_Vec3f &path,
                                  const rog_map::ROGMapROS::Ptr &map_ptr,
                                  const bool unknown_as_occupied) {
            if (path.size() <= 2 || !map_ptr) {
                return;
            }

            vec_Vec3f cleaned;
            cleaned.reserve(path.size());
            for (const auto &pt: path) {
                if (cleaned.empty() || !samePoint(cleaned.back(), pt)) {
                    cleaned.push_back(pt);
                }
            }
            if (cleaned.size() <= 2) {
                path.swap(cleaned);
                return;
            }

            greedyShortcutOnce(cleaned, map_ptr, unknown_as_occupied);
            std::reverse(cleaned.begin(), cleaned.end());
            greedyShortcutOnce(cleaned, map_ptr, unknown_as_occupied);
            std::reverse(cleaned.begin(), cleaned.end());
            greedyShortcutOnce(cleaned, map_ptr, unknown_as_occupied);
            removeRedundantCorners(cleaned, map_ptr, unknown_as_occupied);
            softenSharpCorners(cleaned, map_ptr, unknown_as_occupied);
            removeRedundantCorners(cleaned, map_ptr, unknown_as_occupied);

            path.swap(cleaned);
        }
    }

    SuperPlanner::SuperPlanner
            (const std::string &cfg_path,
             const ros_interface::RosInterface::Ptr &ros_ptr,
             const rog_map::ROGMapROS::Ptr &map_ptr
            ) : cfg_(Config(cfg_path)), ros_ptr_(ros_ptr), map_ptr_(map_ptr) {//初始化 cfg_ ,ros接口和rogmap

        ros_ptr_->setResolution(cfg_.resolution);  //#ros显示分辨率
        ros_ptr_->setVisualizationEn(cfg_.visualization_en);  //开启可视化？
        exp_traj_opt_ = std::make_shared<traj_opt::ExpTrajOpt>(cfg_.exp_traj_cfg, ros_ptr_);  //ExpTraj优化器
        back_traj_opt_ = std::make_shared<traj_opt::BackupTrajOpt>(cfg_.back_traj_cfg, ros_ptr_);//backTraj优化器
        yaw_traj_opt_ = std::make_shared<traj_opt::YawTrajOpt>(cfg_.yaw_dot_max);  //yaw角优化器
        const auto &rog_map_cfg = map_ptr_->getMapConfig();//获取rogmap参数
        astar_ptr_ = std::make_shared<path_search::Astar>(cfg_path, ros_ptr_, map_ptr_); //实例化a*地图
        dstar_lite_ptr_ = std::make_shared<path_search::DStarLite>(cfg_path, ros_ptr_, map_ptr_); //实例化D* Lite地图
        cg_ptr_ = std::make_shared<CorridorGenerator>(ros_ptr_, map_ptr_, cfg_.corridor_bound_dis,  //实例化飞行走廊生成器
                                                      cfg_.corridor_line_max_length,
                                                      cfg_.resolution, rog_map_cfg.virtual_ground_height,
                                                      rog_map_cfg.virtual_ceil_height,
                                                      cfg_.robot_r,
                                                      cfg_.obs_skip_num,
                                                      cfg_.iris_iter_num);
        cg_ptr_->SetLineNeighborList(cfg_.seed_line_neighbour);  //设置种子线邻居


        time_consuming_.resize(8);  //时间消耗记录

        robot_state_.rcv = false;  //机器人状态，初始未收到
        planner_process_start_WT_ = ros_ptr_->getSimTime(); // planner启动时间
        fov_checker_ = std::make_shared<FOVChecker>(FOVType::OMNI,  //实例化fovchecker
                                                    -1.0,
                                                    -35.0,
                                                    35.0);

        const int neighbor_step = floor(cfg_.robot_r / cfg_.resolution);  //a*邻居步数
        astar_ptr_->setFineInfNeighbors(neighbor_step);  //设置a*邻居步数
        dstar_lite_ptr_->setFineInfNeighbors(neighbor_step);  //设置D* Lite邻居步数

        initializeDynamicCollisionAvoidance();
    }

    int SuperPlanner::parseUavId(const std::string &name) const {
        int id = 0;
        for (const auto &ch: name) {
            if (std::isdigit(ch)) {
                id = id * 10 + (ch - '0');
            }
        }
        return id > 0 ? id : 1;
    }

    void SuperPlanner::initializeDynamicCollisionAvoidance() {
        ros::NodeHandle pnh("~");
        pnh.param<std::string>("uav_name", uav_name_, uav_name_);
        pnh.param("swarm_uav_num", swarm_uav_num_, 4);
        pnh.param("dynamic_collision_en", dynamic_collision_en_, true);
        pnh.param("dynamic_collision_clearance", dynamic_collision_clearance_, std::max(0.3, cfg_.robot_r));
        pnh.param("dynamic_collision_stale_time", dynamic_collision_stale_time_, 0.5);
        pnh.param("dynamic_collision_sample_dt", dynamic_collision_sample_dt_, 0.2);
        pnh.param("planning_period", planning_period_, 0.1);
        pnh.param("sync_tolerance", sync_tolerance_, 0.02);
        pnh.param("max_traj_age_ms", max_traj_age_ms_, 150.0);
        double inertial_origin_x = 0.0;
        double inertial_origin_y = 0.0;
        double inertial_origin_z = 0.0;
        pnh.param("inertial_origin_x", inertial_origin_x, 0.0);
        pnh.param("inertial_origin_y", inertial_origin_y, 0.0);
        pnh.param("inertial_origin_z", inertial_origin_z, 0.0);
        pnh.param("local_to_inertial_yaw_deg", local_to_inertial_yaw_deg_, 90.0);
        inertial_origin_ = Vec3f(inertial_origin_x, inertial_origin_y, inertial_origin_z);
        const double yaw_rad = local_to_inertial_yaw_deg_ * M_PI / 180.0;
        local_to_inertial_R_ << std::cos(yaw_rad), -std::sin(yaw_rad), 0.0,
                                std::sin(yaw_rad),  std::cos(yaw_rad), 0.0,
                                0.0,                0.0,               1.0;
        uav_id_ = parseUavId(uav_name_);
        if ((!dynamic_collision_en_ && !cfg_.exp_traj_cfg.local_density_en) || swarm_uav_num_ <= 1) {
            return;
        }

        other_mpc_pred_subs_.clear();
        other_mpc_pred_subs_.reserve(std::max(0, swarm_uav_num_ - 1));
        for (int id = 1; id <= swarm_uav_num_; ++id) {
            if (id == uav_id_) {
                continue;
            }
            const std::string topic = "/uav" + std::to_string(id) + "/mpc_pred_traj";
            other_mpc_pred_subs_.push_back(
                    pnh.subscribe<ius_msgs::Trajectory>(
                            topic, 1,
                            boost::bind(&SuperPlanner::otherMpcPredCallback, this, _1, id),
                            ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));
            other_mpc_predictions_[id] = MpcPredictionCache();
        }

        ros_ptr_->info(" -- [SUPER] swarm prediction enabled for " + uav_name_ +
                       ", subscribed to " + std::to_string(other_mpc_pred_subs_.size()) +
                       " other MPC prediction topics.");
    }

    void SuperPlanner::otherMpcPredCallback(const ius_msgs::TrajectoryConstPtr &msg, int other_id) {
        std::lock_guard<std::mutex> lock(other_mpc_pred_mutex_);
        auto &cache = other_mpc_predictions_[other_id];
        cache.traj = *msg;
        cache.rcv_stamp = ros::Time::now();
        cache.received = true;
    }

    bool SuperPlanner::sampleMpcPrediction(const ius_msgs::Trajectory &traj,
                                           const double &query_wt,
                                           Vec3f &pos) const {
        if (traj.pos.empty()) {
            return false;
        }

        double rel_t = query_wt - traj.header.stamp.toSec();
        if (rel_t < 0.0) {
            rel_t = 0.0;
        }

        if (traj.time.size() != traj.pos.size()) {
            const double assumed_dt = 0.1;
            const double max_t = assumed_dt * static_cast<double>(traj.pos.size() - 1);
            if (rel_t > max_t) {
                return false;
            }
            const double idx_f = rel_t / assumed_dt;
            const size_t idx = std::min(static_cast<size_t>(std::floor(idx_f)), traj.pos.size() - 1);
            const size_t next_idx = std::min(idx + 1, traj.pos.size() - 1);
            const double alpha = std::min(1.0, std::max(0.0, idx_f - static_cast<double>(idx)));
            const auto &p0 = traj.pos[idx];
            const auto &p1 = traj.pos[next_idx];
            pos = (1.0 - alpha) * Vec3f(p0.x, p0.y, p0.z) + alpha * Vec3f(p1.x, p1.y, p1.z);
            return true;
        }

        if (rel_t > traj.time.back()) {
            return false;
        }
        if (rel_t <= traj.time.front()) {
            const auto &p = traj.pos.front();
            pos = Vec3f(p.x, p.y, p.z);
            return true;
        }

        auto upper = std::upper_bound(traj.time.begin(), traj.time.end(), rel_t);
        size_t idx = static_cast<size_t>(std::distance(traj.time.begin(), upper));
        idx = std::min(idx, traj.time.size() - 1);
        const size_t prev_idx = idx - 1;
        const double dt = std::max(1e-3, traj.time[idx] - traj.time[prev_idx]);
        const double alpha = std::min(1.0, std::max(0.0, (rel_t - traj.time[prev_idx]) / dt));
        const auto &p0 = traj.pos[prev_idx];
        const auto &p1 = traj.pos[idx];
        pos = (1.0 - alpha) * Vec3f(p0.x, p0.y, p0.z) + alpha * Vec3f(p1.x, p1.y, p1.z);
        return true;
    }

    void SuperPlanner::buildTimeAwareDynamicHyperplanes(const PolytopeVec &sfc,
                                                        const vec_E<Vec3f> &guide_path,
                                                        const vector<double> &guide_stamp,
                                                        const double &sfc_start_wt,
                                                        std::vector<MatD4f> &dynamic_hplanes) {
        dynamic_hplanes.clear();
        dynamic_hplanes.resize(sfc.size());
        for (auto &planes: dynamic_hplanes) {
            planes.resize(0, 4);
        }

        if (!dynamic_collision_en_ || sfc.empty() || guide_path.empty() || guide_stamp.size() != guide_path.size()) {
            return;
        }

        std::map<int, MpcPredictionCache> predictions;
        {
            std::lock_guard<std::mutex> lock(other_mpc_pred_mutex_);
            predictions = other_mpc_predictions_;
        }

        const double now_wt = ros::Time::now().toSec();
        std::vector<double> last_sample_t(sfc.size(), -std::numeric_limits<double>::infinity());
        int added_planes = 0;
        for (size_t k = 0; k < guide_path.size(); ++k) {
            const Vec3f self_pos = guide_path[k];
            const Vec3f self_pos_inertial = local_to_inertial_R_ * self_pos + inertial_origin_;
            int sfc_id = -1;
            for (size_t i = 0; i < sfc.size(); ++i) {
                if (sfc[i].PointIsInside(self_pos, std::max(0.05, cfg_.resolution * 2.0))) {
                    sfc_id = static_cast<int>(i);
                    break;
                }
            }
            if (sfc_id < 0) {
                continue;
            }
            if (guide_stamp[k] - last_sample_t[sfc_id] < dynamic_collision_sample_dt_ && k + 1 < guide_path.size()) {
                continue;
            }
            last_sample_t[sfc_id] = guide_stamp[k];

            const double query_wt = sfc_start_wt + guide_stamp[k];
            for (const auto &item: predictions) {
                const auto &cache = item.second;
                if (!cache.received || (now_wt - cache.rcv_stamp.toSec()) > dynamic_collision_stale_time_) {
                    continue;
                }

                Vec3f other_pos_inertial;
                if (!sampleMpcPrediction(cache.traj, query_wt, other_pos_inertial)) {
                    continue;
                }

                Vec3f nhyp = other_pos_inertial - self_pos_inertial;
                const double dist = nhyp.norm();
                if (dist < 1e-3) {
                    continue;
                }
                const Vec3f nhypnorm = nhyp / dist;
                const Vec3f zw(0.0, 0.0, 1.0);
                const Vec3f yw(0.0, 1.0, 0.0);
                const Vec3f r = nhypnorm.cross(zw) + nhypnorm.cross(yw);
                const double r_norm = r.norm();
                if (r_norm > 1e-6) {
                    const double initial_goal_dist = (gi_.goal_p - guide_path.front()).norm();
                    double progress = 1.0;
                    if (initial_goal_dist > 1e-3) {
                        const double current_goal_dist = (gi_.goal_p - self_pos).norm();
                        progress = (initial_goal_dist - current_goal_dist) / initial_goal_dist;
                        progress = std::max(0.0, std::min(1.0, progress));
                    }
                    const double c = 0.1;
                    const double perturb_k = 0.05;
                    const Vec3f npert = (c + perturb_k * (1.0 - progress)) * r / r_norm;
                    nhyp += npert;
                }
                const double nhyp_pert_norm = nhyp.norm();
                Vec3f normal = nhypnorm;
                if (nhyp_pert_norm > 1e-6) {
                    normal = nhyp / nhyp_pert_norm;
                }
                const Vec3f mid = 0.5 * (self_pos_inertial + other_pos_inertial);
                const double shift = 0.5 * std::min(2.0 * dynamic_collision_clearance_, dist);
                const Vec3f plane_point_inertial = mid - shift * normal;
                const double plane_d_inertial = -normal.dot(plane_point_inertial);
                const Vec3f normal_local = local_to_inertial_R_.transpose() * normal;
                const double plane_d_local = plane_d_inertial + normal.dot(inertial_origin_);

                MatD4f candidate(sfc[sfc_id].GetPlanes().rows() + dynamic_hplanes[sfc_id].rows() + 1, 4);
                candidate.topRows(sfc[sfc_id].GetPlanes().rows()) = sfc[sfc_id].GetPlanes();
                if (dynamic_hplanes[sfc_id].rows() > 0) {
                    candidate.middleRows(sfc[sfc_id].GetPlanes().rows(), dynamic_hplanes[sfc_id].rows()) =
                            dynamic_hplanes[sfc_id];
                }
                const int new_row = candidate.rows();
                candidate.row(new_row - 1) << normal_local.x(), normal_local.y(),
                                               normal_local.z(), plane_d_local;

                Vec3f interior;
                if (geometry_utils::findInterior(candidate, interior)) {
                    const int dyn_row = dynamic_hplanes[sfc_id].rows();
                    dynamic_hplanes[sfc_id].conservativeResize(dyn_row + 1, 4);
                    dynamic_hplanes[sfc_id].row(dyn_row) << normal_local.x(), normal_local.y(),
                                                            normal_local.z(), plane_d_local;
                    ++added_planes;
                }
            }
        }

        if (cfg_.print_log && added_planes > 0) {
            ros_ptr_->info(" -- [SUPER] Built " + std::to_string(added_planes) +
                           " time-aware dynamic collision hyperplanes from MPC predictions.");
        }
    }

    void SuperPlanner::buildSwarmPredictions(std::vector<traj_opt::SwarmPrediction> &predictions) {
        predictions.clear();
        if (!cfg_.exp_traj_cfg.local_density_en || swarm_uav_num_ <= 1) {
            return;
        }

        std::map<int, MpcPredictionCache> cached_predictions;
        {
            std::lock_guard<std::mutex> lock(other_mpc_pred_mutex_);
            cached_predictions = other_mpc_predictions_;
        }

        const double now_wt = ros::Time::now().toSec();
        for (const auto &item: cached_predictions) {
            const auto &cache = item.second;
            if (!cache.received || (now_wt - cache.rcv_stamp.toSec()) > dynamic_collision_stale_time_ ||
                cache.traj.pos.empty()) {
                continue;
            }

            traj_opt::SwarmPrediction prediction;
            prediction.start_wt = cache.traj.header.stamp.toSec();
            prediction.positions.reserve(cache.traj.pos.size());
            prediction.times.reserve(cache.traj.pos.size());
            const bool has_valid_times = cache.traj.time.size() == cache.traj.pos.size();
            double last_time = -std::numeric_limits<double>::infinity();
            for (size_t i = 0; i < cache.traj.pos.size(); ++i) {
                const auto &point = cache.traj.pos[i];
                const Vec3f inertial_position(point.x, point.y, point.z);
                const Vec3f position = local_to_inertial_R_.transpose() *
                                       (inertial_position - inertial_origin_);
                const double sample_time = has_valid_times ? cache.traj.time[i] : 0.1 * static_cast<double>(i);
                if (!position.allFinite() || !std::isfinite(sample_time) || sample_time < last_time) {
                    continue;
                }
                prediction.positions.push_back(position);
                prediction.times.push_back(sample_time);
                last_time = sample_time;
            }
            if (!prediction.positions.empty()) {
                predictions.push_back(std::move(prediction));
            }
        }
    }

    RET_CODE
    SuperPlanner::PlanFromRest(const Vec3f &goal_p,
                               const double &goal_yaw,
                               const bool &new_goal) {
        std::lock_guard<std::mutex> guard(replan_lock_);// 使用锁来确保对规划数据的安全访问，避免并发访问冲突
        latest_replan.reset(); // 重置最新的规划信息
        latest_replan.setGoal(goal_p, goal_yaw, robot_state_);
        if (robot_state_.rcv == false) { // 检查机器人是否有有效的里程计数据
            ros_ptr_->warn(" -- [SUPER] in [PlanFromRest]: No odom, force return.");
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_NO_ODOM);
            return FAILED;
        }
        gi_.goal_p = goal_p; // 设置目标位置、目标航向以及是否为新目标
        gi_.goal_yaw = goal_yaw;
        gi_.new_goal = new_goal;
        gi_.goal_valid = true;

        // 可视化目标点和当前机器人位置
        vec_Vec3f viz_pts{goal_p, robot_state_.p};
        {
            TimeConsuming t_viz("viz goal path", false);
            ros_ptr_->vizGoalPath(viz_pts);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }


        /// 1) First, shift the start_point to free space.
        /// 1) 首先，确保起始点不是被占用的区域。
        Vec3f local_star_pt;
        // 起始点不是占用
        if (!map_ptr_->getNearestCellNot(GridType::OCCUPIED, robot_state_.p, local_star_pt, 3.0)) {
            ros_ptr_->error(
                    " -- [SUPER] in [PlanFromRest] Local start point is deeply occupied, which should not happened.");
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_NO_START_POINT);
            return FAILED;
        }
        // 设置起始点
        latest_replan.setLocalStartP(local_star_pt);

        /// 2) Generate Exp traj
         /// 2) 生成扩展轨迹（Exp Trajectory）
        ExpTraj exp_traj_info; // 扩展轨迹信息
        BackupTraj back_traj_info; // 备用轨迹信息
        last_exp_traj_info_.setEmpty(); // 清空上一次的轨迹信息
        local_start_p_ = local_star_pt; // 本次规划的起始点
        RET_CODE exp_ret_code = generateExpTraj(last_exp_traj_info_, exp_traj_info);//生成探索轨迹
        //GenerateRestToRestExpTraj(local_star_pt, exp_traj_info);
        // 如果扩展轨迹生成失败，返回失败
        if (exp_ret_code == FAILED) {
            ros_ptr_->warn(" -- [SUPER] in [PlanFromRest] GenerateExpTrajectory failed with {}.",
                           RET_CODE_STR[exp_ret_code].c_str());
            return FAILED;
        } else {
            ros_ptr_->info(" -- [SUPER] in [PlanFromRest] GenerateExpTrajectory SUCCESS.");
        }
         // 生成备用轨迹
        back_traj_info.setEmpty();
        RET_CODE back_ret_code;
        if(!cfg_.backup_traj_en)
        {
            back_ret_code= FINISH;
        }
        else{
         back_ret_code = generateBackupTrajectory(exp_traj_info, back_traj_info);;
        }
   
        if (back_ret_code == SUCCESS) {  // 如果备用轨迹生成成功
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [SUPER] in [PlanFromRest] generateBackupTrajectory SUCCESS.");
            }

            if (!cmd_traj_info_.setTrajectory(exp_traj_info, back_traj_info)) { // 设置轨迹信息并更新轨迹
                ros_ptr_->warn(" -- [SUPER] in [PlanFromRest] reject invalid committed trajectory with backup.");
                return FAILED;
            }
            last_exp_traj_info_ = exp_traj_info; // 更新最后的扩展轨迹信息
            safe_stop_start_wt_ = -1.0;
            next_failed_replan_wt_ = 0.0;
            robot_on_backup_traj_ = false;  /// 标记机器人不在备用轨迹上
            gi_.new_goal = false;  //// 标记目标已处理

            // For visualization
            // 可视化已提交的轨迹
            {
                TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), cmd_traj_info_.getBackupTrajStartTT());
                time_consuming_[VISUALIZATION] += t_viz.stop();
                latest_replan.setRetCode(SUPER_RET_CODE::SUPER_SUCCESS_WITH_BACKUP);
            }

            return SUCCESS;
        } else if (back_ret_code == FINISH || back_ret_code == NO_NEED) { // 如果备用轨迹生成返回“完成”或“不需要”
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [SUPER] in [PlanFromRest] generateBackupTrajectory Finish or NO_NEED.");
            }
            robot_on_backup_traj_ = false;// 标记机器人不在备用轨迹上
            if (!cmd_traj_info_.setTrajectory(exp_traj_info)) {// 设置扩展轨迹
                ros_ptr_->warn(" -- [SUPER] in [PlanFromRest] reject invalid committed trajectory without backup.");
                return FAILED;
            }
            last_exp_traj_info_ = exp_traj_info;
            safe_stop_start_wt_ = -1.0;
            next_failed_replan_wt_ = 0.0;
            gi_.new_goal = false;

            // For visualization
            // 可视化已提交的轨迹
            TimeConsuming t_viz("viz goal VisualizeCommitTrajectory", false);
            { 
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_SUCCESS_NO_BACKUP);
            return SUCCESS;
        }
        ros_ptr_->warn(" -- [SUPER] in [PlanFromRest] generateBackupTrajectory return [{}], force return",
                       RET_CODE_STR[back_ret_code].c_str());
        return FAILED;
    }


    bool SuperPlanner::trajectorySegmentSafe(const Trajectory &traj, double from_t, double to_t) {
        if (traj.empty() || !std::isfinite(from_t) || !std::isfinite(to_t) ||
            from_t < 0.0 || to_t < from_t || to_t > traj.getTotalDuration() + 1e-6) {
            return false;
        }
        std::vector<traj_opt::SwarmPrediction> predictions;
        buildSwarmPredictions(predictions);
        Vec3f previous = traj.getPos(from_t);
        if (!previous.allFinite() || map_ptr_->isOccupied(previous)) {
            return false;
        }
        const double separation = 2.0 * cfg_.robot_r + dynamic_collision_clearance_;
        const int sample_count = std::max(1, static_cast<int>(std::ceil((to_t - from_t) / 0.05)));
        for (int i = 0; i <= sample_count; ++i) {
            const double eval_t = from_t + (to_t - from_t) * i / sample_count;
            const Vec3f position = traj.getPos(eval_t);
            if (!position.allFinite() || map_ptr_->isOccupied(position) ||
                ((position - previous).norm() > 1e-6 &&
                 !map_ptr_->isLineFree(previous, position, true, false))) {
                return false;
            }
            for (const auto &prediction: predictions) {
                Vec3f other_pos, other_vel;
                if (prediction.sample(traj.start_WT + eval_t, other_pos, other_vel) &&
                    (position - other_pos).norm() < separation) {
                    return false;
                }
            }
            previous = position;
        }
        return true;
    }

    bool SuperPlanner::committedTrajectorySafeFor(const double horizon) {
        Trajectory committed;
        cmd_traj_info_.lock();
        if (cmd_traj_info_.empty()) {
            cmd_traj_info_.unlock();
            return false;
        }
        committed = cmd_traj_info_.posTraj();
        cmd_traj_info_.unlock();
        const double from_t = ros_ptr_->getSimTime() - committed.start_WT;
        if (from_t < 0.0 || from_t + horizon > committed.getTotalDuration()) {
            return false;
        }
        if (!robot_state_.rcv ||
            (robot_state_.p - committed.getPos(from_t)).norm() > 0.5) {
            return false;
        }
        return trajectorySegmentSafe(committed, from_t, from_t + horizon);
    }

    bool SuperPlanner::commitSafeStopTrajectory() {
        const double now = ros_ptr_->getSimTime();
        if (!robot_state_.rcv || now - robot_state_.rcv_time > 0.2 ||
            !robot_state_.p.allFinite() || !robot_state_.v.allFinite() ||
            !robot_state_.a.allFinite() || !std::isfinite(robot_state_.yaw) ||
            map_ptr_->isOccupied(robot_state_.p)) {
            return false;
        }
        const double allowed_velocity = std::max(cfg_.exp_traj_cfg.max_vel * 1.05,
                                                  robot_state_.v.norm() * 1.05);
        const double allowed_acceleration = std::max(cfg_.exp_traj_cfg.max_acc * 1.05,
                                                      robot_state_.a.norm() * 1.05);

        for (const double duration: {1.5, 2.0, 2.5, 3.0, 4.0}) {
            StatePVAJ head = StatePVAJ::Zero(), tail = StatePVAJ::Zero();
            head.col(0) = robot_state_.p;
            head.col(1) = robot_state_.v;
            head.col(2) = robot_state_.a;
            tail.col(0) = robot_state_.p + 0.5 * duration * robot_state_.v;

            traj_opt::MINCO_S4NU minco;
            minco.setConditions(head, tail, 1);
            Eigen::MatrixXd no_waypoints(3, 0);
            Eigen::VectorXd times(1);
            times[0] = duration;
            minco.setParameters(no_waypoints, times);
            Trajectory stop_pos;
            minco.getTrajectory(stop_pos);
            stop_pos.start_WT = now;

            bool dynamics_safe = !stop_pos.empty();
            for (double t = 0.0; dynamics_safe && t <= duration + 1e-6; t += 0.02) {
                const double eval_t = std::min(t, duration);
                const Vec3f vel = stop_pos.getVel(eval_t);
                const Vec3f acc = stop_pos.getAcc(eval_t);
                const Vec3f jerk = stop_pos.getJer(eval_t);
                dynamics_safe = vel.allFinite() && acc.allFinite() && jerk.allFinite() &&
                        vel.norm() <= allowed_velocity &&
                        acc.norm() <= allowed_acceleration &&
                        jerk.norm() <= cfg_.exp_traj_cfg.max_jerk * 1.05;
            }
            if (!dynamics_safe || !trajectorySegmentSafe(stop_pos, 0.0, duration)) {
                continue;
            }

            Eigen::Matrix<double, 3, 8> yaw_coeff = Eigen::Matrix<double, 3, 8>::Zero();
            yaw_coeff(0, 7) = robot_state_.yaw;
            Trajectory stop_yaw;
            stop_yaw.emplace_back(duration, yaw_coeff);
            stop_yaw.start_WT = now;
            ExpTraj stop_exp;
            stop_exp.setTrajectory(now, stop_pos, stop_yaw);
            if (!cmd_traj_info_.setTrajectory(stop_exp)) {
                return false;
            }
            last_exp_traj_info_ = stop_exp;
            safe_stop_start_wt_ = now;
            consecutive_exp_replan_fail_count_ = 0;
            next_failed_replan_wt_ = now + 0.5;
            latest_replan.setRetCode(SUPER_SUCCESS_NO_BACKUP);
            ros_ptr_->warn(" -- [SUPER] Replan failed near trajectory end; committed a checked {:.1f}s braking trajectory.",
                           duration);
            return true;
        }
        return false;
    }

    RET_CODE
    SuperPlanner::ReplanOnce(const Vec3f &goal_p,
                             const double &goal_yaw,
                             const bool &new_goal) {
        TimeConsuming replan_total_t("ReplanOnce", false);//记录重规划总时间
        std::lock_guard<std::mutex> guard(replan_lock_); //线程锁

        const bool goal_changed = (goal_p - gi_.goal_p).norm() > 1e-3;
        gi_.goal_p = goal_p;// 设置目标位置、目标航向以及是否为新目标
        gi_.goal_yaw = goal_yaw;
        gi_.new_goal = new_goal;
        gi_.goal_valid = true;
        latest_replan.reset();
        latest_replan.setGoal(goal_p, goal_yaw, robot_state_);

        const double replan_now = ros_ptr_->getSimTime();
        if (!goal_changed && replan_now < next_failed_replan_wt_) {
            return FAILED;
        }

        // 可视化目标点和当前机器人位置
        vec_Vec3f viz_pts{goal_p, robot_state_.p};

        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizGoalPath(viz_pts);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        bool force_plan_from_actual = false;
        double tracking_error = 0.0;
        if (cfg_.tracking_drift_recovery_en && robot_state_.rcv && !last_exp_traj_info_.empty()) {
            cmd_traj_info_.lock();
            if (!cmd_traj_info_.empty() && !cmd_traj_info_.posTraj().empty()) {
                const double total_dur = cmd_traj_info_.getTotalDuration();
                if (total_dur > 1e-6 && std::isfinite(total_dur)) {
                    double eval_t = ros_ptr_->getSimTime() - cmd_traj_info_.getStartWallTime();
                    eval_t = std::max(0.0, std::min(eval_t, total_dur));
                    const Vec3f ref_p = cmd_traj_info_.posTraj().getPos(eval_t);
                    tracking_error = (robot_state_.p - ref_p).norm();
                    force_plan_from_actual = tracking_error > std::max(1.0, cfg_.robot_r * 4.0);
                }
            }
            cmd_traj_info_.unlock();
        }

        ExpTraj planning_reference = last_exp_traj_info_;
        const Vec3f previous_local_start_p = local_start_p_;
        if (force_plan_from_actual) {
            Vec3f local_start_pt;
            if (!map_ptr_->getNearestCellNot(GridType::OCCUPIED, robot_state_.p, local_start_pt, 3.0)) {
                ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: tracking drift {}, but no free restart point.",
                               tracking_error);
                return FAILED;
            }
            ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: tracking drift {} m, restart planning from actual state.",
                           tracking_error);
            planning_reference.setEmpty();
            local_start_p_ = local_start_pt;
        }


        /// 1) Replan EXP traj
        //生成期望轨迹
        ExpTraj exp_traj_info;
        TimeConsuming t_exp("t_exp", false);
        RET_CODE exp_ret_code = generateExpTraj(planning_reference, exp_traj_info);
        time_consuming_[GENERATE_EXP_TRAJ] = t_exp.stop();

        if (exp_ret_code == FAILED) {
            local_start_p_ = previous_local_start_p;
            ++consecutive_exp_replan_fail_count_;
            ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: GenerateExpTrajectory failed {}/{}.",
                           consecutive_exp_replan_fail_count_,
                           max_consecutive_exp_replan_failures_);

            // Keep the previous checked path while it has a safe horizon.
            // Repeatedly discarding it creates a new start state every few
            // timer ticks and can make the vehicle reverse direction.
            if (committedTrajectorySafeFor(1.0)) {
                next_failed_replan_wt_ = replan_now + 0.3;
                return FAILED;
            }

            cmd_traj_info_.lock();
            const bool already_braking = !cmd_traj_info_.empty() &&
                    std::abs(cmd_traj_info_.getStartWallTime() - safe_stop_start_wt_) < 1e-4;
            cmd_traj_info_.unlock();
            if (already_braking) {
                next_failed_replan_wt_ = replan_now + 0.3;
                return FAILED;
            }
            if (commitSafeStopTrajectory()) {
                return SUCCESS;
            }

            if (cfg_.tracking_drift_recovery_en &&
                consecutive_exp_replan_fail_count_ > max_consecutive_exp_replan_failures_ &&
                (last_recovery_wt_ < 0.0 || replan_now - last_recovery_wt_ >= 1.0)) {
                last_recovery_wt_ = replan_now;
                Vec3f local_start_pt;
                if (!robot_state_.rcv ||
                    !map_ptr_->getNearestCellNot(GridType::OCCUPIED, robot_state_.p, local_start_pt, 3.0)) {
                    ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: repeated exp failures, but no free restart point.");
                    consecutive_exp_replan_fail_count_ = 0;
                    return FAILED;
                }

                ros_ptr_->warn(
                        " -- [SUPER] in [ReplanOnce]: repeated exp failures, restart planning from actual state.");
                const ExpTraj previous_exp_traj = last_exp_traj_info_;
                const Vec3f previous_start_p = local_start_p_;
                last_exp_traj_info_.setEmpty();
                local_start_p_ = local_start_pt;

                ExpTraj recovery_exp_traj_info;
                TimeConsuming t_recovery_exp("t_recovery_exp", false);
                exp_ret_code = generateExpTraj(last_exp_traj_info_, recovery_exp_traj_info);
                time_consuming_[GENERATE_EXP_TRAJ] += t_recovery_exp.stop();
                if (exp_ret_code == FAILED) {
                    last_exp_traj_info_ = previous_exp_traj;
                    local_start_p_ = previous_start_p;
                    ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: recovery GenerateExpTrajectory failed, force return");
                    consecutive_exp_replan_fail_count_ = 0;
                    next_failed_replan_wt_ = replan_now + 0.5;
                    return FAILED;
                } else if (exp_ret_code == NEW_TRAJ) {
                    consecutive_exp_replan_fail_count_ = 0;
                    return NEW_TRAJ;
                } else if (exp_ret_code == EMER) {
                    ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: recovery replan failed, switch to emer.");
                    consecutive_exp_replan_fail_count_ = 0;
                    return EMER;
                }
                exp_traj_info = recovery_exp_traj_info;
                consecutive_exp_replan_fail_count_ = 0;
            } else {
                next_failed_replan_wt_ = replan_now + 0.3;
                return FAILED;
            }
        } else if (exp_ret_code == NEW_TRAJ) {
            consecutive_exp_replan_fail_count_ = 0;
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: Last epx traj end, switch to new traj.");
            }
            return NEW_TRAJ;
        } else if (exp_ret_code == EMER) {
            ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: Replan failed, switch to emer.");
            return EMER;
        } else if (exp_ret_code == SUCCESS) {
            consecutive_exp_replan_fail_count_ = 0;
            next_failed_replan_wt_ = 0.0;
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: Replan a new exp traj success.");
            }
        } else if (exp_ret_code == NO_NEED) {
            consecutive_exp_replan_fail_count_ = 0;
            if (cfg_.print_log)
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: No need to replan a new exp traj, use last one.");
        }

        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizYawTraj(exp_traj_info.posTraj(), exp_traj_info.yawTraj());
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }


        BackupTraj back_traj_info;
        // 2）生成back轨迹
        //生成备用轨迹
        TimeConsuming t_back("t_back", false);
        RET_CODE back_ret_code;
        if(!cfg_.backup_traj_en)
        {
            back_ret_code = FINISH;
        }
        else {
             back_ret_code = generateBackupTrajectory(exp_traj_info, back_traj_info);
            }
        
        
        
        time_consuming_[GENERATE_BACK_TRAJ] = t_back.stop();

        {
            ft += time_consuming_[EPX_TRAJ_FRONTEND] + time_consuming_[BACK_TRAJ_FRONTEND];
            ft_cnt++;
            bt += time_consuming_[BACK_TRAJ_OPT] + time_consuming_[EXP_TRAJ_OPT];
            bt_cnt++;
        }

        double replan_dt = replan_total_t.stop();//检查重规划时间
        if (replan_dt > cfg_.max_replan_time) {
            ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: Replan overtime, check parameters, replan dt = {}, max_replan_time = {}.",
                           replan_dt, cfg_.max_replan_time);
            return FAILED;
        }



        if (back_ret_code == SUCCESS) { //成功 (SUCCESS)：设定 exp_traj_info 和 back_traj_info 作为最终轨迹。
            if (!cmd_traj_info_.setTrajectory(exp_traj_info, back_traj_info)) {
                ros_ptr_->warn(" -- [SUPER] in [ReplanOnce] reject invalid committed trajectory with backup.");
                return FAILED;
            }
            last_exp_traj_info_ = exp_traj_info;
            safe_stop_start_wt_ = -1.0;
            next_failed_replan_wt_ = 0.0;
            robot_on_backup_traj_ = false;
            gi_.new_goal = false;

            {
                // For visualization
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), cmd_traj_info_.getBackupTrajStartTT());
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }

            latest_replan.setRetCode(SUPER_SUCCESS_WITH_BACKUP);
            if (cfg_.print_log)
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: Replan a new back traj success, all replan success.");
            return SUCCESS;
        } else if (back_ret_code == NO_NEED) { //需要 BackupTraj (NO_NEED)：直接使用 ExpTraj，并返回 SUCCESS。
            // 这次生成backup轨迹的点没有意义,
            robot_on_backup_traj_ = false;
            last_exp_traj_info_ = exp_traj_info;
            safe_stop_start_wt_ = -1.0;
            next_failed_replan_wt_ = 0.0;
            gi_.new_goal = false;


            {
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();

            }

            if (cfg_.print_log)
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: No need back traj success, all replan success.");
            latest_replan.setRetCode(SUPER_SUCCESS_NO_BACKUP);
            return SUCCESS;
        } else if (back_ret_code == FINISH) {
            // Which means the exp traj is all in known free, no need for backup traj
            if (!cmd_traj_info_.setTrajectory(exp_traj_info)) {
                ros_ptr_->warn(" -- [SUPER] in [ReplanOnce] reject invalid committed trajectory without backup.");
                return FAILED;
            }
            last_exp_traj_info_ = exp_traj_info;
            safe_stop_start_wt_ = -1.0;
            next_failed_replan_wt_ = 0.0;
            robot_on_backup_traj_ = false;
            gi_.new_goal = false;

            {
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizCommittedTraj(cmd_traj_info_.posTraj(), -1);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }

            if (cfg_.print_log)
                ros_ptr_->info(" -- [SUPER] in [ReplanOnce]: No need back traj success, all replan success.");
            latest_replan.setRetCode(SUPER_SUCCESS_NO_BACKUP);
            return SUCCESS;
        }
        ros_ptr_->warn(" -- [SUPER] in [ReplanOnce]: generateBackupTrajectory return {}, replan Failed return",
                       RET_CODE_STR[back_ret_code].c_str());
        return FAILED;
    }

    //这个函数计算了轨迹的当前执行时间，并判断是否已经完成轨迹执行。
    //如果轨迹完成，traj_finish 被设置为 true。
    //同时，如果存在备用轨迹并且当前时间已经超过备用轨迹的开始时间，
    //它会设置 robot_on_backup_traj_ 为 true
    void SuperPlanner::getOneHeartbeatTime(double &start_WT_pos, bool &traj_finish) {
        cmd_traj_info_.lock();
        if (cmd_traj_info_.empty() || cmd_traj_info_.posTraj().empty()) {
            start_WT_pos = ros_ptr_->getSimTime();
            traj_finish = true;
            robot_on_backup_traj_ = false;
            cmd_traj_info_.unlock();
            return;
        }

        double eval_t = (ros_ptr_->getSimTime() - cmd_traj_info_.getStartWallTime());//计算从轨迹开始时间到当前时间的时间差，表示轨迹执行的进度。
        traj_finish = false;
        double total_dur = cmd_traj_info_.getTotalDuration();//获取轨迹的总持续时间。
        if (total_dur <= 1e-6 || !std::isfinite(total_dur)) {
            start_WT_pos = cmd_traj_info_.getStartWallTime();
            traj_finish = true;
            robot_on_backup_traj_ = false;
            cmd_traj_info_.unlock();
            return;
        }
        if (eval_t > total_dur) {//如果当前时间已经超过轨迹的总时长
            traj_finish = true;
            eval_t = total_dur;
        }
        start_WT_pos = cmd_traj_info_.getStartWallTime(); //轨迹的开始时间
        if (cmd_traj_info_.backupTrajAvilibale() && eval_t > cmd_traj_info_.getBackupTrajStartTT()) {//判断是否启用了备用轨迹
            robot_on_backup_traj_ = true;
        } else {
            robot_on_backup_traj_ = false;
        }
        cmd_traj_info_.unlock();
    }

    Trajectory SuperPlanner::getCommittedPositionTrajectory() {
        return cmd_traj_info_.posTraj();
    }

    Trajectory SuperPlanner::getCommittedYawTrajectory() {
        return cmd_traj_info_.yawTraj();
    }

    bool SuperPlanner::startPrestreamedTrajectory() {
        std::lock_guard<std::mutex> guard(replan_lock_);
        if (last_exp_traj_info_.empty()) {
            return false;
        }
        const double start_wt = ros_ptr_->getSimTime();
        if (!cmd_traj_info_.resetStartWallTime(start_wt)) {
            return false;
        }
        last_exp_traj_info_.resetStartWallTime(start_wt);
        consecutive_exp_replan_fail_count_ = 0;
        return true;
    }


    void SuperPlanner::getOneCommandFromTraj(StatePVAJ &pvaj,
                                             double &yaw,
                                             double &yaw_dot,
                                             bool &on_backup_traj,
                                             bool &traj_finish) {
        //从轨迹信息中提取出当前时刻的飞行命令                                        
        cmd_traj_info_.lock();//锁定 
        if (cmd_traj_info_.empty() || cmd_traj_info_.posTraj().empty()) {
            pvaj.setZero();
            pvaj.col(0) = robot_state_.p;
            yaw = robot_state_.yaw;
            yaw_dot = 0.0;
            on_backup_traj = false;
            traj_finish = true;
            robot_on_backup_traj_ = false;
            cmd_traj_info_.unlock();
            return;
        }

        const double &cur_t = ros_ptr_->getSimTime();//当前时间
        const double &cmd_start_WT = cmd_traj_info_.getStartWallTime();//轨迹的开始时间
//        const bool &backup_avilibale = cmd_traj_info_.backupTrajAvilibale();
//        const double &backup_start_TT = cmd_traj_info_.getBackupTrajStartTT();
        const double &total_dur = cmd_traj_info_.getTotalDuration();//轨迹的总持续时间
        if (total_dur <= 1e-6 || !std::isfinite(total_dur)) {
            pvaj.setZero();
            pvaj.col(0) = robot_state_.p;
            yaw = robot_state_.yaw;
            yaw_dot = 0.0;
            on_backup_traj = false;
            traj_finish = true;
            robot_on_backup_traj_ = false;
            cmd_traj_info_.unlock();
            return;
        }

        traj_finish = (cur_t - cmd_start_WT) > total_dur;// 判断轨迹是否完成
        const double &eval_t = traj_finish ? total_dur : (cur_t - cmd_start_WT);

//        bool last_round_robot_on_backup_traj = robot_on_backup_traj_;
        robot_on_backup_traj_ = cmd_traj_info_.isTTOnBackupTraj(eval_t);// 判断是否使用备用轨迹
        on_backup_traj = robot_on_backup_traj_;

        if (!cmd_traj_info_.posTraj().getState(eval_t, pvaj)) {
            pvaj.setZero();
            pvaj.col(0) = robot_state_.p;
            yaw = robot_state_.yaw;
            yaw_dot = 0.0;
            on_backup_traj = false;
            traj_finish = true;
            robot_on_backup_traj_ = false;
            cmd_traj_info_.unlock();
            return;
        }


        /// Get Yaw planning
        static double last_yaw = robot_state_.yaw;

        if (cmd_traj_info_.yawTraj().empty()) {
            yaw = last_yaw;
            yaw_dot = 0.0;
        } else {
            yaw = cmd_traj_info_.getYaw((eval_t))[0];//获取航向角（Yaw）和航向角变化率（Yaw dot
            yaw_dot = cmd_traj_info_.getYawRate((eval_t))[0];
        }

        if (isnan(yaw)) {
            yaw = last_yaw;
            yaw_dot = 0;
        } else {
            last_yaw = yaw;
        }
        if (isnan(yaw_dot)) {
            yaw_dot = 0;
        }

//        if (last_round_robot_on_backup_traj != robot_on_backup_traj_) {
//            if (last_round_robot_on_backup_traj) {
//                ros_ptr_->info(" -- [CMD] Emergency Stop End ========================");
//            } else {
//                ros_ptr_->info(" -- [CMD] Emergency Stop Start ========================");
//            }
//        }

//        double cur_yaw = geometry_utils::get_yaw_from_quaternion(robot_state_.q);
        cmd_traj_info_.unlock();//解锁
    }


    void SuperPlanner::getModuleTimeConsuming(vector<double> &time) {
        time = time_consuming_;
        std::fill(time_consuming_.begin(), time_consuming_.end(), 0);
    }


    RET_CODE SuperPlanner::generateExpTraj(ExpTraj &last_exp_traj_info, ExpTraj &out_exp_traj_info) {
        /* 1) Log the exp traj frontend time*/
        TimeConsuming t_exp_frontend("t_exp_frontend", false);//计算时间

        // use hot init or not, just prepare a guide path, a guide t, init and fina state and sfc for exp traj opt
        //变量初始化
        StatePVAJ pos_init_state, pos_fina_state;  //轨迹起始状态和最终状态 
        PolytopeVec sfc;  //存储安全飞行走廊（Safe Flight Corridor）。
        vec_Vec3f guide_path;  //存储引导路径点序列
        // the guide_stamp saves a TT
        vector<double> guide_stamp; //存储引导路径对应的时间戳。
        double guide_path_end_vel{0.0};//引导路径终点的速度。
        int reserve_size = cfg_.planning_horizon / cfg_.resolution * 1.2;  //保存路径大小
        guide_path.reserve(reserve_size);
        guide_stamp.reserve(reserve_size);//预分配 guide_path 和 guide_stamp 的存储空间

        Vec4f init_yaw{robot_state_.yaw, 0, 0, 0};
        Vec4f fina_yaw{0, 0, 0, 0};


        // alias for last_exp_traj_info
        Trajectory guide_pos_traj, guide_yaw_traj, last_exp_traj;  ///轨迹别名

        // record the wall time (WT) and the trajectory time (TT) at the start of the replan.
        //记录重规划起始时间
        const double replan_process_start_WT = ros_ptr_->getSimTime();
        double replan_process_start_TT, replan_state_TT;//重规划状态的轨迹时间。（计算完成）

        /* 2) Check last exp traj */
        //处理上一条期望轨迹
        if (last_exp_traj_info.empty()) {//如果 last_exp_traj_info 为空
            /* 2.1) Perform rest2rest exp traj generation */  //轨迹为空，说明需要从静止状态（Rest2Rest）重新规划
            // just skip the first part of the guide trajectory
            pos_init_state.setZero();  //清空状态信息。
            pos_init_state.col(0) = local_start_p_;  //将起始位置设为当前局部起点
            replan_process_start_TT = -1;  //-1不考虑上一条轨迹的时间。
            replan_state_TT = -1;
        } else {//如果 last_exp_traj_info 不为空
            guide_pos_traj = cmd_traj_info_.posTraj(); // last_exp_traj;
            guide_yaw_traj = cmd_traj_info_.yawTraj(); //last_exp_traj_info.exp_yaw_traj;
            last_exp_traj = last_exp_traj_info.posTraj();//从 cmd_traj_info_ 取出上一条轨迹的位置信息 posTraj() 和偏航信息 yawTraj()

            replan_process_start_TT = replan_process_start_WT - last_exp_traj.start_WT;
            replan_state_TT = replan_process_start_TT + cfg_.replan_forward_dt;  //计算 replan_state_TT：重规划时间点再往前推进一小段 
            /* 2.2) Perform collision check on last exp traj*/
            vector<TimePosPair> last_exp_traj_time_pos;
            vector<double> last_exp_traj_vel;


            // check early exit condition  //早退出条件（Early Exit）
            // 1) if the replan state is beyond the last cmd traj, return NO_NEED
            //如果 replan_state_TT 超过当前命令轨迹 cmd_traj_info_ 的总时长，则说明无需重规划，直接返回 NO_NEED 或 FAILED
            if (replan_state_TT >= cmd_traj_info_.getTotalDuration()) {
                out_exp_traj_info = last_exp_traj_info;

                if (robot_on_backup_traj_) {
                    if (cfg_.print_log)
                        ros_ptr_->warn(
                                " -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                    return FAILED;
                }

                if (cfg_.print_log) {
                    ros_ptr_->warn(
                            " -- [generateExpTraj] replan_state_TT >= cmd_traj_info_.pos_traj.getTotalDuration(), return NONEED and wait for plan form rest.");
                }
                return NO_NEED;
            }

            if (!last_exp_traj_info.empty()) {
                if (replan_state_TT >= last_exp_traj.getTotalDuration()) {//检查 replan_state_TT 是否超过 last_exp_traj 的总时长：
                    out_exp_traj_info = last_exp_traj_info;
                    if (cfg_.print_log)
                        ros_ptr_->warn(
                                " -- [generateExpTraj] replan_state_TT >= last_exp_traj.getTotalDuration(), return NONEED and wait for plan form rest.");
                    if (robot_on_backup_traj_) {
                        if (cfg_.print_log)
                            ros_ptr_->warn(
                                    " -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                        return FAILED;
                    } else {
                        return NO_NEED;
                    }
                }

                /// 1) Check a series of early termination conditions.
                // 终止条件：轨迹与目标点连接
                //如果轨迹只有一个安全飞行走廊 (SFCSize() == 1) 且已连接到目标：
                //说明轨迹已经足够好，不需要新的规划。
                bool has_fresh_dynamic_prediction = false;
                if (dynamic_collision_en_) {
                    const double now_wt = ros::Time::now().toSec();
                    std::lock_guard<std::mutex> lock(other_mpc_pred_mutex_);
                    for (const auto &item: other_mpc_predictions_) {
                        const auto &cache = item.second;
                        if (cache.received && (now_wt - cache.rcv_stamp.toSec()) <= dynamic_collision_stale_time_) {
                            has_fresh_dynamic_prediction = true;
                            break;
                        }
                    }
                }

                if (!gi_.new_goal && last_exp_traj_info.getSFCSize() == 1 && last_exp_traj_info.connectedToGoal()
                    && !has_fresh_dynamic_prediction) {
                    if (cfg_.print_log) {
                        ros_ptr_->warn(
                                " -- [SUPER] Replan, last exp have only one corridor and connected to goal return NONEED.");
                    }

                    out_exp_traj_info = last_exp_traj_info;
                    if (robot_on_backup_traj_) {
                        if (cfg_.print_log)
                            ros_ptr_->warn(
                                    " -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                        return FAILED;
                    } else {
                        return NO_NEED;
                    }
                }
                if (!gi_.new_goal && last_exp_traj_info.getSFCSize() == 1 && last_exp_traj_info.connectedToGoal()
                    && has_fresh_dynamic_prediction && cfg_.print_log) {
                    ros_ptr_->info(
                            " -- [SUPER] Replan, last exp is connected to goal, but fresh dynamic predictions exist, continue SFC cutting.");
                }

                if (!gi_.new_goal &&
                    (gi_.goal_p - last_exp_traj.getPos(replan_state_TT)).norm() < cfg_.resolution * 3) {//终止条件：接近目标点
                    // Return if the traj close to goal                 //如果轨迹点距离目标点 goal_p 小于 3 * cfg_.resolution：
                    out_exp_traj_info = last_exp_traj_info;
                    out_exp_traj_info.setGoalConnectedFlag(true);

                    ros_ptr_->warn(" -- [SUPER] Replan, close to goal and return NONEED.");
                    if (robot_on_backup_traj_) {
                        ros_ptr_->warn(
                                " -- [SUPER] Replan, emergency stop, return FAILED and wait for plan form rest.");
                        return FAILED;
                    } else {
                        return NO_NEED;
                    }
                }
            }
            /// Ready for replan.
            out_exp_traj_info.setGoalConnectedFlag(false);// 设置 replan 标志，新的轨迹不一定连接到目标。

            // * 2) Check if in backup trajectory. While in backup trajectory,
            // *    the guide trajectory should be a part of cmd trajectory.
            // TODO: Why cannot directly replan on cmd traj? 241121

            // * 3) Perform collision check on the guide trajectory.   //碰撞检测
            // TODO 0929 critical change for hot init.
            double eval_t = replan_state_TT; //replan_process_start_TT;  //表示从 replan_state_TT 时刻开始检测轨迹安全性。
            double guide_pos_traj_total_time = guide_pos_traj.getTotalDuration();//获取引导轨迹的总时间

            Vec3f temp_pt, last_sample_pt;
            last_exp_traj_time_pos.clear();//清空存储上一次轨迹的位置信息。
            last_exp_traj_info.setWholeTrajKnownFreeFlag(true); //假设整个轨迹是安全的。
            last_sample_pt = guide_pos_traj.getPos(eval_t);  //获取 eval_t 时刻的轨迹位置
            eval_t += cfg_.sample_traj_dt;
            // * 4) 记录replan点在evaluated_pts上的id
            int replan_id = -1;
            for (; eval_t < guide_pos_traj_total_time; eval_t += cfg_.sample_traj_dt) {//遍历轨迹进行碰撞检测//sample_traj_dt = resolution / exp_traj_cfg.max_vel;
                temp_pt = guide_pos_traj.getPos(eval_t);
                if ((temp_pt - last_sample_pt).norm() < cfg_.resolution * 0.8) {//两个相邻点之间的距离过小（小于 0.8 * cfg_.resolution），跳过该点，以减少计算量
                    continue;
                }

                rog_map::GridType temp_grid = map_ptr_->getInfGridType(temp_pt);

                if (temp_grid == rog_map::GridType::OCCUPIED || temp_grid == rog_map::GridType::OUT_OF_MAP) {
                    last_exp_traj_info.setWholeTrajKnownFreeFlag(false);//示轨迹存在障碍物，终止检测。
                    break;
                }
                if (eval_t > replan_state_TT && replan_id == -1) {
                    replan_id = last_exp_traj_time_pos.size();
                }
                last_exp_traj_time_pos.emplace_back(eval_t, temp_pt);
                last_exp_traj_vel.emplace_back(guide_pos_traj.getVel(eval_t).norm());  //录轨迹点时间 eval_t 及位置 temp_pt，并存储速度信息。
                last_sample_pt = temp_pt;
            }


            // * 7）Begin replan process, first get the replan state from the committed trajectory.
            if (!guide_pos_traj.getState(replan_state_TT, pos_init_state)) {//选择重规划起点
                ros_ptr_->warn(" -- [SUPER] Invalid traj or eval t");
                return FAILED;
            }
            // * Generate guide path with time stampe, for hot trajectory initialization
            // * the guide stamp is time from the replan start t
            //* 生成带时间戳的引导路径，用于热轨迹初始化* 引导戳是从重新规划开始的时间
            //生成新的引导路径
            guide_stamp.clear();
            guide_path.clear();
            // Disable old trajectory reuse for frontend guide generation. Each replan starts
            // from the forward-projected committed state, then searches a fresh D* path.
            guide_path.push_back(pos_init_state.col(0));
            guide_stamp.push_back(0.0);
            last_exp_traj_time_pos.clear();
            last_exp_traj_time_pos.emplace_back(replan_state_TT, pos_init_state.col(0));
            guide_path_end_vel = robot_state_.v.norm();
        }

        // second, geometry part of the guide path
        ///=================The Second Part of Guide Path ================================================

        double guide_path_length = geometry_utils::computePathLength(guide_path);
        double temp_horizon = cfg_.planning_horizon - guide_path_length;//计算剩余的规划距离（还可以扩展的路径长度）

        vector<int> path_passed_waypoint_id;
        vec_Vec3f inside_poly_goals;
        vector<int> sfc_waypoint_ids;

        if (guide_path.empty() ||
            ((guide_path.front() - pos_init_state.col(0)).norm() > 1e-2)) {
            //如果路径为空，或者起点和当前状态不一致，则将当前位置插入路径的起点，时间戳为 0
            guide_path.insert(guide_path.begin(), pos_init_state.col(0));
            guide_stamp.insert(guide_stamp.begin(), 0.0);
        }

        // if need a geometry path
        if (temp_horizon > cfg_.resolution * 2) {//如果剩余的规划距离大于两倍的地图分辨率，才考虑扩展路径
            /// start point TT + exp_traj start_WT
//            double path_search_start_point_WT = guide_stamp.back() + guide_pos_traj.start_WT;
            // if the goal is close to the last point of the guide path, just add the goal to the guide path
            if ((guide_path.back() - gi_.goal_p).norm() < cfg_.resolution * 5) {//检查目标点是否已经很近
                guide_stamp.push_back(guide_stamp.back() +
                                      (guide_path.back() - gi_.goal_p).norm() / cfg_.exp_traj_cfg.max_vel);
                guide_path.push_back(gi_.goal_p);
                // NO NEED
            } else {
                vec_Vec3f new_path;
                // project goal within the planning horizon
//                const Vec3f dir = (gi_.goal_p - robot_state_.p).normalized();
//                const double dis2goal = (gi_.goal_p - robot_state_.p).norm();
//                Vec3f cadi_p = gi_.goal_p;
//                if(dis2goal > cfg_.planning_horizon) {
//                    double proj_l = cfg_.planning_horizon;
//                    Vec3f cadi_p = robot_state_.p + dir * proj_l;
//                    int max_iter = 100;
//                    while(map_ptr_->isOccupiedInflate(cadi_p) && max_iter-- > 0) {
//                        if(map_ptr_->getNearestInfCellNot(OCCUPIED, cadi_p, cadi_p, 1.0)) {
//                            break;
//                        }
//                        proj_l -= 2.0;
//                        if(proj_l < 1){
//                            ros_ptr_->warn(" -- [SUPER] Project goal failed");
//                            gi_.goal_valid = false;
//                            return FAILED;
//                        }
//                        cadi_p = robot_state_.p + dir * proj_l;
//                    }
//                    if(max_iter <= 0) {
//                        ros_ptr_->warn(" -- [SUPER] Project goal failed");
//                        gi_.goal_valid = false;
//                        return FAILED;
//                    }
//                }
                //路径搜索扩展A*
                if (!PathSearch(guide_path.back(), gi_.goal_p, temp_horizon, new_path)) {
                    ros_ptr_->warn(" -- [SUPER] PathSearch for new path failed");
                    return FAILED;
                }
                vec_Vec3f deduped_path;
                deduped_path.reserve(new_path.size());
                for (const auto &pt: new_path) {
                    if (deduped_path.empty() || !samePoint(deduped_path.back(), pt)) {
                        deduped_path.push_back(pt);
                    }
                }
                new_path.swap(deduped_path);
                if (new_path.size() < 2) {
                    ros_ptr_->warn(" -- [SUPER] PathSearch for new path failed");
                    return FAILED;
                }

                // compute the accumulated path distance from the current guide tail
                double total_dis{0.0};
                vector<double> dis(new_path.size(), 0.0);
                Vec3f last_p = guide_path.back();
                for (size_t i = 0; i < new_path.size(); ++i) {
                    const auto d = (new_path[i] - last_p).norm();
                    total_dis += d;
                    dis[i] = total_dis;
                    last_p = new_path[i];
                }
//                for (int i = 0; i < dis.size(); i++) {
//                    cout << dis[i] << " ";
//                }
//                cout << endl;
                //时间分配
                vector<double> stamps(new_path.size(), 0);
                for (size_t i = 0; i < dis.size(); ++i) {
                    double vel;//距离分布和限制速度/加速度
                    geometry_utils::simplePMTimeAllocator(cfg_.exp_traj_cfg.max_acc, cfg_.exp_traj_cfg.max_vel,
                                                          guide_path_end_vel,
                                                          total_dis,
                                                          dis[i],
                                                          stamps[i],
                                                          vel);
                }
                double time_stamp = guide_stamp.back();

//                for (int i = 0; i < stamps.size(); i++) {
//                    cout << stamps[i] << " ";
//                }
//                cout << endl;
//
//                for (int i = 0; i < dt.size(); i++) {
//                    cout << dt[i] << " ";
//                }
//                cout << endl;
                //拼接路径 将 new_path 附加到 guide_path，并记录对应的时间戳
                for (long unsigned int i = 1; i < new_path.size(); i++) {
                    if ((new_path[i] - guide_path.back()).norm() < 1.0e-6) {
                        continue;
                    }
                    double t = stamps[i] - stamps[i - 1];
                    if (!std::isfinite(t) || t <= 0.0) {
                        t = (new_path[i] - new_path[i - 1]).norm() /
                            std::max(cfg_.exp_traj_cfg.max_vel, 1.0e-3);
                    }
                    time_stamp += t;
                    guide_path.emplace_back(new_path[i]);
                    guide_stamp.emplace_back(time_stamp);
                }
            }
        }
        //断目标是否“连接上了”
        const bool connected_goal = (guide_path.back().head(2) - gi_.goal_p.head(2)).norm() < cfg_.resolution * 2;
        out_exp_traj_info.setGoalConnectedFlag(connected_goal);

        sfc.clear();//构造 SFC（Safe Flight Corridor
        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizFrontendPath(guide_path);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }
        shifted_sfc_start_pt_ = Vec3f(9999,9999,9999);
        bool bool_ret_code = cg_ptr_->SearchPolytopeOnPath(guide_path, sfc, shifted_sfc_start_pt_, cfg_.use_fov_cut);
        //调用 SFC 构造函数 SearchPolytopeOnPath，输入 guide_path 输出 corridor sfc
        //shifted_sfc_start_pt_ 用来记录偏移后的 corridor 起点（一般用于对接轨迹）

        if (!bool_ret_code) {
            ros_ptr_->warn(" -- [SUPER] SearchPolytopeOnPath for new path failed");
            return FAILED;//构造失败就报错退出
        }
        double dynamic_sfc_start_wt = replan_process_start_WT;
        if (!last_exp_traj_info.empty()) {
            dynamic_sfc_start_wt = guide_pos_traj.start_WT + replan_state_TT;
        }
        std::vector<MatD4f> dynamic_hplanes;
        buildTimeAwareDynamicHyperplanes(sfc, guide_path, guide_stamp, dynamic_sfc_start_wt, dynamic_hplanes);
        std::vector<traj_opt::SwarmPrediction> swarm_predictions;
        buildSwarmPredictions(swarm_predictions);
        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizExpSfc(sfc);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        time_consuming_[EPX_TRAJ_FRONTEND] = t_exp_frontend.stop();


        pos_fina_state.setZero();//设置轨迹优化的终点状态
        pos_fina_state.col(0) = guide_path.back();// 位置为 guide_path 终点
        //如果启用了末端速度约束，并且离目标较远，则设置一个“朝向目标方向”的速度终态
        if (cfg_.goal_vel_en && (gi_.goal_p - robot_state_.p).norm() > cfg_.planning_horizon / 2) {
            pos_fina_state.col(1) = (gi_.goal_p - robot_state_.p).normalized() * cfg_.exp_traj_cfg.max_vel / 2;
        }
        //如果终点离目标很近，设置为静止状态，且直接对齐目标点
        if ((pos_fina_state.col(0) - gi_.goal_p).norm() < cfg_.resolution * 2) {
            pos_fina_state.col(1).setZero();
            pos_fina_state.col(0) = gi_.goal_p;
        }

        // optimize and update exp traj
        //在 SFC 中进行轨迹优化
        bool temp_ret;
        Trajectory out_traj;
        TimeConsuming t_exp_opt("t_exp_opt", false);
        auto original_sfc = sfc;
        temp_ret = exp_traj_opt_->optimize(pos_init_state,
                                           pos_fina_state,
                                           guide_path,
                                           guide_stamp,
                                           sfc,
                                           dynamic_hplanes,
                                           swarm_predictions,
                                           dynamic_sfc_start_wt,
                                           out_traj);
        time_consuming_[EXP_TRAJ_OPT] = t_exp_opt.stop();
        {
            VecDf init_ts;
            vec_Vec3f init_ps;
            exp_traj_opt_->getInitValue(init_ts, init_ps);
            latest_replan.setExpCondition(init_ts, init_ps, pos_init_state, pos_fina_state, sfc);
        }
        if (!temp_ret) {
            ros_ptr_->warn(" -- [SUPER] OptimizationExpTrajInPolytopes for new path failed");
            return FAILED;
        }
        //判断重规划是否超时
        double replan_total_t = (ros_ptr_->getSimTime() - replan_process_start_WT);
        if (replan_total_t > cfg_.max_replan_time) {
            ros_ptr_->warn(" -- [SUPER] Replan over time({}) > max_replan_time({})!!!! Return FAILED",
                           replan_total_t, cfg_.max_replan_time);
            return FAILED;
        }
        //可视化轨迹
        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizExpTraj(out_traj);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

        double new_traj_WT = last_exp_traj_info.empty() ? ros_ptr_->getSimTime() : replan_process_start_WT;
        //对新的轨迹设置时间偏移（世界时间 -> 轨迹时间）
        replan_process_start_TT = replan_process_start_WT - guide_pos_traj.start_WT;
        Trajectory temp_exp_traj;
        if (!last_exp_traj_info_.empty() && ////如果上一次轨迹还存在，则从 last_traj 里截取一个“到当前时刻的部分”，用来和新轨迹拼接
            !guide_pos_traj.getPartialTrajectoryByTime(replan_process_start_TT, replan_state_TT,
                                                       temp_exp_traj)) {
            ros_ptr_->error(" -- [SUPER] in [generateExpTraj]: getPartialTrajectoryByTime failed, force return");
            return FAILED;
        }
        out_exp_traj_info.setSFC(sfc);
        temp_exp_traj = temp_exp_traj + out_traj;
        temp_exp_traj.start_WT = new_traj_WT; //last_exp_traj_info.replan_start_WT ;
        
        if (!last_exp_traj_info.empty()) {
            StatePVAJ yaw_replan_state;
            if (!guide_yaw_traj.getState(replan_state_TT, yaw_replan_state)) {
                ros_ptr_->warn(" -- [SUPER] Invalid traj or eval t");
                return FAILED;
            }
            init_yaw = yaw_replan_state.row(0);//重建 yaw 轨迹起始状态
        }
        // /


        bool free_end{true};
        if (cfg_.goal_yaw_en && !isnan(gi_.goal_yaw) && connected_goal) {//判断终点是否固定 yaw 角
            free_end = false;
            fina_yaw[0] = gi_.goal_yaw;
        }
        Trajectory new_traj, old_traj;
        //优化 yaw 轨迹
        if (!yaw_traj_opt_->optimize(init_yaw, fina_yaw, out_traj, new_traj, 3, false, free_end)) {
            ros_ptr_->error(" -- [SUPER] in [generateExpTraj]: YawTrajOpt failed, force return");
            return FAILED;
        }
        //获取旧的 yaw 轨迹用于拼接
        if (!last_exp_traj_info.empty()) {
            if (!guide_yaw_traj.getPartialTrajectoryByTime(replan_process_start_TT, replan_state_TT,
                                                           old_traj)) {
                ros_ptr_->error(" -- [SUPER] in [generateExpTraj]: getPartialTrajectoryByTime failed, force return");
                return FAILED;
            }
        }

        const auto temp_yaw_traj = old_traj + new_traj; //拼接 yaw 轨迹
        // check if part of the exp on last backup
        double on_backup_end_TT{-1}, on_backup_start_TT{-1};
        //检查轨迹是否包含 BackupTraj 部分
        if (!last_exp_traj_info.empty() && replan_state_TT > cmd_traj_info_.getBackupTrajStartTT()) {
            on_backup_start_TT = cmd_traj_info_.getBackupTrajStartTT() - replan_process_start_TT;
            on_backup_end_TT = replan_state_TT - replan_process_start_TT;
        }
        //写入最终结果（轨迹 + 时间 + 信息）
        out_exp_traj_info.setTrajectory(new_traj_WT, temp_exp_traj, temp_yaw_traj, on_backup_start_TT,
                                        on_backup_end_TT);

        latest_replan.setExpYawTraj(temp_yaw_traj);
        latest_replan.setExpTraj(temp_exp_traj);

        return SUCCESS;
    }

    RET_CODE SuperPlanner::generateBackupTrajectory(ExpTraj &ref_exp_traj, BackupTraj &back_traj_info) {
        drone_state_mutex_.lock();
        back_traj_info.setRobotPos(robot_state_.p);
        drone_state_mutex_.unlock();
        TimeConsuming t_back_frontend("t_back_frontend", false);
        double total_dur = ref_exp_traj.getTotalDuration();
        double start_t = ros_ptr_->getSimTime() - ref_exp_traj.getStartWallTime();


        if (start_t > total_dur - 0.01) {
            if (cfg_.print_log) {
                ros_ptr_->info(" -- [SUPER] in [generateBackupTrajectory]: start_t > total_dur, return NO_NEED");
            }
            return NO_NEED;
        }

        Vec3f temp_point;
        double out_t;
        bool all_traj_visible{true};
        // 同时记录每一个点的刹车时间和刹车距离
        vector<double> min_stop_dis;
        vector<TimePosPair> eval_ps;
        Vec3f temp_vel;

        // 记录当前时刻到最远时刻的所有可视部分
        Vec3f last_pos = ref_exp_traj.getPos(start_t);
        Vec3f last_eval_pos = last_pos;
        bool has_last_eval_pos{true};
        const double risk_gate_end_t =
                cfg_.backup_traj_risk_gate_en && cfg_.backup_traj_trigger_lookahead_time > 0.0
                ? std::min(total_dur, start_t + cfg_.backup_traj_trigger_lookahead_time)
                : total_dur;
        for (out_t = start_t; out_t < total_dur; out_t += cfg_.sample_traj_dt) {
            if (cfg_.backup_traj_risk_gate_en && out_t > risk_gate_end_t) {
                break;
            }
            temp_point = ref_exp_traj.getPos(out_t);
            if ((last_pos - temp_point).norm() < cfg_.resolution * 0.8) {
                continue;
            }
            const bool exp_segment_infeasible =
                    has_last_eval_pos && !map_ptr_->isLineFree(last_eval_pos, temp_point, false, false);
            last_pos = temp_point;
            temp_vel = ref_exp_traj.getVel(out_t);
            // Compute initial
            double v_norm = temp_vel.norm();
            min_stop_dis.push_back(v_norm * v_norm / 2.0 / cfg_.exp_traj_cfg.max_acc);
            eval_ps.push_back(std::pair<double, Vec3f>(out_t, temp_point));

            if (cfg_.backup_traj_risk_gate_en) {
                Vec3f nearest_occ;
                const bool exp_point_infeasible = map_ptr_->isOccupied(temp_point);
                const bool exp_point_near_obstacle =
                        cfg_.backup_traj_trigger_clearance > 0.0 &&
                        map_ptr_->getNearestCellIs(GridType::OCCUPIED, temp_point, nearest_occ,
                                                   cfg_.backup_traj_trigger_clearance);

                if (exp_point_infeasible || exp_segment_infeasible || exp_point_near_obstacle) {
                    all_traj_visible = false;
                    if (cfg_.print_log) {
                        ros_ptr_->info(" -- [SUPER] in [generateBackupTrajectory]: backup risk gate triggered.");
                    }
                    break;
                }

                has_last_eval_pos = true;
                last_eval_pos = temp_point;
                continue;
            }

            const double min_dis =
                    cfg_.sensing_horizon > 0 ? std::min(cfg_.sensing_horizon, cfg_.safe_corridor_line_max_length)
                                             : cfg_.safe_corridor_line_max_length;
            if (!map_ptr_->isLineFree(back_traj_info.getRobotPos(),
                                      temp_point,
                                      min_dis,
                                      cfg_.seed_line_neighbour)) {
                all_traj_visible = false;
                break;
            }
            has_last_eval_pos = true;
            last_eval_pos = temp_point;
        }

        if (all_traj_visible) {
            back_traj_info.setEmpty();
            {
                double dur = ref_exp_traj.getTotalDuration();
                Vec3f seed_pt = ref_exp_traj.getPos(dur);
                Line line{back_traj_info.getRobotPos(), seed_pt};
                Polytope temp_poly;
                if (cg_ptr_->GeneratePolytopeFromLine(line, temp_poly)) {
                    back_traj_info.setSFC(temp_poly);
                    {
                        TimeConsuming t_viz("tviz", false);
                        ros_ptr_->vizBackupSfc(temp_poly);
                        time_consuming_[VISUALIZATION] += t_viz.stop();
                    }
                }
            }
            return FINISH;
        }
        if (eval_ps.empty()) {
            ros_ptr_->warn(" -- [SUPER] in [generateBackupTrajectory]: no sampled backup seed, force return");
            back_traj_info.setEmpty();
            return OPT_FAILED;
        }
        Vec3f invisible_p = eval_ps.back().second;
        while (out_t > start_t) {
            out_t -= cfg_.sample_traj_dt;
            Vec3f out_p = ref_exp_traj.getPos(out_t);
            if ((out_p - invisible_p).norm() > cfg_.robot_r) {
                break;
            }
        }

        double seed_point_t = std::max(start_t, out_t);

        // TODO check this logic, comment on Dec. 13
        // if
        // 1) last exp traj has a backup traj
        // 2) last backup WT is larger than this term
        // 3) last exp is collision free
        // if (ref_exp_traj.back_traj_start_TT > 0 &&
        // seed_point_t < ref_exp_traj.back_traj_start_TT) {
        // return NO_NEED;
        // }


        Vec3f seed_point = ref_exp_traj.getPos(seed_point_t);

        Vec3f shifted_robot_p = shifted_sfc_start_pt_.norm()> 999?robot_state_.p:shifted_sfc_start_pt_;
        if (!map_ptr_->getNearestCellNot(GridType::OCCUPIED, shifted_robot_p, shifted_robot_p, 3.0)) {
            ros_ptr_->error(
                    " -- [SUPER] in [PlanFromRest] Local start point is deeply occupied, which should not happened.");
            latest_replan.setRetCode(SUPER_RET_CODE::SUPER_NO_START_POINT);
            return FAILED;
        }

        Line line{shifted_robot_p, seed_point};
        Polytope temp_poly;
        if (!cg_ptr_->GeneratePolytopeFromLine(line, temp_poly)) {
            ros_ptr_->warn(" -- [SUPER] GeneratePolytopeFromLine failed, force return");
            return FAILED;
        }
        Eigen::Vector3d inner;
        Eigen::Matrix3Xd vPoly;
        if (!geometry_utils::findInterior(temp_poly.GetPlanes(), inner)) {
            ros_ptr_->warn(" -- [SUPER] Cannot generate feasible backup sfc, force return");
            vec_Vec3f seed{back_traj_info.getRobotPos(), seed_point};
            return FAILED;
        }

        if (cfg_.use_fov_cut) {
            if (!fov_checker_->cutPolyByFov(robot_state_.p, robot_state_.q, seed_point,
                                            temp_poly)) {
                ros_ptr_->warn(" -- [SUPER] cutPolyByFov failed, force return");
                return FAILED;
            }
        }
        // cut by sensing horizon
        if (cfg_.sensing_horizon > 0 &&
            !fov_checker_->cutPolyBySensingHorizon(robot_state_.p, seed_point, cfg_.sensing_horizon,
                                                   temp_poly)) {
            ros_ptr_->warn(" -- [SUPER] cutPolyBySensingHorizon failed, force return");
            vec_Vec3f seed{back_traj_info.getRobotPos(), seed_point};
            return FAILED;
        }

        back_traj_info.setSFC(temp_poly);

        {
            TimeConsuming t_viz("tviz", false);
            ros_ptr_->vizBackupSfc(temp_poly);
            time_consuming_[VISUALIZATION] += t_viz.stop();
        }

//        Vec3f out_p = temp_point;
//        double t_R = 0.0;
        double eval_t = eval_ps.back().first + cfg_.sample_traj_dt;
        last_pos = eval_ps.back().second;
        while (temp_poly.PointIsInside(eval_ps.back().second) && eval_t < total_dur) {
            Vec3f cur_pos = ref_exp_traj.getPos(eval_t);

            if ((cur_pos - last_pos).norm() < cfg_.resolution * 0.8) {
                eval_t += cfg_.sample_traj_dt;
                continue;
            }
            temp_vel = ref_exp_traj.getVel(eval_t);
            double v_norm = temp_vel.norm();
            min_stop_dis.push_back(v_norm * v_norm / 2.0 / cfg_.exp_traj_cfg.max_acc);
            eval_ps.emplace_back(eval_t, cur_pos);
            last_pos = cur_pos;
            eval_t += cfg_.sample_traj_dt;
        }
        if (eval_ps.size() <= 1) {
            ros_ptr_->warn(" -- [SUPER] in [generateBackupTrajectory]: backup seed interval is too short, force return");
            back_traj_info.setEmpty();
            return OPT_FAILED;
        }
        eval_ps.pop_back();
        if (eval_ps.empty()) {
            ros_ptr_->warn(" -- [SUPER] in [generateBackupTrajectory]: backup seed list is empty, force return");
            back_traj_info.setEmpty();
            return OPT_FAILED;
        }
        seed_point = eval_ps.back().second;
        seed_point_t = eval_ps.back().first;

        //        bool use_new{true};
        //        if (use_new) {
        double t0 = ros_ptr_->getSimTime() -
                    ref_exp_traj.getStartWallTime() + 0.01;
        double te = seed_point_t;
        if (te <= t0 + 1e-6) {
            ros_ptr_->warn(" -- [SUPER] in [generateBackupTrajectory]: invalid backup time window, force return");
            back_traj_info.setEmpty();
            return OPT_FAILED;
        }
        //            cout << "t0: " << t0 << endl;
        //            cout << "te: " << te << endl;
        //            cout << "exp_traj_dur: " << ref_exp_traj.optimized_exp_traj.getTotalDuration() << endl;
        double vel_e_n = ref_exp_traj.getVel(te).norm();
        double heu_ts = std::max((t0 + te) / 2, te - vel_e_n / cfg_.back_traj_cfg.max_acc);
        double heu_dur = te - heu_ts;
        Vec3f heu_p = seed_point;
        time_consuming_[BACK_TRAJ_FRONTEND] = t_back_frontend.stop();
        TimeConsuming t_back_opt("t_back_opt", false);
        double opt_ts = heu_ts;
        Trajectory temp_pos_traj;
        auto sfc0 = back_traj_info.getSFC();
        std::vector<traj_opt::SwarmPrediction> swarm_predictions;
        buildSwarmPredictions(swarm_predictions);
        bool temp_ret = back_traj_opt_->optimize(ref_exp_traj.posTraj(),
                                                 t0,
                                                 te,
                                                 heu_ts,
                                                 heu_p,
                                                 heu_dur,
                                                 back_traj_info.getSFC(),
                                                 temp_pos_traj,
                                                 opt_ts,
                                                 swarm_predictions);
        time_consuming_[BACK_TRAJ_OPT] = t_back_opt.stop();

        {
            double init_ts;
            VecDf init_times;
            vec_Vec3f init_ps;
            back_traj_opt_->getInitValue(init_ts, init_times, init_ps);
            latest_replan.setBackupCondition(init_ts, init_times, init_ps,
                                             t0, te,
                                             back_traj_info.getSFC());
        }

        if (!temp_ret) {
            ros_ptr_->warn(" -- [SUPER] OptimizationBakTrajInPolytopes failed, force return");
            back_traj_info.setEmpty();
            return OPT_FAILED;
        } else {
            Vec4f yaw_init_vec = ref_exp_traj.getYawState(opt_ts).row(0);
            Vec4f yaw_goal{0, 0, 0, 0};
            bool free_end{true};
            if (cfg_.goal_yaw_en) {
                if (!isnan(gi_.goal_yaw)) {
                    free_end = false;
                    yaw_goal[0] = gi_.goal_yaw;
                }
            }
            Trajectory temp_yaw_traj;
            if (!yaw_traj_opt_->optimize(yaw_init_vec, yaw_goal, temp_pos_traj,
                                         temp_yaw_traj, 3, false, free_end)) {
                ros_ptr_->error(" -- [SUPER] in [generateBackupTrajectory] YawTrajOpt FAILD.");
                return OPT_FAILED;
            }


            if (opt_ts < t0) {
                ros_ptr_->error(" -- [SUPER] opt_ts {} < t0 {}", opt_ts, t0);
                return OPT_FAILED;
            }
            double new_ts_WT = ref_exp_traj.getStartWallTime() + opt_ts;
            const auto &committed_ts_WT = cmd_traj_info_.getBackupTrajStartTT();
            if (committed_ts_WT < cmd_traj_info_.getTotalDuration() && new_ts_WT < committed_ts_WT) {
                ros_ptr_->error(" -- [SUPER] new_ts_WT {} < committed_ts_WT {}", new_ts_WT, committed_ts_WT);
                return OPT_FAILED;
            }


            {
                TimeConsuming t_viz("tviz", false);
                ros_ptr_->vizBackupTraj(temp_pos_traj);
                time_consuming_[VISUALIZATION] += t_viz.stop();
            }

            back_traj_info.setTrajectory(new_ts_WT, opt_ts, temp_pos_traj, temp_yaw_traj);
            latest_replan.setBackupTraj(temp_pos_traj);
            latest_replan.setBackupYawTraj(temp_yaw_traj);
            return SUCCESS;
        }
        ros_ptr_->warn(" -- [SUPER] Cannot find backup traj start point.");
        return FAILED;
    }

    int SuperPlanner::getNearestFurtherGoalPoint(const vec_E<Vec3f> &goals, const Vec3f &start_pt) {
        if (goals.size() == 1) {
            return 0;
        }
        Vec3f a = start_pt, b;
        int min_id = 0;
        double min_dis = 1e10;
        for (long unsigned int i = 0; i < goals.size() - 1; i++) {
            b = goals[i];
            double dis = geometry_utils::pointLineSegmentDistance(start_pt, a, b);
            if (dis < min_dis) {
                min_dis = dis;
                min_id = i;
            }
            a = b;
        }
        return min_id;
    }

    bool
    SuperPlanner::PathSearch(const Vec3f &start_pt, const Vec3f &goal,
                             const double &searching_horizon,
                             vec_Vec3f &path) {
        using namespace path_search;
        if (searching_horizon <= 0.0) {
            ros_ptr_->error(" -- [SUPER] Goal waypoints empty or searching horizon negative, force return.");
            return false;
        }

        // 1) check and shift pts
        // 		For start point, must be collision free
        rog_map::GridType start_type;
        start_type = map_ptr_->getGridType(start_pt);

        /// If the start_pt is obstacle in prob map, just shift it to the nearest free point.
        if (start_type == rog_map::GridType::OCCUPIED ||
            start_type == rog_map::GridType::OUT_OF_MAP) {
            ros_ptr_->warn(
                    " -- [SUPER] The start point in obstacle, this should not happen since the start point should be shift before pathsearch.");
            return false;
        }
        vec_E<Vec3f> start_point_escape_path;

        int flag_es = ON_PROB_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE);
        vec_Vec3f out_path;
        RET_CODE ret_es = astar_ptr_->escapePathSearch(start_pt, flag_es, out_path);
        if (ret_es != NO_NEED) {
            if (ret_es != REACH_HORIZON && ret_es != REACH_GOAL) {
                ros_ptr_->error(
                        " -- [SUPER] Escape path search failed with [{}], force return.",
                        RET_CODE_STR[ret_es].c_str());
                return false;
            } else {
                start_point_escape_path = out_path;
            }
        }

        Vec3f shifted_start_pt = start_pt;

        if (!start_point_escape_path.empty()) {
            shifted_start_pt = start_point_escape_path.back();
        }

        Vec3f temp_goal_point, temp_start_point;
        temp_start_point = shifted_start_pt;
        double temp_plannning_horizon = searching_horizon;
        //            int start_id = getNearestFurtherGoalPoint(goal_waypoints, start_pt);

        int flag = ON_INF_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE) | DONT_USE_INF_NEIGHBOR;

        bool dstar_path_available = cfg_.use_dstar_lite_frontend;
        RET_CODE ret_code = cfg_.use_dstar_lite_frontend
                            ? dstar_lite_ptr_->searchOrRepair(temp_start_point, goal, flag, temp_plannning_horizon,
                                                              path)
                            : astar_ptr_->pointToPointPathSearch(temp_start_point, goal, flag,
                                                                 temp_plannning_horizon, path);

        if(ret_code == INIT_ERROR){
            gi_.goal_valid = false;
            return false;
        }
        //add may23, if failed on inf map, use prob map try again

        if (ret_code == NO_PATH) {
            flag = ON_PROB_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE) |
                   USE_INF_NEIGHBOR;
            fmt::print(fg(fmt::color::indian_red) | fmt::emphasis::bold,
                       " -- [Astar] Path search failed on inf map, try again on prob map.\n");
            ret_code = cfg_.use_dstar_lite_frontend
                       ? dstar_lite_ptr_->searchOrRepair(temp_start_point, goal, flag, temp_plannning_horizon,
                                                         path)
                       : astar_ptr_->pointToPointPathSearch(temp_start_point, goal, flag,
                                                            temp_plannning_horizon, path);
            if (ret_code == SUCCESS || ret_code == REACH_HORIZON || ret_code == REACH_GOAL) {
                fmt::print(fg(fmt::color::lime_green) | fmt::emphasis::bold,
                           " -- [Astar] Path search on prob map success.\n");
            } else {
                fmt::print(fg(fmt::color::indian_red) | fmt::emphasis::bold,
                           " -- [Astar] Path search failed on prob map still failed.\n");
            }
        }
        if (cfg_.use_dstar_lite_frontend && ret_code != REACH_HORIZON && ret_code != REACH_GOAL) {
            fmt::print(fg(fmt::color::indian_red) | fmt::emphasis::bold,
                       " -- [D* Lite] Dynamic frontend failed, fallback to A*.\n");
            dstar_path_available = false;
            flag = ON_INF_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE) |
                   DONT_USE_INF_NEIGHBOR;
            ret_code = astar_ptr_->pointToPointPathSearch(temp_start_point, goal, flag, temp_plannning_horizon,
                                                          path);
            if (ret_code == NO_PATH) {
                flag = ON_PROB_MAP | (cfg_.frontend_in_known_free ? UNKNOWN_AS_OCCUPIED : UNKNOWN_AS_FREE) |
                       USE_INF_NEIGHBOR;
                ret_code = astar_ptr_->pointToPointPathSearch(temp_start_point, goal, flag, temp_plannning_horizon,
                                                              path);
            }
        }
        if (ret_code != REACH_HORIZON && ret_code != REACH_GOAL) {
            ros_ptr_->error(
                    " -- [SUPER] Path search failed with [{}], force return.\n", RET_CODE_STR[ret_code].c_str());
            return false;
        }
        if (!start_point_escape_path.empty()) {
            path.insert(path.begin(), start_point_escape_path.begin(),
                        start_point_escape_path.end());
        }

        if (path.empty()) {
            ros_ptr_->warn(
                    " -- [SUPER] Path search failed with empty segments, force return.");
            return false;
        }
        path.insert(path.begin(), start_pt);
        if (ret_code == REACH_GOAL) {
            path.push_back(goal);
        }
        const size_t raw_path_size = path.size();
        shortcutFrontendPath(path, map_ptr_, cfg_.frontend_in_known_free);
        if (cfg_.print_log && path.size() != raw_path_size) {
            ros_ptr_->info(std::string(" -- [SUPER] Frontend path smoothing") +
                           (dstar_path_available ? " (D* Lite): " : " (A*): ") +
                           std::to_string(raw_path_size) + " -> " +
                           std::to_string(path.size()) + " points.");
        }
        return true;
    }


    void SuperPlanner::getRobotState(rog_map::RobotState &out) {
        robot_state_ = map_ptr_->getRobotState();
        out = robot_state_;
    }
}
