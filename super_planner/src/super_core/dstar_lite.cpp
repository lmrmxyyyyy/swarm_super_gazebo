#include <path_search/dstar_lite.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

#include <fmt/color.h>

using namespace color_text;
using namespace super_utils;

namespace path_search {
    using namespace rog_map;

    namespace {
        constexpr double kKeyEps = 1e-9;
        constexpr int kMaxExtractIter = 100000;
        constexpr double kDStarHeuristicWeight = 1.0;
        constexpr double kExtractTieEps = 1e-6;
        constexpr double kExtractTurnWeight = 2.0e-2;
        constexpr double kExtractGoalWeight = 3.0e-3;
        constexpr double kExtractVerticalWeight = 2.0e-5;

        bool sameId(const rog_map::Vec3i &a, const rog_map::Vec3i &b) {
            return a.x() == b.x() && a.y() == b.y() && a.z() == b.z();
        }

        double directionChangeCost(const rog_map::Vec3i &last_delta,
                                   const rog_map::Vec3i &new_delta) {
            if (last_delta.squaredNorm() == 0 || new_delta.squaredNorm() == 0) {
                return 0.0;
            }
            const double dot = last_delta.cast<double>().dot(new_delta.cast<double>());
            const double denom = last_delta.cast<double>().norm() * new_delta.cast<double>().norm();
            if (denom < 1e-6) {
                return 0.0;
            }
            const double cos_angle = std::max(-1.0, std::min(1.0, dot / denom));
            return 1.0 - cos_angle;
        }

        double pointPathLength(const rog_map::vec_Vec3f &path) {
            if (path.size() <= 1) {
                return 0.0;
            }
            double length = 0.0;
            for (size_t i = 1; i < path.size(); ++i) {
                length += (path[i] - path[i - 1]).norm();
            }
            return length;
        }

        bool pathTooIndirect(const rog_map::vec_Vec3f &path,
                             const rog_map::Vec3f &start_pt,
                             const rog_map::Vec3f &goal_pt) {
            if (path.size() < 8) {
                return false;
            }
            const double straight = (goal_pt - start_pt).norm();
            if (straight < 1.0) {
                return false;
            }
            const double length = pointPathLength(path);
            return length > std::max(straight * 1.45, straight + 1.5);
        }
    }

    bool DStarQueueComparator::operator()(const DStarQueueNode &a, const DStarQueueNode &b) const {
        if (std::fabs(a.key.k1 - b.key.k1) > kKeyEps) {
            return a.key.k1 > b.key.k1;
        }
        return a.key.k2 > b.key.k2;
    }

    DStarLite::DStarLite(const std::string &cfg_path,
                         const ros_interface::RosInterface::Ptr &ros_ptr,
                         rog_map::ROGMapROS::Ptr rm) : map_ptr_(rm), ros_ptr_(ros_ptr) {
        cfg_ = PathSearchConfig(cfg_path);
        cout << rog_map::GREEN << " -- [D* Lite] Init D* Lite frontend." << rog_map::RESET << endl;

        setFineInfNeighbors(1);
    }

    std::tuple<int, int, int> DStarLite::idToKey(const rog_map::Vec3i &id) {
        return std::make_tuple(id.x(), id.y(), id.z());
    }

    bool DStarLite::keyLess(const DStarKey &a, const DStarKey &b) {
        if (std::fabs(a.k1 - b.k1) > kKeyEps) {
            return a.k1 < b.k1;
        }
        return a.k2 < b.k2 - kKeyEps;
    }

    bool DStarLite::keyEqual(const DStarKey &a, const DStarKey &b) {
        return std::fabs(a.k1 - b.k1) <= kKeyEps && std::fabs(a.k2 - b.k2) <= kKeyEps;
    }

    DStarNode *DStarLite::getNode(const rog_map::Vec3i &id) {
        const auto key = idToKey(id);
        auto iter = nodes_.find(key);
        if (iter != nodes_.end()) {
            return iter->second.get();
        }

        auto node = std::make_unique<DStarNode>();
        node->id_g = id;
        DStarNode *ptr = node.get();
        nodes_[key] = std::move(node);
        return ptr;
    }

    DStarNode *DStarLite::findNode(const rog_map::Vec3i &id) {
        const auto iter = nodes_.find(idToKey(id));
        return iter == nodes_.end() ? nullptr : iter->second.get();
    }

    void DStarLite::resetSearch() {
        nodes_.clear();
        occupancy_cache_.clear();
        open_set_ = std::priority_queue<DStarQueueNode, std::vector<DStarQueueNode>, DStarQueueComparator>();
        initialized_ = false;
        km_ = 0.0;
    }

    void DStarLite::reset() {
        resetSearch();
    }

    void DStarLite::setFineInfNeighbors(const int &neighbor_step) {
        neighbor_list_.clear();
        const int step = std::max(1, neighbor_step);
        for (int i = -step; i <= step; i++) {
            for (int j = -step; j <= step; j++) {
                for (int k = -step; k <= step; k++) {
                    if (i == 0 && j == 0 && k == 0) {
                        continue;
                    }
                    if (i * i + j * j + k * k > step * step) {
                        continue;
                    }
                    neighbor_list_.emplace_back(i, j, k);
                }
            }
        }
    }

    RET_CODE DStarLite::setup(const rog_map::Vec3f &start_pt,
                              const rog_map::Vec3f &goal_pt,
                              const int &flag,
                              const double &searching_horizon,
                              rog_map::Vec3f &local_start_pt,
                              rog_map::Vec3f &local_goal_pt,
                              bool &goal_out_local_map) {
        use_inf_map_ = flag & ON_INF_MAP;
        use_prob_map_ = flag & ON_PROB_MAP;
        unknown_as_occ_ = flag & UNKNOWN_AS_OCCUPIED;
        unknown_as_free_ = flag & UNKNOWN_AS_FREE;
        use_inf_neighbor_ = flag & USE_INF_NEIGHBOR;
        if (flag & DONT_USE_INF_NEIGHBOR) {
            use_inf_neighbor_ = false;
        }

        if ((use_inf_map_ && use_prob_map_) || (!use_inf_map_ && !use_prob_map_)) {
            cout << YELLOW << " -- [D* Lite] " << RET_CODE_STR[INIT_ERROR]
                 << ": cannot use both inf map and prob map." << RESET << endl;
            return INIT_ERROR;
        }
        if (unknown_as_occ_ && unknown_as_free_) {
            cout << YELLOW << " -- [D* Lite] " << RET_CODE_STR[INIT_ERROR]
                 << ": cannot use both unknown_as_occupied and unknown_as_free." << RESET << endl;
            return INIT_ERROR;
        }

        resolution_ = use_inf_map_ ? map_ptr_->getInfResolution() : map_ptr_->getResolution();
        local_map_center_d_ = searching_horizon > 0.0 ? start_pt : (start_pt + goal_pt) / 2;
        posToGlobalIndex(local_map_center_d_, local_map_center_id_g_);
        local_map_min_d_ = local_map_center_d_ - resolution_ * cfg_.map_size_i.cast<double>();
        local_map_max_d_ = local_map_center_d_ + resolution_ * cfg_.map_size_i.cast<double>();
        if (cfg_.visual_process || cfg_.debug_visualization_en) {
            ros_ptr_->vizAstarBoundingBox(local_map_min_d_, local_map_max_d_);
        }

        local_start_pt = start_pt;
        local_goal_pt = goal_pt;
        goal_out_local_map = false;

        if (!insideLocalMap(start_pt)) {
            ros_ptr_->warn(" -- [D* Lite] Start point [{}] is out of local D* map.", start_pt.transpose());
            return INIT_ERROR;
        }

        if (!insideLocalMap(goal_pt)) {
            goal_out_local_map = true;
            rog_map::Vec3f hit_pt;
            if (rog_map::lineIntersectBox(goal_pt, start_pt, local_map_min_d_, local_map_max_d_, hit_pt)) {
                rog_map::Vec3f dir = (hit_pt - goal_pt).normalized();
                const double dis = (hit_pt - goal_pt).norm();
                local_goal_pt = goal_pt + dir * (dis + resolution_ * 2.0);
                if (!map_ptr_->getNearestInfCellNot(OCCUPIED, local_goal_pt, local_goal_pt, 2.0)) {
                    ros_ptr_->error(" -- [D* Lite] Local horizon goal [{}] deeply occupied.",
                                    local_goal_pt.transpose());
                    return INIT_ERROR;
                }
            }
        }

        posToGlobalIndex(local_start_pt, start_idx_);
        posToGlobalIndex(local_goal_pt, goal_idx_);
        if (!insideLocalMap(start_idx_) || !insideLocalMap(goal_idx_)) {
            ros_ptr_->error(" -- [D* Lite] Start or local goal is outside the D* local map.");
            return INIT_ERROR;
        }

        return SUCCESS;
    }

    void DStarLite::posToGlobalIndex(const rog_map::Vec3f &pos, rog_map::Vec3i &id_g) const {
        if (use_inf_map_) {
            map_ptr_->infMapPosToGlobalIndex(pos, id_g);
        } else if (use_prob_map_) {
            map_ptr_->probMapPosToGlobalIndex(pos, id_g);
        } else {
            throw std::runtime_error(" -- [D* Lite] Map type not defined.");
        }
    }

    void DStarLite::globalIndexToPos(const rog_map::Vec3i &id_g, rog_map::Vec3f &pos) const {
        if (use_inf_map_) {
            map_ptr_->infMapGlobalIndexToPos(id_g, pos);
        } else if (use_prob_map_) {
            map_ptr_->probMapGlobalIndexToPos(id_g, pos);
        } else {
            throw std::runtime_error(" -- [D* Lite] Map type not defined.");
        }
    }

    bool DStarLite::insideLocalMap(const rog_map::Vec3i &id_g) const {
        const rog_map::Vec3i delta = id_g - local_map_center_id_g_;
        return std::fabs(delta.x()) <= cfg_.map_size_i.x() &&
               std::fabs(delta.y()) <= cfg_.map_size_i.y() &&
               std::fabs(delta.z()) <= cfg_.map_size_i.z();
    }

    bool DStarLite::insideLocalMap(const rog_map::Vec3f &pos) const {
        rog_map::Vec3i id_g;
        posToGlobalIndex(pos, id_g);
        return insideLocalMap(id_g);
    }

    double DStarLite::heuristic(const rog_map::Vec3i &a, const rog_map::Vec3i &b) const {
        const double dx = std::abs(a.x() - b.x());
        const double dy = std::abs(a.y() - b.y());
        const double dz = std::abs(a.z() - b.z());

        switch (cfg_.heu_type) {
            case 0: {
                double x = dx;
                double y = dy;
                double z = dz;
                const double diag3 = std::min(std::min(x, y), z);
                x -= diag3;
                y -= diag3;
                z -= diag3;
                return std::sqrt(3.0) * diag3 + std::sqrt(2.0) * std::min(std::min(x, y), z) +
                       std::max(std::max(x, y), z) - std::min(std::min(x, y), z);
            }
            case 1:
                return dx + dy + dz;
            case 2:
            default:
                return (a - b).cast<double>().norm();
        }
    }

    rog_map::GridType DStarLite::queryGridType(const rog_map::Vec3i &id) {
        rog_map::Vec3f pos;
        globalIndexToPos(id, pos);
        if (use_inf_map_) {
            return map_ptr_->getInfGridType(pos);
        }
        if (!use_inf_neighbor_) {
            return map_ptr_->getGridType(pos);
        }

        rog_map::GridType neighbor_type = neighborHaveOne(OCCUPIED, id) ? OCCUPIED : UNDEFINED;
        if (unknown_as_occ_ && neighbor_type != OCCUPIED) {
            neighbor_type = neighborHaveOne(KNOWN_FREE, id) ? KNOWN_FREE : UNKNOWN;
        }
        return neighbor_type;
    }

    bool DStarLite::neighborHaveOne(const rog_map::GridType &type, const rog_map::Vec3i &src_id) {
        for (const auto &delta: neighbor_list_) {
            const rog_map::Vec3i id = src_id + delta;
            rog_map::Vec3f pos;
            globalIndexToPos(id, pos);
            if (!map_ptr_->insideLocalMap(pos)) {
                continue;
            }
            if (map_ptr_->getGridType(pos) == type) {
                return true;
            }
        }
        return false;
    }

    bool DStarLite::isBlocked(const rog_map::Vec3i &id) {
        if (!insideLocalMap(id)) {
            return true;
        }
        const rog_map::GridType type = queryGridType(id);
        if (type == OCCUPIED || type == OUT_OF_MAP) {
            return true;
        }
        return unknown_as_occ_ && type == UNKNOWN;
    }

    double DStarLite::edgeCost(const rog_map::Vec3i &from, const rog_map::Vec3i &to) {
        if (isBlocked(from) || isBlocked(to)) {
            return inf;
        }
        const rog_map::Vec3i delta = to - from;
        if (!cfg_.allow_diag && std::abs(delta.x()) + std::abs(delta.y()) + std::abs(delta.z()) > 1) {
            return inf;
        }
        return delta.cast<double>().norm();
    }

    DStarKey DStarLite::calculateKey(DStarNode *node) const {
        const double best = std::min(node->g, node->rhs);
        return {best + kDStarHeuristicWeight * heuristic(start_idx_, node->id_g) + km_, best};
    }

    void DStarLite::pushIfInconsistent(DStarNode *node) {
        if (std::fabs(node->g - node->rhs) > kKeyEps) {
            open_set_.push({node, calculateKey(node)});
        }
    }

    bool DStarLite::queueTop(DStarQueueNode &out) {
        while (!open_set_.empty()) {
            out = open_set_.top();
            if (!out.node) {
                open_set_.pop();
                continue;
            }
            if (std::fabs(out.node->g - out.node->rhs) <= kKeyEps) {
                open_set_.pop();
                continue;
            }
            const DStarKey current_key = calculateKey(out.node);
            if (!keyEqual(out.key, current_key)) {
                if (keyLess(out.key, current_key)) {
                    open_set_.pop();
                    open_set_.push({out.node, current_key});
                    continue;
                }
                open_set_.pop();
                continue;
            }
            return true;
        }
        return false;
    }

    void DStarLite::initialize(const rog_map::Vec3i &start_idx, const rog_map::Vec3i &goal_idx) {
        resetSearch();
        start_idx_ = start_idx;
        last_start_idx_ = start_idx;
        goal_idx_ = goal_idx;

        DStarNode *goal = getNode(goal_idx_);
        goal->rhs = 0.0;
        open_set_.push({goal, calculateKey(goal)});
        initialized_ = true;
    }

    void DStarLite::updateVertex(DStarNode *node) {
        if (!node) {
            return;
        }
        if (!sameId(node->id_g, goal_idx_)) {
            double min_rhs = inf;
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dz = -1; dz <= 1; dz++) {
                        if (dx == 0 && dy == 0 && dz == 0) {
                            continue;
                        }
                        if (!cfg_.allow_diag && std::abs(dx) + std::abs(dy) + std::abs(dz) > 1) {
                            continue;
                        }
                        const rog_map::Vec3i succ_id = node->id_g + rog_map::Vec3i(dx, dy, dz);
                        if (!insideLocalMap(succ_id)) {
                            continue;
                        }
                        DStarNode *succ = getNode(succ_id);
                        min_rhs = std::min(min_rhs, edgeCost(node->id_g, succ_id) + succ->g);
                    }
                }
            }
            node->rhs = min_rhs;
        }

        pushIfInconsistent(node);
    }

    void DStarLite::computeShortestPath(const double &start_time, const double &time_out) {
        DStarNode *start = getNode(start_idx_);
        DStarQueueNode top;
        int iter = 0;
        while (queueTop(top) &&
               (keyLess(top.key, calculateKey(start)) || std::fabs(start->rhs - start->g) > kKeyEps)) {
            if (++iter > kMaxExtractIter) {
                return;
            }
            if ((ros_ptr_->getSimTime() - start_time) > time_out) {
                return;
            }

            open_set_.pop();
            DStarNode *u = top.node;
            const DStarKey new_key = calculateKey(u);
            if (keyLess(top.key, new_key)) {
                open_set_.push({u, new_key});
                continue;
            }

            if (u->g > u->rhs) {
                u->g = u->rhs;
                for (int dx = -1; dx <= 1; dx++) {
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dz = -1; dz <= 1; dz++) {
                            if (dx == 0 && dy == 0 && dz == 0) {
                                continue;
                            }
                            if (!cfg_.allow_diag && std::abs(dx) + std::abs(dy) + std::abs(dz) > 1) {
                                continue;
                            }
                            const rog_map::Vec3i pred_id = u->id_g + rog_map::Vec3i(dx, dy, dz);
                            if (insideLocalMap(pred_id)) {
                                updateVertex(getNode(pred_id));
                            }
                        }
                    }
                }
            } else {
                u->g = inf;
                updateVertex(u);
                for (int dx = -1; dx <= 1; dx++) {
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dz = -1; dz <= 1; dz++) {
                            if (dx == 0 && dy == 0 && dz == 0) {
                                continue;
                            }
                            if (!cfg_.allow_diag && std::abs(dx) + std::abs(dy) + std::abs(dz) > 1) {
                                continue;
                            }
                            const rog_map::Vec3i pred_id = u->id_g + rog_map::Vec3i(dx, dy, dz);
                            if (insideLocalMap(pred_id)) {
                                updateVertex(getNode(pred_id));
                            }
                        }
                    }
                }
            }
        }
    }

    void DStarLite::detectChangedCells(std::vector<rog_map::Vec3i> &changed_cells) {
        changed_cells.clear();
        const rog_map::Vec3i min_id = local_map_center_id_g_ - cfg_.map_size_i;
        const rog_map::Vec3i max_id = local_map_center_id_g_ + cfg_.map_size_i;

        for (int x = min_id.x(); x <= max_id.x(); ++x) {
            for (int y = min_id.y(); y <= max_id.y(); ++y) {
                for (int z = min_id.z(); z <= max_id.z(); ++z) {
                    const rog_map::Vec3i id(x, y, z);
                    const bool occupied = isBlocked(id);
                    const auto key = idToKey(id);
                    const auto iter = occupancy_cache_.find(key);
                    if (iter == occupancy_cache_.end()) {
                        occupancy_cache_[key] = occupied;
                        continue;
                    }
                    if (iter->second != occupied) {
                        iter->second = occupied;
                        changed_cells.push_back(id);
                    }
                }
            }
        }
    }

    RET_CODE DStarLite::extractPath(const rog_map::Vec3f &original_start_pt,
                                    const bool &goal_out_local_map,
                                    const double &searching_horizon,
                                    rog_map::vec_Vec3f &out_path) {
        out_path.clear();
        DStarNode *start = getNode(start_idx_);
        if (!std::isfinite(start->g)) {
            return NO_PATH;
        }

        rog_map::Vec3i current = start_idx_;
        double traveled_grid = 0.0;
        const double max_grid = searching_horizon > 0.0 ? searching_horizon / std::max(1e-3, resolution_) :
                                static_cast<double>(cfg_.map_voxel_num.maxCoeff());

        rog_map::Vec3f current_pos;
        globalIndexToPos(current, current_pos);
        out_path.push_back(current_pos);

        std::set<std::tuple<int, int, int>> visited;
        visited.insert(idToKey(current));
        rog_map::Vec3i last_delta(0, 0, 0);

        for (int iter = 0; iter < kMaxExtractIter && !sameId(current, goal_idx_); ++iter) {
            double best_nominal_score = inf;
            double best_tie_score = inf;
            rog_map::Vec3i best_id = current;

            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dz = -1; dz <= 1; dz++) {
                        if (dx == 0 && dy == 0 && dz == 0) {
                            continue;
                        }
                        if (!cfg_.allow_diag && std::abs(dx) + std::abs(dy) + std::abs(dz) > 1) {
                            continue;
                        }
                        const rog_map::Vec3i succ_id = current + rog_map::Vec3i(dx, dy, dz);
                        if (!insideLocalMap(succ_id)) {
                            continue;
                        }
                        if (!sameId(succ_id, goal_idx_) && visited.find(idToKey(succ_id)) != visited.end()) {
                            continue;
                        }
                        DStarNode *succ = findNode(succ_id);
                        if (!succ) {
                            continue;
                        }
                        const rog_map::Vec3i new_delta = succ_id - current;
                        const double nominal_score = edgeCost(current, succ_id) + succ->g;
                        if (!std::isfinite(nominal_score)) {
                            continue;
                        }

                        // Tie-break only: prefer smooth, goal-progressing paths when D* costs are equivalent.
                        const double turn_cost = directionChangeCost(last_delta, new_delta);
                        const double goal_dist = heuristic(succ_id, goal_idx_);
                        const double z_cost = std::abs(new_delta.z()) > 0 ? 1.0 : 0.0;
                        const double tie_score = kExtractTurnWeight * turn_cost +
                                                 kExtractGoalWeight * goal_dist +
                                                 kExtractVerticalWeight * z_cost;
                        if (nominal_score < best_nominal_score - kExtractTieEps ||
                            (std::fabs(nominal_score - best_nominal_score) <= kExtractTieEps &&
                             tie_score < best_tie_score)) {
                            best_nominal_score = nominal_score;
                            best_tie_score = tie_score;
                            best_id = succ_id;
                        }
                    }
                }
            }

            if (sameId(best_id, current) || !std::isfinite(best_nominal_score)) {
                return NO_PATH;
            }

            traveled_grid += (best_id - current).cast<double>().norm();
            last_delta = best_id - current;
            current = best_id;
            visited.insert(idToKey(current));
            globalIndexToPos(current, current_pos);
            out_path.push_back(current_pos);

            if (searching_horizon > 0.0 && traveled_grid > max_grid) {
                return REACH_HORIZON;
            }
        }

        if (!out_path.empty()) {
            out_path.front() = original_start_pt;
        }
        return goal_out_local_map ? REACH_HORIZON : REACH_GOAL;
    }

    RET_CODE DStarLite::searchOrRepair(const rog_map::Vec3f &start_pt,
                                       const rog_map::Vec3f &goal_pt,
                                       const int &flag,
                                       const double &searching_horizon,
                                       rog_map::vec_Vec3f &out_path,
                                       const double &time_out) {
        const double effective_time_out = time_out > 0.0 ? time_out : cfg_.time_out;
        const double start_time = ros_ptr_->getSimTime();
        out_path.clear();

        const bool had_initialized = initialized_;
        const int old_flag = last_flag_;
        const rog_map::Vec3i old_goal_idx = goal_idx_;
        const rog_map::Vec3i old_local_map_center_id = local_map_center_id_g_;

        rog_map::Vec3f local_start_pt;
        rog_map::Vec3f local_goal_pt;
        bool goal_out_local_map = false;
        const RET_CODE setup_ret = setup(start_pt, goal_pt, flag, searching_horizon,
                                         local_start_pt, local_goal_pt, goal_out_local_map);
        if (setup_ret != SUCCESS) {
            return setup_ret;
        }
        (void)local_start_pt;

        const rog_map::Vec3i new_start_idx = start_idx_;
        const rog_map::Vec3i new_goal_idx = goal_idx_;
        const bool goal_changed = had_initialized && !sameId(old_goal_idx, new_goal_idx);
        const bool local_window_shifted = had_initialized &&
                                          !sameId(old_local_map_center_id, local_map_center_id_g_);
        const bool need_initialize = !had_initialized ||
                                     old_flag != flag ||
                                     goal_changed ||
                                     local_window_shifted ||
                                     findNode(new_goal_idx) == nullptr;

        if (need_initialize) {
            initialize(new_start_idx, new_goal_idx);
            last_flag_ = flag;
            std::vector<rog_map::Vec3i> changed_cells;
            detectChangedCells(changed_cells);
        } else {
            km_ += heuristic(last_start_idx_, new_start_idx);
            last_start_idx_ = new_start_idx;
            start_idx_ = new_start_idx;

            std::vector<rog_map::Vec3i> changed_cells;
            detectChangedCells(changed_cells);
            for (const auto &id: changed_cells) {
                updateVertex(getNode(id));
                for (int dx = -1; dx <= 1; dx++) {
                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dz = -1; dz <= 1; dz++) {
                            if (dx == 0 && dy == 0 && dz == 0) {
                                continue;
                            }
                            if (!cfg_.allow_diag && std::abs(dx) + std::abs(dy) + std::abs(dz) > 1) {
                                continue;
                            }
                            const rog_map::Vec3i pred_id = id + rog_map::Vec3i(dx, dy, dz);
                            if (insideLocalMap(pred_id)) {
                                updateVertex(getNode(pred_id));
                            }
                        }
                    }
                }
            }
        }

        computeShortestPath(start_time, effective_time_out);
        if ((ros_ptr_->getSimTime() - start_time) > effective_time_out) {
            fmt::print(fg(fmt::color::indian_red),
                       " -- [D* Lite] Path repairing exceeded {} seconds.\n",
                       effective_time_out);
            return TIME_OUT;
        }

        if (cfg_.visual_process) {
            ros_ptr_->vizAstarPoints(start_pt, Color::Green(), "dstar_start_pt", 0.3, 1);
            ros_ptr_->vizAstarPoints(local_goal_pt, Color::Blue(), "dstar_goal_pt", 0.3, 1);
        }

        RET_CODE extract_ret = extractPath(start_pt, goal_out_local_map, searching_horizon, out_path);
        if (!need_initialize && (extract_ret == REACH_HORIZON || extract_ret == REACH_GOAL) &&
            pathTooIndirect(out_path, start_pt, local_goal_pt)) {
            resetSearch();
            setup(start_pt, goal_pt, flag, searching_horizon,
                  local_start_pt, local_goal_pt, goal_out_local_map);
            initialize(start_idx_, goal_idx_);
            last_flag_ = flag;
            std::vector<rog_map::Vec3i> changed_cells;
            detectChangedCells(changed_cells);
            computeShortestPath(start_time, effective_time_out);
            extract_ret = extractPath(start_pt, goal_out_local_map, searching_horizon, out_path);
        }

        return extract_ret;
    }
}
