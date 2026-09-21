/**
* D* Lite frontend path search for SUPER.
*/

#pragma once

#include <Eigen/Dense>
#include <map>
#include <memory>
#include <queue>
#include <tuple>
#include <vector>

#include "path_search/astar.h"
#include "path_search/config.hpp"
#include "rog_map_ros/rog_map_ros1.hpp"
#include "ros_interface/ros_interface.hpp"
#include "utils/header/type_utils.hpp"

namespace path_search {
    using namespace super_utils;

    struct DStarKey {
        double k1{inf};
        double k2{inf};
    };

    struct DStarNode {
        rog_map::Vec3i id_g{0, 0, 0};
        double g{inf};
        double rhs{inf};
    };

    struct DStarQueueNode {
        DStarNode *node{nullptr};
        DStarKey key;
    };

    class DStarQueueComparator {
    public:
        bool operator()(const DStarQueueNode &a, const DStarQueueNode &b) const;
    };

    class DStarLite {
        rog_map::ROGMapROS::Ptr map_ptr_;
        ros_interface::RosInterface::Ptr ros_ptr_;
        PathSearchConfig cfg_;

        std::map<std::tuple<int, int, int>, std::unique_ptr<DStarNode>> nodes_;
        std::map<std::tuple<int, int, int>, bool> occupancy_cache_;
        std::priority_queue<DStarQueueNode, std::vector<DStarQueueNode>, DStarQueueComparator> open_set_;

        rog_map::vec_Vec3i neighbor_list_;

        rog_map::Vec3f local_map_center_d_{0, 0, 0};
        rog_map::Vec3f local_map_min_d_{0, 0, 0};
        rog_map::Vec3f local_map_max_d_{0, 0, 0};
        rog_map::Vec3i local_map_center_id_g_{0, 0, 0};

        rog_map::Vec3i start_idx_{0, 0, 0};
        rog_map::Vec3i last_start_idx_{0, 0, 0};
        rog_map::Vec3i goal_idx_{0, 0, 0};

        bool initialized_{false};
        bool use_inf_map_{false};
        bool use_prob_map_{false};
        bool unknown_as_occ_{false};
        bool unknown_as_free_{false};
        bool use_inf_neighbor_{false};
        int last_flag_{0};
        double resolution_{0.1};
        double km_{0.0};

        static std::tuple<int, int, int> idToKey(const rog_map::Vec3i &id);
        static bool keyLess(const DStarKey &a, const DStarKey &b);
        static bool keyEqual(const DStarKey &a, const DStarKey &b);

        DStarNode *getNode(const rog_map::Vec3i &id);
        DStarNode *findNode(const rog_map::Vec3i &id);

        void resetSearch();
        RET_CODE setup(const rog_map::Vec3f &start_pt,
                       const rog_map::Vec3f &goal_pt,
                       const int &flag,
                       const double &searching_horizon,
                       rog_map::Vec3f &local_start_pt,
                       rog_map::Vec3f &local_goal_pt,
                       bool &goal_out_local_map);

        void posToGlobalIndex(const rog_map::Vec3f &pos, rog_map::Vec3i &id_g) const;
        void globalIndexToPos(const rog_map::Vec3i &id_g, rog_map::Vec3f &pos) const;
        bool insideLocalMap(const rog_map::Vec3i &id_g) const;
        bool insideLocalMap(const rog_map::Vec3f &pos) const;

        double heuristic(const rog_map::Vec3i &a, const rog_map::Vec3i &b) const;
        double edgeCost(const rog_map::Vec3i &from, const rog_map::Vec3i &to);
        DStarKey calculateKey(DStarNode *node) const;
        void pushIfInconsistent(DStarNode *node);
        bool queueTop(DStarQueueNode &out);
        bool isBlocked(const rog_map::Vec3i &id);
        bool neighborHaveOne(const rog_map::GridType &type, const rog_map::Vec3i &src_id);
        rog_map::GridType queryGridType(const rog_map::Vec3i &id);

        void initialize(const rog_map::Vec3i &start_idx, const rog_map::Vec3i &goal_idx);
        void updateVertex(DStarNode *node);
        void computeShortestPath(const double &start_time, const double &time_out);
        void detectChangedCells(std::vector<rog_map::Vec3i> &changed_cells);
        RET_CODE extractPath(const rog_map::Vec3f &original_start_pt,
                             const bool &goal_out_local_map,
                             const double &searching_horizon,
                             rog_map::vec_Vec3f &out_path);

    public:
        DStarLite(const std::string &cfg_path,
                  const ros_interface::RosInterface::Ptr &ros_ptr,
                  rog_map::ROGMapROS::Ptr rm);

        typedef std::shared_ptr<DStarLite> Ptr;

        void reset();

        void setFineInfNeighbors(const int &neighbor_step);

        RET_CODE searchOrRepair(const rog_map::Vec3f &start_pt,
                                const rog_map::Vec3f &goal_pt,
                                const int &flag,
                                const double &searching_horizon,
                                rog_map::vec_Vec3f &out_path,
                                const double &time_out = -1.0);
    };
}
