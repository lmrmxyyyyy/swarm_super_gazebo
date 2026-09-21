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

#include <memory>

#include <traj_opt/config.hpp>
#include <traj_opt/exp_traj_optimizer_s4.h>
#include <traj_opt/minco.h>

#include <data_structure/base/polytope.h>
#include <data_structure/base/trajectory.h>

#include <utils/header/type_utils.hpp>
#include <utils/header/scope_timer.hpp>
#include <utils/optimization/lbfgs.h>
#include <utils/geometry/geometry_utils.h>
#include <utils/optimization/optimization_utils.h>

#include <ros_interface/ros_interface.hpp>


namespace traj_opt {

    using super_utils::TimeConsuming;

    using namespace geometry_utils;
    using namespace math_utils;
    using namespace optimization_utils;

    class BackupTrajOpt {
    private:
        traj_opt::Config cfg_;
        ros_interface::RosInterface::Ptr ros_ptr_;

        std::ofstream failed_traj_log;
        std::ofstream penalty_log;

        struct OptimizationVariables {
            double rho;
            int iter_num{0};
            int pos_constraint_type;
            bool block_energy_cost;
            double smooth_eps;
            int integral_res;
            flatness::FlatnessMap quadrotor_flatness;

            Eigen::Matrix3Xd gradByPoints;
            Eigen::VectorXd gradByTimes;
            Eigen::MatrixX3d partialGradByCoeffs;
            Eigen::VectorXd partialGradByTimes;
            bool default_init{true};
            bool given_init_ts_and_ps{false};
            int piece_num;
            Eigen::Matrix3Xd points;
            Eigen::VectorXd times;
            Eigen::VectorXd magnitudeBounds, penaltyWeights;

            Eigen::Matrix3Xd init_path;
            Eigen::Matrix3Xd waypoint_attractor;
            Eigen::VectorXd waypoint_attractor_dead_d;

            PolyhedronH hPolytope;
            PolyhedronV vPolytope;

            MINCO_S4NU minco;

            StatePVAJ headPVAJ;
            StatePVAJ tailPVAJ;
            vec_E<Vec3f> guide_path;
            vector<double> guide_t;

            int temporalDim, spatialDim;

            VecDf penalty_log;

            bool debug_en;
            double ts;
            double gradTs;

            Trajectory exp_traj;
            double min_ts, max_ts;
            double weight_ts;
            vector<SwarmPrediction> swarm_predictions;
            bool local_density_en{false};
            double local_density_radius_x{0.5}, local_density_radius_y{0.5}, local_density_radius_z{0.35};
            double local_density_smooth_eps{0.1}, local_density_time_inflation{0.1};
            double local_density_weight{0.0};
            double trigger_density_weight{0.0};
            double trigger_static_clearance{0.8};
            double trigger_static_weight{1.0};
            double trigger_swarm_weight{1.0};
            double ts_delay_weight{0.0};
            double ts_delay_high_order_weight{0.0};
            double ts_delay_deadband{0.0};
            double reference_ts{0.0};
            double ts_anchor_weight{0.0};
            double ts_anchor_deadband{0.0};
            Vec3f tail_anchor_pos{Vec3f::Zero()};
            double tail_anchor_weight{0.0};
            double tail_anchor_deadband{0.0};
            double time_balance_weight{0.0};

            double init_ts;
            VecDf init_t_vec;
            // init_ps include the optimized fina p
            vec_Vec3f init_ps;

            double given_init_ts;
            VecDf given_init_t_vec;
            // init_ps include the optimized fina p
            vec_Vec3f given_init_ps;

        } opt_vars{};

    private:
        /// Optimization functions
        static double costFunctional(void *ptr,
                                     const Eigen::VectorXd &x,
                                     Eigen::VectorXd &g);

        static void constraintsFunctional(const Eigen::VectorXd &T,
                                          const Eigen::MatrixX3d &coeffs,
                                          const PolyhedronH &hPoly,
                                          const double &smoothFactor,
                                          const int &integralResolution,
                                          const Eigen::VectorXd &magnitudeBounds,
                                          const Eigen::VectorXd &penaltyWeights,
                                          flatness::FlatnessMap &flatMap,
                                          const vector<SwarmPrediction> &swarm_predictions,
                                          const double trajectory_start_wt,
                                          const bool local_density_en,
                                          const double local_density_radius_x,
                                          const double local_density_radius_y,
                                          const double local_density_radius_z,
                                          const double local_density_smooth_eps,
                                          const double local_density_time_inflation,
                                          const double local_density_weight,
                                          double &cost,
                                          Eigen::VectorXd &gradT,
                                          Eigen::MatrixX3d &gradC,
                                          double &grad_start_wt,
                                          VecDf &pena_log);

        bool processCorridor();

        static int
        visualizeProgress(void *instance, const Eigen::VectorXd &x, const Eigen::VectorXd &g, const double fx,
                          const double step, const int k, const int ls);

        bool setupProblemAndCheck();

        static bool SimplifySFC(const Vec3f &head_p, const Vec3f &tail_p, PolytopeVec &sfcs);


        double optimize(Trajectory &traj, const double &relCostTol);

    public:

        explicit BackupTrajOpt(const traj_opt::Config &cfg , const ros_interface::RosInterface::Ptr &ros_ptr);

        ~BackupTrajOpt() {
            penalty_log.close();
        }

        typedef std::shared_ptr<BackupTrajOpt> Ptr;

        bool checkTrajMagnitudeBound(Trajectory &out_traj);

        bool optimize(const Trajectory &exp_traj,
                      const double &t_0,
                      const double &t_e,
                      const double &heu_ts,
                      const VecDf &heu_end_pt,
                      double &heu_dur,
                      const Polytope &sfc,
                      Trajectory &out_traj,
                      double &out_ts,
                      const vector<SwarmPrediction> &swarm_predictions,
                      const bool &debug = false);

        bool optimize(const Trajectory &exp_traj,
                      const double &t_0,
                      const double &t_e,
                      const double &heu_ts,
                      const Polytope &sfc,
                      const VecDf & init_t_vec,
                      const vec_Vec3f &init_ps,
                      Trajectory &out_traj,
                      double & out_ts);

        void getInitValue(double & ts, VecDf&  times, vec_Vec3f & ps) {
            ts = opt_vars.init_ts;
            times = opt_vars.init_t_vec;
            ps = opt_vars.init_ps;
        }

    };
}
