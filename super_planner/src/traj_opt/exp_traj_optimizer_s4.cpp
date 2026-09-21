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

#include <traj_opt/exp_traj_optimizer_s4.h>
#include <utils/optimization/lbfgs.h>
#include <ros_interface/ros_interface.hpp>
#include <array>
#include <tuple>

#define POS_IDX 1
#define VEL_IDX 2
#define ACC_IDX 3
#define JER_IDX 4
#define ATT_IDX 5
#define OMG_IDX 6
#define THR_IDX 7

using namespace traj_opt;
using namespace color_text;
using namespace super_utils;
using namespace math_utils;
using namespace optimization_utils;

using Vec8f = Eigen::Matrix<double, 8, 1>;
using Mat83f = Eigen::Matrix<double, 8, 3>;

namespace {
    constexpr double kMinStablePieceTime = 0.05;
    constexpr double kMinMappedPieceTime = 1.0e-3;
    constexpr double kInvalidCost = 1.0e12;

    double squaredBoundViolationTolerance(const double bound, const double margin) {
        const double allowed_bound = bound * (1.0 + margin);
        return allowed_bound * allowed_bound - bound * bound;
    }

    double thrustViolationTolerance(const traj_opt::Config &cfg) {
        const double thrust_min = cfg.min_acc_thr * cfg.mass;
        const double thrust_max = cfg.max_acc_thr * cfg.mass;
        const double radius = 0.5 * std::abs(thrust_max - thrust_min);
        const double bound = std::max(std::abs(thrust_min), std::abs(thrust_max));
        const double allowed_radius = radius + bound * cfg.penna_margin;
        return allowed_radius * allowed_radius - radius * radius;
    }

    bool finiteGuide(const vec_E<Vec3f> &guide_path, const std::vector<double> &guide_t) {
        if (guide_path.empty() || guide_path.size() != guide_t.size()) {
            return false;
        }
        double last_t = -std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < guide_path.size(); ++i) {
            if (!guide_path[i].allFinite() || !std::isfinite(guide_t[i]) || guide_t[i] < last_t) {
                return false;
            }
            last_t = guide_t[i];
        }
        return true;
    }

    bool mergeFiniteDynamicPlanes(const MatD4f &base_planes,
                                  const MatD4f &dynamic_planes,
                                  MatD4f &merged_planes) {
        if (base_planes.cols() != 4 || dynamic_planes.cols() != 4 || dynamic_planes.rows() <= 0) {
            return false;
        }

        std::vector<int> valid_rows;
        valid_rows.reserve(dynamic_planes.rows());
        for (int i = 0; i < dynamic_planes.rows(); ++i) {
            const auto row = dynamic_planes.row(i);
            const double normal_norm = row.head<3>().norm();
            if (row.allFinite() && std::isfinite(normal_norm) && normal_norm > 1.0e-6) {
                valid_rows.push_back(i);
            }
        }
        if (valid_rows.empty()) {
            return false;
        }

        merged_planes.resize(base_planes.rows() + static_cast<int>(valid_rows.size()), 4);
        merged_planes.topRows(base_planes.rows()) = base_planes;
        for (size_t i = 0; i < valid_rows.size(); ++i) {
            merged_planes.row(base_planes.rows() + static_cast<int>(i)) = dynamic_planes.row(valid_rows[i]);
        }
        return true;
    }

    void forwardMapTauToBoundedT(const VecDf &tau, VecDf &times) {
        gcopter::forwardMapTauToT(tau, times);
        times.array() += kMinStablePieceTime;
    }

    template<typename EIGENVEC>
    void backwardMapBoundedTToTau(const VecDf &times, EIGENVEC &tau) {
        const VecDf positive_times =
                (times.array() - kMinStablePieceTime).max(kMinMappedPieceTime).matrix();
        gcopter::backwardMapTToTau(positive_times, tau);
    }

    double sigmoid(const double value) {
        if (value >= 0.0) {
            const double exp_neg = std::exp(-value);
            return 1.0 / (1.0 + exp_neg);
        }
        const double exp_pos = std::exp(value);
        return exp_pos / (1.0 + exp_pos);
    }

    void localEllipsoidOccupancy(const Vec3f &self_pos,
                                 const Vec3f &other_pos,
                                 const double radius_x,
                                 const double radius_y,
                                 const double radius_z,
                                 const double smooth_eps,
                                 double &occupancy,
                                 Vec3f &grad_by_self_pos) {
        const Vec3f delta = other_pos - self_pos;
        const double eps = std::max(1.0e-3, smooth_eps);
        const Vec3f radii(std::max(1.0e-3, radius_x),
                          std::max(1.0e-3, radius_y),
                          std::max(1.0e-3, radius_z));
        const Vec3f normalized = delta.cwiseQuotient(radii);
        const double normalized_distance = std::sqrt(normalized.squaredNorm() + eps * eps);

        occupancy = sigmoid((1.0 - normalized_distance) / eps);
        const double occupancy_derivative = occupancy * (1.0 - occupancy);
        grad_by_self_pos =
                occupancy_derivative * delta.cwiseQuotient(radii.cwiseProduct(radii)) /
                (normalized_distance * eps);
    }

    void evaluateLocalDensity(const Vec3f &self_pos,
                              const double query_wt,
                              const vector<SwarmPrediction> &predictions,
                              const double radius_x,
                              const double radius_y,
                              const double radius_z,
                              const double smooth_eps,
                              const double time_inflation,
                              double &neighbor_count,
                              Vec3f &grad_by_self_pos,
                              double &grad_by_query_time) {
        neighbor_count = 0.0;
        grad_by_self_pos.setZero();
        grad_by_query_time = 0.0;

        const double inflation = std::max(0.0, time_inflation);
        const std::array<double, 3> offsets{{-inflation, 0.0, inflation}};
        const std::array<double, 3> weights = inflation > 1.0e-6
                                              ? std::array<double, 3>{{0.25, 0.5, 0.25}}
                                              : std::array<double, 3>{{0.0, 1.0, 0.0}};

        for (const auto &prediction: predictions) {
            double prediction_occupancy = 0.0;
            Vec3f prediction_grad = Vec3f::Zero();
            double prediction_time_grad = 0.0;
            double valid_weight = 0.0;
            for (size_t k = 0; k < offsets.size(); ++k) {
                if (weights[k] <= 0.0) {
                    continue;
                }
                Vec3f other_pos, other_vel;
                if (!prediction.sample(query_wt + offsets[k], other_pos, other_vel)) {
                    continue;
                }

                double occupancy;
                Vec3f occupancy_grad;
                localEllipsoidOccupancy(self_pos, other_pos, radius_x, radius_y, radius_z,
                                        smooth_eps, occupancy, occupancy_grad);
                prediction_occupancy += weights[k] * occupancy;
                prediction_grad += weights[k] * occupancy_grad;
                prediction_time_grad += weights[k] * (-occupancy_grad.dot(other_vel));
                valid_weight += weights[k];
            }
            if (valid_weight <= 0.0) {
                continue;
            }
            neighbor_count += prediction_occupancy / valid_weight;
            grad_by_self_pos += prediction_grad / valid_weight;
            grad_by_query_time += prediction_time_grad / valid_weight;
        }
    }
}

bool SwarmPrediction::sample(const double query_wt, Vec3f &position, Vec3f &velocity) const {
    if (positions.empty() || positions.size() != times.size() || !std::isfinite(query_wt)) {
        return false;
    }
    const double relative_t = query_wt - start_wt;
    if (relative_t < times.front() || relative_t > times.back()) {
        return false;
    }
    if (positions.size() == 1) {
        position = positions.front();
        velocity.setZero();
        return true;
    }

    auto upper = std::upper_bound(times.begin(), times.end(), relative_t);
    size_t next_idx = static_cast<size_t>(std::distance(times.begin(), upper));
    if (next_idx == 0) {
        next_idx = 1;
    } else if (next_idx >= times.size()) {
        next_idx = times.size() - 1;
    }
    const size_t prev_idx = next_idx - 1;
    const double dt = std::max(1.0e-3, times[next_idx] - times[prev_idx]);
    const double alpha = std::min(1.0, std::max(0.0, (relative_t - times[prev_idx]) / dt));
    velocity = (positions[next_idx] - positions[prev_idx]) / dt;
    position = (1.0 - alpha) * positions[prev_idx] + alpha * positions[next_idx];
    return position.allFinite() && velocity.allFinite();
}

void ExpTrajOpt::constraintsFunctional(const VecDf &T,
                                       const MatD3f &coeffs,
                                       const VecDi &hIdx,
                                       const PolyhedraH &hPolys,
                                       const Mat3Df &waypoint_attractor,
                                       const VecDf &waypoint_attractor_dead_d,
                                       const double &smoothFactor,
                                       const int &integralResolution,
                                       const VecDf &magnitudeBounds,
                                       const VecDf &penaltyWeights,
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
        // outputs
                                       double &cost,
                                       VecDf &gradT,
                                       MatD3f &gradC,
                                       VecDf &pena_log) {
    /* 1) define some varible alias*/
    const auto &vmax = magnitudeBounds[0];
    const auto &amax = magnitudeBounds[1];
    const auto &jmax = magnitudeBounds[2];
    const auto &omgmax = magnitudeBounds[3];
    const auto &accthrmin = magnitudeBounds[4];
    const auto &accthrmax = magnitudeBounds[5];

    const auto &vmaxSqr = vmax * vmax;
    const auto &amaxSqr = amax * amax;
    const auto &jmaxSqr = jmax * jmax;
    const auto &omgmaxSqr = omgmax * omgmax;

    const auto &thrustMean = 0.5 * (accthrmax + accthrmin);
    const auto &thrustRadi = 0.5 * std::abs(accthrmax - accthrmin);
    const auto &thrustSqrRadi = thrustRadi * thrustRadi;

    const auto &weightPos = penaltyWeights[0];
    const auto &weightVel = penaltyWeights[1];
    const auto &weightAcc = penaltyWeights[2];
    const auto &weightJer = penaltyWeights[3];
    const auto &weightAtt = penaltyWeights[4];
    const auto &weightOmg = penaltyWeights[5];
    const auto &weightAccThr = penaltyWeights[6];

    const auto &piece_num = T.size();

    const double integralFrac = 1.0 / integralResolution;
    VecDf max_pena(8);
    max_pena.setZero();

    /* 2) add integral cost */

    double piece_start_t = 0.0;
    for (int i = 0; i < piece_num; i++) {
        const Mat83f &c = coeffs.block<8, 3>(i * 8, 0);
        const auto &step = T(i) * integralFrac;
        for (int j = 0; j <= integralResolution; j++) {
            double s1 = j * step;
            double s2 = s1 * s1;
            double s3 = s2 * s1;
            double s4 = s2 * s2;
            double s5 = s4 * s1;
            double s6 = s4 * s2;
            double s7 = s4 * s3;
            Vec8f beta0, beta1, beta2, beta3, beta4;
            beta0 << 1.0, s1, s2, s3, s4, s5, s6, s7;
            beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
            beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
            beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2, 120.0 * s3, 210.0 * s4;
            beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1, 360.0 * s2, 840.0 * s3;
            //beta5 << 0.0, 0.0, 0.0, 0., 0.0, 120.0, 720.0 * s1, 2520.0 * s2;

            const Vec3f pos = c.transpose() * beta0;
            const Vec3f vel = c.transpose() * beta1;
            const Vec3f acc = c.transpose() * beta2;
            const Vec3f jer = c.transpose() * beta3;
            const Vec3f sna = c.transpose() * beta4;

            double tmp_cost{0.0};
            Vec3f gradPos{0, 0, 0}, gradVel{0, 0, 0}, gradAcc{0, 0, 0}, gradJer{0, 0, 0};
            double local_density_time_grad{0.0};

            /* 2.1  For position cost */
            const auto &L = hIdx(i);
            const auto &K = hPolys[L].rows();
            if (weightPos > 0) {
                for (int k = 0; k < K; k++) {
                    const Vec3f outerNormal = hPolys[L].block<1, 3>(k, 0);
                    const double violaPos = outerNormal.dot(pos) + hPolys[L](k, 3);
                    if (violaPos > max_pena(POS_IDX)) max_pena(POS_IDX) = violaPos;
                    double violaPosPena, violaPosPenaD;
                    if (gcopter::smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD)) {
                        gradPos += weightPos * violaPosPenaD * outerNormal;
                        tmp_cost += weightPos * violaPosPena;
                    }
                }
            }

            /* 2.2  For attract point cost  *///吸引点 attractor 惩罚
            if (weightAtt > 0.0) {
                const auto is_waypoint = (j == 0) && (i != 0);
                const auto is_end = ((j == integralResolution) && (i != piece_num - 1));
                const auto idx = is_end ? i : i - 1;

                if (is_waypoint || is_end) {
                    Vec3f p_a = pos - waypoint_attractor.col(idx);
                    const auto &violaAtt =
                            p_a.squaredNorm() - waypoint_attractor_dead_d(idx) * waypoint_attractor_dead_d(idx);
                    double violaAttPena, violaAttPenaD;
                    if (violaAtt > max_pena(ATT_IDX)) max_pena(ATT_IDX) = violaAtt;
                    if (gcopter::smoothedL1(violaAtt, smoothFactor, violaAttPena, violaAttPenaD)) {
                        gradPos += weightAtt * violaAttPenaD * 2.0 * p_a;
                        tmp_cost += weightAtt * violaAttPena;
                    }
                }
            }

            if (local_density_en && local_density_weight > 0.0 && !swarm_predictions.empty()) {
                double neighbor_count;
                Vec3f neighbor_count_grad;
                double neighbor_count_time_grad;
                const double query_wt = trajectory_start_wt + piece_start_t + s1;
                evaluateLocalDensity(pos, query_wt, swarm_predictions,
                                     local_density_radius_x, local_density_radius_y, local_density_radius_z,
                                     local_density_smooth_eps, local_density_time_inflation,
                                     neighbor_count, neighbor_count_grad, neighbor_count_time_grad);

                const double density_cost = local_density_weight * neighbor_count * neighbor_count;
                const double density_scale = 2.0 * local_density_weight * neighbor_count;
                tmp_cost += density_cost;
                gradPos += density_scale * neighbor_count_grad;
                local_density_time_grad = density_scale * neighbor_count_time_grad;
            }

            /* 2.3 For vel cost  */
            const auto &violaVel = vel.squaredNorm() - vmaxSqr;
            double violaVelPena, violaVelPenaD;
            if (weightVel > 0 && gcopter::smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD)) {
                gradVel += weightVel * violaVelPenaD * 2.0 * vel;
                tmp_cost += weightVel * violaVelPena;
                if (violaVel > max_pena(VEL_IDX)) max_pena(VEL_IDX) = violaVel;
            }

            /* 2.4 For acc cost  */
            const auto &violaAcc = acc.squaredNorm() - amaxSqr;
            double violaAccPena, violaAccPenaD;
            if (weightAcc > 0 && gcopter::smoothedL1(violaAcc, smoothFactor, violaAccPena, violaAccPenaD)) {
                gradAcc += weightAcc * violaAccPenaD * 2.0 * acc;
                tmp_cost += weightAcc * violaAccPena;
                if (violaAcc > max_pena(ACC_IDX)) max_pena(ACC_IDX) = violaAcc;
            }

            /* 2.5 For acc cost  */
            const auto &violaJer = jer.squaredNorm() - jmaxSqr;
            double violaJerPena, violaJerPenaD;
            if (weightJer > 0 && gcopter::smoothedL1(violaJer, smoothFactor, violaJerPena, violaJerPenaD)) {
                gradJer += weightJer * violaJerPenaD * 2.0 * jer;
                tmp_cost += weightJer * violaJerPena;
                if (violaJer > max_pena(JER_IDX)) max_pena(JER_IDX) = violaJer;
            }

            Vec3f totalGradPos{0.0, 0.0, 0.0}, totalGradVel{0.0, 0.0, 0.0},
                    totalGradAcc{0.0, 0.0, 0.0}, totalGradJer{0.0, 0.0, 0.0};

            /* 2.6  For omg amd thr cost  */
            if (weightOmg > 0 && weightAccThr > 0) {
                double thr;
                Vec4f quat;
                Vec3f omg;
                flatMap.forward(vel, acc, jer, 0.0, 0.0, thr, quat, omg);
                const auto &violaOmg = omg.squaredNorm() - omgmaxSqr;
                const auto &violaThrust = (thr - thrustMean) * (thr - thrustMean) - thrustSqrRadi;

                /* 2.6.1  For omg cost  */
                double violaOmgPena, violaOmgPenaD;
                Vec3f gradOmg{0, 0, 0};
                if (weightOmg > 0 && gcopter::smoothedL1(violaOmg, smoothFactor, violaOmgPena, violaOmgPenaD)) {
                    gradOmg += weightOmg * violaOmgPenaD * 2.0 * omg;
                    tmp_cost += weightOmg * violaOmgPena;
                    if (violaOmg > max_pena(OMG_IDX)) max_pena(OMG_IDX) = violaOmg;
                }

                /* 2.6.2  For thr cost  */
                double violaThrustPena, violaThrustPenaD;
                double gradThr{0.0};
                if (weightAccThr > 0 &&
                    gcopter::smoothedL1(violaThrust, smoothFactor, violaThrustPena, violaThrustPenaD)) {
                    gradThr += weightAccThr * violaThrustPenaD * 2.0 * (thr - thrustMean);
                    tmp_cost += weightAccThr * violaThrustPena;
                    if (violaThrust > max_pena(THR_IDX)) max_pena(THR_IDX) = violaThrust;
                }
                double totalGradPsi{0.0}, totalGradPsiD{0.0};
                flatMap.backward(gradPos, gradVel, gradAcc, gradJer, gradThr, Vec4f(0, 0, 0, 0), gradOmg,
                                 totalGradPos, totalGradVel, totalGradAcc, totalGradJer,
                                 totalGradPsi, totalGradPsiD);
            } else {
                totalGradPos = gradPos;
                totalGradVel = gradVel;
                totalGradAcc = gradAcc;
                totalGradJer = gradJer;
            }

            const auto node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
            const double alpha = j * integralFrac;
            gradC.block<8, 3>(i * 8, 0) += (beta0 * totalGradPos.transpose() +
                                            beta1 * totalGradVel.transpose() +
                                            beta2 * totalGradAcc.transpose() +
                                            beta3 * totalGradJer.transpose()) *
                                           node * step;
            gradT(i) += (totalGradPos.dot(vel) +
                         totalGradVel.dot(acc) +
                         totalGradAcc.dot(jer) +
                         totalGradJer.dot(sna)) *
                        alpha * node * step +
                        node * integralFrac * tmp_cost;
            for (int previous_piece = 0; previous_piece < i; ++previous_piece) {
                gradT(previous_piece) += node * step * local_density_time_grad;
            }
            gradT(i) += node * step * alpha * local_density_time_grad;
            cost += node * step * tmp_cost;
        }
        piece_start_t += T(i);
    }

    /* 3) log all violations */
    pena_log.tail(7) = max_pena.tail(7);
}


/*
 * @ brief: This is the callback function of the L-BFGS solver
 *
 */
double ExpTrajOpt::costFunctional(void *ptr,
                                  const VecDf &x,
                                  VecDf &g) {
    /* 1) Decode the pointer */
    OptimizationVariables &obj = *static_cast<OptimizationVariables *>(ptr);
    const auto &dimTau = obj.temporalDim;
    const auto &dimXi = obj.spatialDim;
    const auto &weightT = obj.rho;
    const auto &vPolyIdx = obj.vPolyIdx;
    const auto &vPolytopes = obj.vPolytopes;
    const auto &hPolyIdx = obj.hPolyIdx;
    const auto &hPolytopes = obj.hPolytopes;
    const auto &waypoint_attractor = obj.waypoint_attractor;
    const auto &waypoint_attractor_dead_d = obj.waypoint_attractor_dead_d;
    const auto &smooth_eps = obj.smooth_eps;
    const auto &integral_res = obj.integral_res;
    const auto &magnitudeBounds = obj.magnitudeBounds;
    const auto &penaltyWeights = obj.penaltyWeights;
    const auto &block_energy_cost = obj.block_energy_cost;

    auto &quadrotor_flatness = obj.quadrotor_flatness;

    obj.iter_num++;
    const auto &pos_constraint_type = obj.pos_constraint_type;

    const Eigen::Map<const VecDf> tau(x.data(), dimTau);
    const Eigen::Map<const VecDf> xi(x.data() + dimTau, dimXi);
    Eigen::Map<VecDf> gradTau(g.data(), dimTau);
    Eigen::Map<VecDf> gradXi(g.data() + dimTau, dimXi);

    /* 2) Reconstruct the optimization varibles */

    Mat3Df points;
    VecDf times;
    forwardMapTauToBoundedT(tau, times);
    if (!times.allFinite()) {
        g.setZero();
        obj.penalty_log.setZero();
        obj.penalty_log(0) = kInvalidCost;
        return kInvalidCost;
    }
    switch (pos_constraint_type) {
        case 1: {
            VecDf xi_e = xi;
            points = Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>>(xi_e.data(), 3, xi_e.size() / 3);
            break;
        }
        default: {
            gcopter::forwardP(xi, vPolyIdx, vPolytopes, points);
            break;
        }
    }
    if (!points.allFinite()) {
        g.setZero();
        obj.penalty_log.setZero();
        obj.penalty_log(0) = kInvalidCost;
        return kInvalidCost;
    }

    /* 3) Compute the energy const and gradient */
    // ③ 计算轨迹能量项（最小控制量 cost）
    double cost{0};
    obj.minco.setParameters(points, times);
    if (!obj.minco.getCoeffs().allFinite()) {
        g.setZero();
        obj.penalty_log.setZero();
        obj.penalty_log(0) = kInvalidCost;
        return kInvalidCost;
    }
    MatD3f partialGradByCoeffs(8 * times.size(), 3);
    VecDf partialGradByTimes(times.size());
    partialGradByCoeffs.setZero();
    partialGradByTimes.setZero();
    if (!block_energy_cost) {
        obj.minco.getEnergy(cost);
        obj.minco.getEnergyPartialGradByCoeffs(partialGradByCoeffs);
        obj.minco.getEnergyPartialGradByTimes(partialGradByTimes);
    }
    if (!std::isfinite(cost) || !partialGradByCoeffs.allFinite() || !partialGradByTimes.allFinite()) {
        g.setZero();
        obj.penalty_log.setZero();
        obj.penalty_log(0) = kInvalidCost;
        return kInvalidCost;
    }
    obj.penalty_log(0) = cost;

    /* 4) Compute the constrain cost and gradient  */
    //④ 添加约束惩罚项（碰撞、推力、加速度、吸引点等）
    constraintsFunctional(times, obj.minco.getCoeffs(),
                          hPolyIdx, hPolytopes,
                          waypoint_attractor, waypoint_attractor_dead_d,
                          smooth_eps, integral_res,
                          magnitudeBounds, penaltyWeights,
                          quadrotor_flatness, obj.swarm_predictions, obj.trajectory_start_wt,
                          obj.local_density_en, obj.local_density_radius_x, obj.local_density_radius_y,
                          obj.local_density_radius_z,
                          obj.local_density_smooth_eps, obj.local_density_time_inflation,
                          obj.local_density_weight,
                          cost, partialGradByTimes, partialGradByCoeffs, obj.penalty_log);
    if (!std::isfinite(cost) || !partialGradByCoeffs.allFinite() || !partialGradByTimes.allFinite()) {
        g.setZero();
        obj.penalty_log.setZero();
        obj.penalty_log(0) = kInvalidCost;
        return kInvalidCost;
    }

    /* 5) Propagate the gradient from CT to PT */
    Mat3Df gradByPoints;
    VecDf gradByTimes;
    obj.minco.propogateGrad(partialGradByCoeffs, partialGradByTimes,
                            gradByPoints, gradByTimes);
    if (!gradByPoints.allFinite() || !gradByTimes.allFinite()) {
        g.setZero();
        obj.penalty_log.setZero();
        obj.penalty_log(0) = kInvalidCost;
        return kInvalidCost;
    }
    cost += weightT * times.sum();
    gradByTimes.array() += weightT;

    /* 6) Propagate the gradient from PT to optimization varibles*/
    gcopter::propagateGradientTToTau(tau, gradByTimes, gradTau);
    switch (pos_constraint_type) {
        case 1: {
            MatDf gp = gradByPoints;
            gradXi = Eigen::Map<VecDf>(gp.data(), gp.size());
            break;
        }
        default: {
            gcopter::backwardGradP(xi, vPolyIdx, vPolytopes, gradByPoints, gradXi);
            gcopter::normRetrictionLayer(xi, vPolyIdx, vPolytopes, cost, gradXi);
            break;
        }
    }
    if (!std::isfinite(cost) || !gradTau.allFinite() || !gradXi.allFinite()) {
        g.setZero();
        obj.penalty_log.setZero();
        obj.penalty_log(0) = kInvalidCost;
        return kInvalidCost;
    }
    return cost;
}

static void truncateToSixDecimals(double &num) {
    num = std::trunc(num * 1e6) / 1e6; // 直接截断，无四舍五入
}

/*
 * @ brief: This function pre-process the corridor
 *
 */
bool ExpTrajOpt::processCorridor() {
    const long sizeCorridor = static_cast<long>(opt_vars.hPolytopes.size() - 1);

    opt_vars.vPolytopes.clear();
    opt_vars.vPolytopes.reserve(2 * sizeCorridor + 1);

    long nv;
    PolyhedronH curIH;
    PolyhedronV curIV, curIOB;
    opt_vars.waypoint_attractor.resize(3, sizeCorridor);
    opt_vars.waypoint_attractor_dead_d.resize(sizeCorridor);
    opt_vars.hOverlapPolytopes.resize(sizeCorridor);

    for (long i = 0; i < sizeCorridor; i++) {
        if (!geometry_utils::enumerateVs(opt_vars.hPolytopes[i], curIV)) {
            cout << YELLOW << " -- [SUPER] in [ GcopterExpS4::processCorridor]: Failed to enumerate corridor Vs." << RESET
                 << endl;
            return false;
        }
        nv = curIV.cols();
        curIOB.resize(3, nv);
        curIOB.col(0) = curIV.col(0);
        curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
        opt_vars.vPolytopes.push_back(curIOB);
        curIH.resize(opt_vars.hPolytopes[i].rows() + opt_vars.hPolytopes[i + 1].rows(), 4);
        curIH.topRows(opt_vars.hPolytopes[i].rows()) = opt_vars.hPolytopes[i];
        curIH.bottomRows(opt_vars.hPolytopes[i + 1].rows()) = opt_vars.hPolytopes[i + 1];
        opt_vars.hOverlapPolytopes[i] = curIH;
        Vec3f interior;
        const double &dis = geometry_utils::findInteriorDist(curIH, interior) / 2;
        opt_vars.waypoint_attractor.col(i) = curIV.colwise().mean();
        opt_vars.waypoint_attractor_dead_d(i) = dis;
        nv = curIV.cols();
        curIOB.resize(3, nv);
        curIOB.col(0) = curIV.col(0);
        curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
        opt_vars.vPolytopes.push_back(curIOB);
    }

    if (!geometry_utils::enumerateVs(opt_vars.hPolytopes.back(), curIV)) {
        cout << YELLOW << " -- [SUPER] in [ GcopterExpS4::processCorridor]: Failed to enumerate corridor Vs." <<
             RESET << endl;
        return false;
    }

    nv = curIV.cols();
    curIOB.resize(3, nv);
    curIOB.col(0) = curIV.col(0);
    curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
    opt_vars.vPolytopes.push_back(curIOB);
    return true;
}
//将多面体走廊（hPolytopes）转换为多面体顶点形式（vPolytopes），
//并结合引导轨迹 guide_path 提取每段走廊的 attractor（吸引点）用于轨迹初始化和优化。
bool ExpTrajOpt::processCorridorWithGuideTraj() {///
    // * 1) allocate memory for vertex
    // 1) corridor 段数 = 多面体个数 - 1
    const int sizeCorridor = static_cast<int>(opt_vars.hPolytopes.size() - 1);
    // 清空并准备空间（多面体顶点形式）
    opt_vars.vPolytopes.clear();
    opt_vars.vPolytopes.reserve(2 * sizeCorridor + 1);

    long nv;// 当前多面体顶点数
    PolyhedronH curIH;// overlap 多面体的 H-rep（半空间）
    PolyhedronV curIV, curIOB;// 当前顶点 V-rep（vertex）// 顶点帧偏移表达
    opt_vars.waypoint_attractor.resize(3, sizeCorridor); // attractor（每段一个3D点）和对应的容差（半径）
    opt_vars.hOverlapPolytopes.resize(sizeCorridor);
    opt_vars.waypoint_attractor_dead_d.resize(sizeCorridor);
    // * 2) Process the corridor
    // 2) 遍历 corridor 段
    for (int i = 0; i < sizeCorridor; i++) {
        // * 2.1) Get current vertex  // 2.1) 将第 i 个多面体转换为顶点表达
        if (!geometry_utils::enumerateVs(opt_vars.hPolytopes[i], curIV)) {
            cout << YELLOW << " -- [SUPER] in [ GcopterExpS4::processCorridor]: Failed to enumerate corridor Vs."
                 << RESET << endl;

            return false;
        }
        // * 2.3) Conver the vertex to the frame of the first point // 2.3) 将顶点转换为相对坐标：首点不动，其余点减去首点
        nv = curIV.cols();
        curIOB.resize(3, nv);
        // *    Save the position of the first point
        curIOB.col(0) = curIV.col(0);
        // *    Use the relative position of the rest vertex.
        curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
        // *    save the i-th corridor's vertex
        opt_vars.vPolytopes.push_back(curIOB);

        // * 2.4) Find the overlap corridor // 2.4) 计算第 i 和 i+1 个多面体的交集（overlap polytope）
        curIH.resize(opt_vars.hPolytopes[i].rows() + opt_vars.hPolytopes[i + 1].rows(), 4);
        curIH.topRows(opt_vars.hPolytopes[i].rows()) = opt_vars.hPolytopes[i];
        curIH.bottomRows(opt_vars.hPolytopes[i + 1].rows()) = opt_vars.hPolytopes[i + 1];
        opt_vars.hOverlapPolytopes[i] = curIH;
        Vec3f interior;
         // 查找 overlap polytope 的一个内部点并记录其“到边界最远的距离（dis）”
        const double dis = geometry_utils::findInteriorDist(curIH, interior) / 2;
        if (dis < 0.0 || std::isinf(dis)) {

            cout << YELLOW << " -- [SUPER] in [ GcopterExpS4::processCorridor]: Failed findInteriorDist Vs." <<
                 RESET << endl;
            return false;
        }
        geometry_utils::enumerateVs(curIH, interior, curIV);
        const double test_sum = curIV.sum();
        if (std::isnan(test_sum) || std::isinf(test_sum)) {
            return false;
        }
        // 存储该吸引点与其安全容差（半径）
        opt_vars.waypoint_attractor.col(i) = interior;
        opt_vars.waypoint_attractor_dead_d(i) = dis;
        nv = curIV.cols();
        curIOB.resize(3, nv);
        curIOB.col(0) = curIV.col(0);
        curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
        opt_vars.vPolytopes.push_back(curIOB);
    }

    // * 3) Time and waypoint allocation for hot initialization
      // 3) 用引导轨迹初始化每段点位置 & 时间戳（用于轨迹优化初值）
    VecDf min_dis(opt_vars.waypoint_attractor.cols());
    VecDi min_id(opt_vars.waypoint_attractor.cols());
    VecDf time_stamps(opt_vars.waypoint_attractor.cols() + 2);
    time_stamps(0) = 0.0;
    time_stamps(opt_vars.waypoint_attractor.cols() + 1) = opt_vars.guide_t.back();
    min_id.setConstant(0);
    min_dis.setConstant(std::numeric_limits<double>::max());
    // 遍历所有引导点，对每个吸引点找最近的 guide_path 点，并记录时间
    for (int i = 0; i < opt_vars.guide_path.size(); i++) {
        for (int j = 0; j < opt_vars.waypoint_attractor.cols(); j++) {
            const double dis = (opt_vars.guide_path[i] - opt_vars.waypoint_attractor.col(j)).norm();
            if (dis < min_dis[j]) {
                min_dis[j] = dis;
                min_id[j] = i;
                opt_vars.points.col(j) = opt_vars.waypoint_attractor.col(j);//opt_vars.guide_path[i];
                time_stamps(j + 1) = opt_vars.guide_t[i];
            }
        }
    }
     // 生成每段时间差（delta_t）。guide 时间戳提供整体节奏，空间距离提供每段下限，
     // 避免多个 corridor 吸引点映射到同一个 guide 采样点后产生过短热启动小段。
    for (int i = 1; i < time_stamps.size(); i++) {
        Vec3f from;
        Vec3f to;
        if (i == 1) {
            from = opt_vars.headPVAJ.col(0);
        } else {
            from = opt_vars.points.col(i - 2);
        }
        if (i == time_stamps.size() - 1) {
            to = opt_vars.tailPVAJ.col(0);
        } else {
            to = opt_vars.points.col(i - 1);
        }
        const double guide_dt = time_stamps(i) - time_stamps(i - 1);
        const double distance_dt = (to - from).norm() / std::max(cfg_.max_vel, 1.0e-3);
        opt_vars.times(i - 1) = std::max(kMinStablePieceTime, std::max(guide_dt, distance_dt));
    }

    if (!geometry_utils::enumerateVs(opt_vars.hPolytopes.back(), curIV)) {
        return false;
    }
    nv = curIV.cols();
    curIOB.resize(3, nv);
    curIOB.col(0) = curIV.col(0);
    curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
    opt_vars.vPolytopes.push_back(curIOB);
    return true;
}

void ExpTrajOpt::defaultInitialization() {
    const VecDf dis = (opt_vars.init_path.leftCols(opt_vars.piece_num) -
                       opt_vars.init_path.rightCols(opt_vars.piece_num)).colwise().norm();
    const double speed = cfg_.max_vel;
    opt_vars.times = dis / speed;
    opt_vars.points = opt_vars.waypoint_attractor;
}

bool ExpTrajOpt::setupProblemAndCheck() {
    // init internal variables size;
    opt_vars.piece_num = static_cast<int>(opt_vars.hPolytopes.size());
    opt_vars.times.resize(opt_vars.piece_num);
    opt_vars.points.resize(3, opt_vars.piece_num - 1);


    // Check corridor and init points
    if (opt_vars.default_init) {
        throw std::runtime_error("Not support default init in this version.");
        if (!processCorridor()) {
            return false;
        }
    } else {
        if (!processCorridorWithGuideTraj()) {
            return false;
        }
    }
    opt_vars.init_path.resize(3, opt_vars.piece_num + 1);
    for (long i = 0; i < opt_vars.piece_num - 1; i++) {
        opt_vars.init_path.col(i + 1) = opt_vars.waypoint_attractor.col(i);
    }
    opt_vars.init_path.col(0) = opt_vars.headPVAJ.col(0);
    opt_vars.init_path.rightCols(1) = opt_vars.tailPVAJ.col(0);
    if (opt_vars.default_init) {
        defaultInitialization();
    } else {
        opt_vars.times = opt_vars.times.cwiseMax(kMinStablePieceTime);
    }

    if (!opt_vars.times.allFinite() || opt_vars.times.minCoeff() < kMinStablePieceTime) {
        cout << YELLOW << " -- [ExpOpt] Init times and point failed." << RESET << endl;
        return false;
    }

    const Mat3Df deltas = opt_vars.init_path.rightCols(opt_vars.piece_num)
                          - opt_vars.init_path.leftCols(opt_vars.piece_num);
    opt_vars.pieceIdx = (deltas.colwise().norm() / INFINITY).cast<int>().transpose();
    opt_vars.pieceIdx.array() += 1;

    opt_vars.temporalDim = opt_vars.piece_num;
    opt_vars.spatialDim = 0;
    opt_vars.vPolyIdx.resize(opt_vars.piece_num - 1);
    opt_vars.hPolyIdx.resize(opt_vars.piece_num);

    switch (cfg_.pos_constraint_type) {
        case 1: {
            for (int i = 0, j = 0, k; i < opt_vars.piece_num; i++) {
                k = opt_vars.pieceIdx(i);
                for (int l = 0; l < k; l++, j++) {
                    if (l < k - 1) {
                        opt_vars.vPolyIdx(j) = 2 * i;
                    } else if (i < opt_vars.piece_num - 1) {
                        opt_vars.vPolyIdx(j) = 2 * i + 1;
                    }
                    opt_vars.hPolyIdx(j) = i;
                }
            }
            opt_vars.spatialDim = 3 * (opt_vars.piece_num - 1);
            break;
        }
        default: {
            for (int i = 0, j = 0, k; i < opt_vars.piece_num; i++) {
                k = opt_vars.pieceIdx(i);
                for (int l = 0; l < k; l++, j++) {
                    if (l < k - 1) {
                        opt_vars.vPolyIdx(j) = 2 * i;
                        opt_vars.spatialDim += static_cast<int>(opt_vars.vPolytopes[2 * i].cols());
                    } else if (i < opt_vars.piece_num - 1) {
                        opt_vars.vPolyIdx(j) = 2 * i + 1;
                        opt_vars.spatialDim += static_cast<int>(opt_vars.vPolytopes[2 * i + 1].cols());
                    }
                    opt_vars.hPolyIdx(j) = i;
                }
            }
        }
    }

    opt_vars.minco.setConditions(opt_vars.headPVAJ, opt_vars.tailPVAJ, opt_vars.piece_num);
    opt_vars.gradByPoints.resize(3, opt_vars.piece_num - 1);
    opt_vars.gradByTimes.resize(opt_vars.piece_num);
    opt_vars.partialGradByCoeffs.resize(8 * opt_vars.piece_num, 3);
    opt_vars.partialGradByTimes.resize(opt_vars.piece_num);
    return true;
}

bool ExpTrajOpt::setInitPsAndTs(const vec_Vec3f &init_ps, const vector<double> &init_ts) {
    opt_vars.default_init = false;
    if (opt_vars.times.size() != init_ts.size()) {
        return false;
    }
    if (opt_vars.points.cols() != init_ps.size()) {
        return false;
    }

    for (long i = 0; i < opt_vars.points.cols(); i++) {
        opt_vars.times[i] = init_ts[i];
        opt_vars.points.col(i) = init_ps[i];
    }
    opt_vars.times[opt_vars.times.size() - 1] = init_ts.back();
    return true;
}
//在多面体约束下，用 L-BFGS 对轨迹的时间分配和空间点位置进行联合优化，使轨迹平滑并符合动态约束（加速度、角速度、推力、吸引点等）。
//输出轨迹结果存入 traj 对象。
double ExpTrajOpt::optimize(Trajectory &traj, const double &relCostTol) {
    /* 1) allocate vector for optimization varibles */
    //① 准备优化变量
    VecDf x(opt_vars.temporalDim + opt_vars.spatialDim);
    /*    creat map for the opt_var vector */
    Eigen::Map<VecDf> tau(x.data(), opt_vars.temporalDim);
    Eigen::Map<VecDf> xi(x.data() + opt_vars.temporalDim, opt_vars.spatialDim);

    opt_vars.penalty_log.resize(8);
    opt_vars.penalty_log.setZero();
    //如果有用户提供的初始时间和空间点，就直接用它们。（exp用guid path，不用这个）
    if (opt_vars.given_init_ts_and_ps) {
        opt_vars.times = opt_vars.init_ts;
        for (int i = 0; i < opt_vars.init_ps.size(); i++) {
            opt_vars.points.col(i) = opt_vars.init_ps[i];
        }
    }
    //② 初始化轨迹时间和点
    /* 2) check the initial value of the optimization varibles */
    if (!opt_vars.headPVAJ.allFinite() || !opt_vars.tailPVAJ.allFinite() ||
        !opt_vars.times.allFinite() || opt_vars.times.minCoeff() < kMinStablePieceTime ||
        !opt_vars.points.allFinite()) {
        cout << YELLOW << " -- [TrajOpt] Error, invalid init state/time/points, force return." << RESET << endl;
        cout << " -- Head PVAJ: " << endl;
        cout << opt_vars.headPVAJ << endl;
        cout << " -- Tail PVAJ: " << endl;
        cout << opt_vars.tailPVAJ << endl;
        cout << " -- Times: " << endl;
        cout << opt_vars.times.transpose() << endl;
        return INFINITY;
    }

    /* 3)  construct the initial guess of the optimization varibles*/
    //③ 初始化优化变量（初值）
    backwardMapBoundedTToTau(opt_vars.times, tau);
    switch (opt_vars.pos_constraint_type) {
        case 1: {
            MatDf p_e = opt_vars.points;
            xi = Eigen::Map<const VecDf>(p_e.data(), p_e.size());
            break;
        }
        default: {
            gcopter::backwardP(opt_vars.points, opt_vars.vPolyIdx, opt_vars.vPolytopes, xi);
            break;
        }
    }

    /* 4) setup the optimizer's parameters*/
    //④ 设置 L-BFGS 参数
    opt_vars.iter_num = 0;
    double minCostFunctional{0};
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = 256;
    lbfgs_params.past = 3;
    lbfgs_params.min_step = 1.0e-32;
    lbfgs_params.g_epsilon = 0.0;//表示禁用梯度停止条件
    lbfgs_params.delta = relCostTol; //控制相对变化小于这个值则终止；//opt_accuracy: 5.0e-6  #//相对优化终止容差。
    VecDf times_init = opt_vars.times;

    opt_vars.init_ts = opt_vars.times;
    opt_vars.init_ps.clear();
    for (int col = 0; col < opt_vars.points.cols(); col++) {
        opt_vars.init_ps.emplace_back(opt_vars.points.col(col));
    }

    // keep fixed accuracy for
    //把 attractor（吸引点）值用 truncateToSixDecimals 限制精度，提高数值稳定性。
    for (int i = 0; i < opt_vars.waypoint_attractor_dead_d.size(); i++) {
        truncateToSixDecimals(opt_vars.waypoint_attractor_dead_d(i));
        truncateToSixDecimals(opt_vars.waypoint_attractor(0, i));
        truncateToSixDecimals(opt_vars.waypoint_attractor(1, i));
        truncateToSixDecimals(opt_vars.waypoint_attractor(2, i));
    }

    cout << std::fixed << std::setprecision(15);
    auto x0 = x;
    // only for debug
//    cout << " -- [ExpOpt] Start optimization." << x.transpose() << endl;
//    cout << " -- [ExpOpt] minCostFunctional: " << minCostFunctional << endl;
//    cout << " -- [ExpOpt] relCostTol: " << relCostTol << endl;
//    cout << " -- [ExpOpt] weightAtt: " << opt_vars.penaltyWeights(4) << endl;
//    cout << " -- [ExpOpt] waypoint_attractor: " << opt_vars.waypoint_attractor << endl;
//    cout << " -- [ExpOpt] waypoint_attractor_dead_d: " << opt_vars.waypoint_attractor_dead_d.transpose() << endl;
    // TimeConsuming ttt(" -- [ExpTrajOpt]", false);
    opt_vars.iter_num = 0;
    //⑥ 执行 L-BFGS 优化
    int ret = lbfgs::lbfgs_optimize(x,
                                    minCostFunctional,
                                    &ExpTrajOpt::costFunctional,
                                    nullptr,
                                    nullptr,
                                    &this->opt_vars,
                                    lbfgs_params);
    // double dt = ttt.stop();
    //⑦ 解包优化结果
    forwardMapTauToBoundedT(tau, opt_vars.times);
    if (cfg_.print_optimizer_log) {
        cout << " -- [ExpOpt] Opt finish, with iter num: " << opt_vars.iter_num << "\n";
        cout << "\tEnergy: " << opt_vars.penalty_log(0) << endl;
        cout << "\tPos: " << opt_vars.penalty_log(1) << endl;
        cout << "\tVel: " << opt_vars.penalty_log(2) << endl;
        cout << "\tAcc: " << opt_vars.penalty_log(3) << endl;
        cout << "\tJerk: " << opt_vars.penalty_log(4) << endl;
        cout << "\tAttract: " << opt_vars.penalty_log(5) << endl;
        cout << "\tOmg: " << opt_vars.penalty_log(6) << endl;
        cout << "\tThr: " << opt_vars.penalty_log(7) << endl;
        cout << "\tOptimized Time: " << opt_vars.times.transpose() << endl;
    }
    //检查加速度、角速度、推力、位置等是否超出限制。
    //若违反约束，虽然优化过程成功，逻辑上也认为失败。
    const double acc_violation_tol = squaredBoundViolationTolerance(cfg_.max_acc, cfg_.penna_margin);
    const double omg_violation_tol = squaredBoundViolationTolerance(cfg_.max_omg, cfg_.penna_margin);
    const double thr_violation_tol = thrustViolationTolerance(cfg_);
    if ((cfg_.penna_pos > 0 && opt_vars.penalty_log(1) > 0.2) ||
        // (cfg_.penna_vel > 0 && opt_vars.penalty_log(2) > cfg_.max_vel * cfg_.penna_margin) ||
        (cfg_.penna_acc > 0 && opt_vars.penalty_log(3) > acc_violation_tol) ||
        (cfg_.penna_omg > 0 && opt_vars.penalty_log(6) > omg_violation_tol) ||
        (cfg_.penna_thr > 0 && opt_vars.penalty_log(7) > thr_violation_tol)) {
        if (cfg_.print_optimizer_log) {
            cout << " -- [ExpOpt] Opt finish, with iter num: " << opt_vars.iter_num << "\n";
            cout << "\tEnergy: " << opt_vars.penalty_log(0) << endl;
            cout << "\tPos: " << opt_vars.penalty_log(1) << endl;
            cout << "\tVel: " << opt_vars.penalty_log(2) << endl;
            cout << "\tAcc: " << opt_vars.penalty_log(3) << endl;
            cout << "\tJerk: " << opt_vars.penalty_log(4) << endl;
            cout << "\tAttract: " << opt_vars.penalty_log(5) << endl;
            cout << "\tOmg: " << opt_vars.penalty_log(6) << endl;
            cout << "\tThr: " << opt_vars.penalty_log(7) << endl;
            cout << "\tOptimized Time: " << opt_vars.times.transpose() << endl;
        }
        ros_ptr_->warn(" -- [ExpOpt] Opt failed, Pos/Acc/Omg/Thr violation. "
                       "pos={:.6f}, acc={:.6f}/{:.6f}, omg={:.6f}/{:.6f}, "
                       "thr={:.6f}/{:.6f}, time_sum={:.6f}",
                       opt_vars.penalty_log(1),
                       opt_vars.penalty_log(3), acc_violation_tol,
                       opt_vars.penalty_log(6), omg_violation_tol,
                       opt_vars.penalty_log(7), thr_violation_tol,
                       opt_vars.times.sum());
        ret = -1;
    }

    if (ret >= 0) {
        forwardMapTauToBoundedT(tau, opt_vars.times);
        switch (opt_vars.pos_constraint_type) {
            case 1: {
                VecDf xi_e = xi;
                opt_vars.points = Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>>(xi_e.data(), 3, xi_e.size() / 3);
                break;
            }
            default: {
                gcopter::forwardP(xi, opt_vars.vPolyIdx,
                                  opt_vars.vPolytopes, opt_vars.points);
                break;
            }
        }
//        opt_vars.minco.setConditions(opt_vars.headPVAJ, opt_vars.tailPVAJ, opt_vars.temporalDim);
        opt_vars.minco.setParameters(opt_vars.points, opt_vars.times);
        opt_vars.minco.getTrajectory(traj);
    } else {
        traj.clear();
        minCostFunctional = INFINITY;
        cout << YELLOW << " -- [MINCO] TrajOpt failed, " << lbfgs::lbfgs_strerror(ret) << RESET << endl;
//        cout << "Init times: " << times_init.transpose() << endl;
    }
    return minCostFunctional + ret;
}

ExpTrajOpt::ExpTrajOpt(const traj_opt::Config &cfg, const ros_interface::RosInterface::Ptr &ros_ptr) :
        cfg_(cfg),
        ros_ptr_(ros_ptr) {
    /// Use time as log file name
    //    auto now = std::chrono::system_clock::now();
    //    std::time_t t = std::chrono::system_clock::to_time_t(now);
    //    std::tm tm = *std::localtime(&t);
    //    std::stringstream ss;
    //    ss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    //    std::string filename = ss.str() + "_exp_opt_log.csv";
    if(cfg_.save_log_en){
        std::string filename = "exp_opt_log.csv";
        failed_traj_log.open(DEBUG_FILE_DIR(filename), std::ios::out | std::ios::trunc);
        penalty_log.open(DEBUG_FILE_DIR("exp_opt_penna.csv"), std::ios::out | std::ios::trunc);
    }

    opt_vars.magnitudeBounds.resize(6);
    opt_vars.penaltyWeights.resize(7);
    opt_vars.magnitudeBounds << cfg_.max_vel, cfg_.max_acc, cfg_.max_jerk,
            cfg_.max_omg, cfg_.min_acc_thr * cfg_.mass, cfg_.max_acc_thr * cfg_.mass;
    opt_vars.penaltyWeights << cfg_.penna_pos, cfg_.penna_vel,
            cfg_.penna_acc, cfg_.penna_jerk,
            cfg_.penna_attract, cfg_.penna_omg,
            cfg_.penna_thr;
    opt_vars.rho = cfg_.penna_t;
    opt_vars.pos_constraint_type = cfg_.pos_constraint_type;
    opt_vars.block_energy_cost = cfg_.block_energy_cost;
    opt_vars.smooth_eps = cfg_.smooth_eps;
    opt_vars.integral_res = cfg_.integral_reso;
    opt_vars.quadrotor_flatness = cfg_.quadrotot_flatness;
    opt_vars.local_density_en = cfg_.local_density_en;
    opt_vars.local_density_radius_x = cfg_.local_density_radius_x;
    opt_vars.local_density_radius_y = cfg_.local_density_radius_y;
    opt_vars.local_density_radius_z = cfg_.local_density_radius_z;
    opt_vars.local_density_smooth_eps = cfg_.local_density_smooth_eps;
    opt_vars.local_density_time_inflation = cfg_.local_density_time_inflation;
    opt_vars.local_density_weight = cfg_.local_density_weight;
}

ExpTrajOpt::~ExpTrajOpt() {
    failed_traj_log.close();
    penalty_log.close();
}


//bool ExpTrajOpt::optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
//                          PolytopeVec &sfcs,
//                          Trajectory &out_traj) {
//    /// Check if SFC is valid
//    if (sfcs.empty()) {
//        cout << YELLOW << " -- [TrajOpt] Error, the SFC is empty." << RESET << endl;
//        return false;
//    }
//
//    if (!SimplifySFC(headPVAJ.col(0), tailPVAJ.col(0), sfcs)) {
//        cout << YELLOW << " -- [TrajOpt] Cannot simplify sfcs." << RESET << endl;
//        //        VisualUtils::VisualizePoint(mkr_pub_, headPVAJ.col(0),Color::Pink(),"ill_start",0.5,1);
//        //        VisualUtils::VisualizePoint(mkr_pub_, tailPVAJ.col(0),Color::Pink(),"ill_end",0.5,2);
//        //        cout << "headPVAJ: " << headPVAJ.col(0).transpose() << endl;
//        //        cout << "tailPVAJ: " << tailPVAJ.col(0).transpose() << endl;
//        //        cout << YELLOW << "Killing the node." << RESET << endl;
//        //        exit(-1);
//        return false;
//    }
//
//    for (const auto &poly: sfcs) {
//        if (std::isnan(poly.GetPlanes().sum())) {
//            cout << YELLOW << " -- [TrajOpt] Error, the SFC containes NaN." << RESET << endl;
//            return false;
//        }
//    }
//
//    bool success{true};
//
//    /// Setup optimization problems
//    opt_vars.default_init = true;
//    opt_vars.given_init_ts_and_ps = false;
//    opt_vars.headPVAJ = headPVAJ;
//    opt_vars.tailPVAJ = tailPVAJ;
//    opt_vars.guide_path.clear();
//    opt_vars.guide_t.clear();
//    opt_vars.hPolytopes.resize(sfcs.size());
//    for (long i = 0; i < sfcs.size(); i++) {
//        opt_vars.hPolytopes[i] = sfcs[i].GetPlanes();
//    }
//
//    if (!setupProblemAndCheck()) {
//        cout << YELLOW << " -- [SUPER] Minco corridor preprocess error." << RESET << endl;
//        success = false;
//    }
//
//    if (success && std::isinf(optimize(out_traj, cfg_.opt_accuracy))) {
//        std::cout << YELLOW << " -- [SUPER] in [ExpTrajOpt::optimize]: Optimization failed." << RESET << std::endl;
//        success = false;
//    }
//
//    if(success){
//        out_traj.start_WT = ros_ptr_->getSimTime();
//    }
//
//    if (!success && cfg_.save_log_en) {
//        failed_traj_log << 990419 << endl;
//        failed_traj_log << headPVAJ << endl;
//        failed_traj_log << tailPVAJ << endl;
//        for (long i = 0; i < sfcs.size(); i++) {
//            failed_traj_log << i << endl;
//            failed_traj_log << sfcs[i].GetPlanes() << endl;
//        }
//    }
//
//    return success;
//}

bool ExpTrajOpt::optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                          const vec_E<Vec3f> &guide_path, const vector<double> &guide_t,
                          PolytopeVec &sfcs,
                          Trajectory &out_traj) {
    static const std::vector<MatD4f> empty_dynamic_hplanes;
    return optimize(headPVAJ, tailPVAJ, guide_path, guide_t, sfcs, empty_dynamic_hplanes, out_traj);
}

bool ExpTrajOpt::optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                          const vec_E<Vec3f> &guide_path, const vector<double> &guide_t,
                          PolytopeVec &sfcs,
                          const std::vector<MatD4f> &dynamic_hplanes,
                          Trajectory &out_traj) {
    static const vector<SwarmPrediction> empty_swarm_predictions;
    return optimize(headPVAJ, tailPVAJ, guide_path, guide_t, sfcs, dynamic_hplanes,
                    empty_swarm_predictions, ros_ptr_->getSimTime(), out_traj);
}

bool ExpTrajOpt::optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                          const vec_E<Vec3f> &guide_path, const vector<double> &guide_t,
                          PolytopeVec &sfcs,
                          const std::vector<MatD4f> &dynamic_hplanes,
                          const vector<SwarmPrediction> &swarm_predictions,
                          const double trajectory_start_wt,
                          Trajectory &out_traj) {
    /// Check if hot init is valid 检查输入是否合法
    //引导路径的点数要和时间戳长度一致，否则无法确定轨迹采样点。
    if (!headPVAJ.allFinite() || !tailPVAJ.allFinite() || !finiteGuide(guide_path, guide_t)) {
        cout << YELLOW << " -- [TrajOpt] Error, invalid state or guide trajectory." << RESET << endl;
        return false;
    }
    /// Check if SFC is valid
    if (sfcs.empty()) {
        cout << YELLOW << " -- [TrajOpt] Error, the SFC is empty." << RESET << endl;
        return false;
    }

    PolytopeVec opt_sfcs = sfcs;
    if (!dynamic_hplanes.empty()) {
        for (long i = 0; i < static_cast<long>(opt_sfcs.size()); ++i) {
            if (i >= static_cast<long>(dynamic_hplanes.size())) {
                continue;
            }
            MatD4f merged_planes;
            if (!mergeFiniteDynamicPlanes(opt_sfcs[i].GetPlanes(), dynamic_hplanes[i], merged_planes)) {
                continue;
            }
            Vec3f interior;
            if (geometry_utils::findInterior(merged_planes, interior)) {
                opt_sfcs[i].SetPlanes(merged_planes);
            }
        }
    }

    if (!SimplifySFC(headPVAJ.col(0), tailPVAJ.col(0), opt_sfcs)) {//简化 SFC（可选压缩、合并等）
        cout << YELLOW << " -- [TrajOpt] Cannot simplify sfcs." << RESET << endl;
        return false;
    }
    sfcs = opt_sfcs;

    bool success{true};

    /// Setup optimization problems
    //设置优化变量
    opt_vars.default_init = false;
    opt_vars.given_init_ts_and_ps = false;
    opt_vars.headPVAJ = headPVAJ;
    opt_vars.tailPVAJ = tailPVAJ;
    opt_vars.guide_path = guide_path;
    opt_vars.guide_t = guide_t;
    opt_vars.swarm_predictions = swarm_predictions;
    opt_vars.trajectory_start_wt = trajectory_start_wt;
    opt_vars.hPolytopes.resize(sfcs.size());
    //准备多面体（H-Polytope）约束

    for (long i = 0; i < sfcs.size(); i++) {
        opt_vars.hPolytopes[i] = sfcs[i].GetPlanes();
        if (opt_vars.hPolytopes[i].cols() != 4 || opt_vars.hPolytopes[i].rows() <= 0 ||
            !opt_vars.hPolytopes[i].allFinite()) {
            cout << YELLOW << " -- [TrajOpt] Error, invalid SFC planes." << RESET << endl;
            return false;
        }
        const Eigen::ArrayXd norms = opt_vars.hPolytopes[i].leftCols<3>().rowwise().norm();
        if ((norms <= 1.0e-6).any() || (!norms.isFinite()).any()) {
            cout << YELLOW << " -- [TrajOpt] Error, degenerate SFC plane normal." << RESET << endl;
            return false;
        }
        opt_vars.hPolytopes[i].array().colwise() /= norms;
    }

    if (!setupProblemAndCheck()) { //设置优化问题并检查
        cout << YELLOW << " -- [SUPER] Minco corridor preprocess error." << RESET << endl;
        success = false;
    }

    out_traj.clear();

    //调用轨迹优化求解器
    double opt_cost = success ? optimize(out_traj, cfg_.opt_accuracy) : INFINITY;
    if (success && !std::isfinite(opt_cost)) {
        const VecDf base_init_ts = opt_vars.init_ts;
        const vec_Vec3f base_init_ps = opt_vars.init_ps;
        const VecDf base_penalty_weights = opt_vars.penaltyWeights;
        const double base_rho = opt_vars.rho;
        const bool base_given_init = opt_vars.given_init_ts_and_ps;

        const std::array<std::tuple<double, double, double>, 2> retry_settings = {
                std::make_tuple(1.20, 3.0, 0.80),
                std::make_tuple(1.50, 6.0, 0.60)
        };
        for (const auto &[time_scale, constraint_scale, rho_scale]: retry_settings) {
            if (base_init_ts.size() != opt_vars.piece_num ||
                static_cast<int>(base_init_ps.size()) != opt_vars.piece_num - 1) {
                break;
            }
            opt_vars.given_init_ts_and_ps = true;
            opt_vars.init_ts = (base_init_ts * time_scale).cwiseMax(kMinStablePieceTime);
            opt_vars.init_ps = base_init_ps;
            opt_vars.penaltyWeights = base_penalty_weights;
            opt_vars.penaltyWeights(0) *= constraint_scale;
            opt_vars.penaltyWeights(5) *= constraint_scale;
            opt_vars.rho = base_rho * rho_scale;
            out_traj.clear();
            ros_ptr_->warn(" -- [ExpOpt] Retry with init_time_scale={:.2f}, constraint_scale={:.2f}, rho_scale={:.2f}",
                           time_scale, constraint_scale, rho_scale);
            opt_cost = optimize(out_traj, cfg_.opt_accuracy);
            if (std::isfinite(opt_cost)) {
                break;
            }
        }
        opt_vars.penaltyWeights = base_penalty_weights;
        opt_vars.rho = base_rho;
        opt_vars.given_init_ts_and_ps = base_given_init;
    }
    if (success && !std::isfinite(opt_cost)) {
        cout << YELLOW << " -- [SUPER] Minco exp_traj opt failed." << RESET << endl;
        success = false;
    }

    penalty_log << opt_vars.penalty_log.transpose() << endl;

    if (success) {
        out_traj.start_WT = ros_ptr_->getSimTime();//设置输出轨迹的起始时间戳
    }

    if (!success && cfg_.save_log_en) {
        failed_traj_log << 990419 << endl;
        failed_traj_log << headPVAJ << endl;
        failed_traj_log << tailPVAJ << endl;
        for (double i: guide_t) {
            failed_traj_log << i << " ";
        }
        failed_traj_log << endl;
        for (const auto &i: guide_path) {
            failed_traj_log << i.transpose() << " ";
        }
        failed_traj_log << endl;
        for (long i = 0; i < sfcs.size(); i++) {
            failed_traj_log << i << endl;
            failed_traj_log << sfcs[i].GetPlanes() << endl;
        }
    }
    return success;
}

bool ExpTrajOpt::optimize(const StatePVAJ &headPVAJ, const StatePVAJ &tailPVAJ,
                          PolytopeVec &sfcs,
                          const vec_Vec3f &init_ps,
                          const VecDf &init_ts,
                          Trajectory &out_traj) {
    vec_Vec3f guide_path;
    guide_path.emplace_back(headPVAJ.col(0));
    for (const auto &i: init_ps) {
        guide_path.emplace_back(i);
    }
    guide_path.emplace_back(tailPVAJ.col(0));
    vector<double> guide_t;
    guide_t.emplace_back(0);
    double accumulate_t = 0;
    for (int i = 0; i < init_ts.size(); i++) {
        accumulate_t += init_ts[i];
        guide_t.emplace_back(accumulate_t);
    }
    /// Check if hot init is valid
    if (!headPVAJ.allFinite() || !tailPVAJ.allFinite() || !init_ts.allFinite() || !finiteGuide(guide_path, guide_t)) {
        cout << YELLOW << " -- [TrajOpt] Error, invalid state or guide trajectory." << RESET << endl;
        return false;
    }
    /// Check if SFC is valid
    if (sfcs.empty()) {
        cout << YELLOW << " -- [TrajOpt] Error, the SFC is empty." << RESET << endl;
        return false;
    }

    if (!SimplifySFC(headPVAJ.col(0), tailPVAJ.col(0), sfcs)) {
        cout << YELLOW << " -- [TrajOpt] Cannot simplify sfcs." << RESET << endl;
        return false;
    }

    bool success{true};

    /// Setup optimization problems
    opt_vars.default_init = false;
    opt_vars.given_init_ts_and_ps = true;
    opt_vars.init_ts = init_ts;
    opt_vars.init_ps = init_ps;
    opt_vars.headPVAJ = headPVAJ;
    opt_vars.tailPVAJ = tailPVAJ;
    opt_vars.guide_path = guide_path;
    opt_vars.guide_t = guide_t;
    opt_vars.swarm_predictions.clear();
    opt_vars.trajectory_start_wt = ros_ptr_->getSimTime();
    opt_vars.hPolytopes.resize(sfcs.size());

    for (long i = 0; i < sfcs.size(); i++) {
        opt_vars.hPolytopes[i] = sfcs[i].GetPlanes();
        if (opt_vars.hPolytopes[i].cols() != 4 || opt_vars.hPolytopes[i].rows() <= 0 ||
            !opt_vars.hPolytopes[i].allFinite()) {
            cout << YELLOW << " -- [TrajOpt] Error, invalid SFC planes." << RESET << endl;
            return false;
        }
        const Eigen::ArrayXd norms = opt_vars.hPolytopes[i].leftCols<3>().rowwise().norm();
        if ((norms <= 1.0e-6).any() || (!norms.isFinite()).any()) {
            cout << YELLOW << " -- [TrajOpt] Error, degenerate SFC plane normal." << RESET << endl;
            return false;
        }
        opt_vars.hPolytopes[i].array().colwise() /= norms;
    }

    if (!setupProblemAndCheck()) {
        cout << YELLOW << " -- [SUPER] Minco corridor preprocess error." << RESET << endl;
        success = false;
    }

    out_traj.clear();

    const double opt_cost = success ? optimize(out_traj, cfg_.opt_accuracy) : INFINITY;
    if (success && !std::isfinite(opt_cost)) {
        cout << YELLOW << " -- [SUPER] Minco exp_traj opt failed." << RESET << endl;
        success = false;
    }
    penalty_log << opt_vars.penalty_log.transpose() << endl;

    if (success) {
        out_traj.start_WT = ros_ptr_->getSimTime();
    }


    if (!success && cfg_.save_log_en) {
        failed_traj_log << 990419 << endl;
        failed_traj_log << headPVAJ << endl;
        failed_traj_log << tailPVAJ << endl;
        for (double i: guide_t) {
            failed_traj_log << i << " ";
        }
        failed_traj_log << endl;
        for (const auto &i: guide_path) {
            failed_traj_log << i.transpose() << " ";
        }
        failed_traj_log << endl;
        for (long i = 0; i < sfcs.size(); i++) {
            failed_traj_log << i << endl;
            failed_traj_log << sfcs[i].GetPlanes() << endl;
        }
    }
    return success;
}
