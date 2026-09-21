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

#include <traj_opt/backup_traj_optimizer_s4.h>
#include <utils/header/color_msg_utils.hpp>

#include <array>

using namespace traj_opt;
using namespace color_text;
using namespace super_utils;

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

    void forwardMapTauToBoundedT(const Eigen::VectorXd &tau, Eigen::VectorXd &times) {
        gcopter::forwardMapTauToT(tau, times);
        times.array() += kMinStablePieceTime;
    }

    template<typename EIGENVEC>
    void backwardMapBoundedTToTau(const Eigen::VectorXd &times, EIGENVEC &tau) {
        const Eigen::VectorXd positive_times =
                (times.array() - kMinStablePieceTime).max(kMinMappedPieceTime).matrix();
        gcopter::backwardMapTToTau(positive_times, tau);
    }

    double sigmoid(const double value) {
        const double e = std::exp(value >= 0.0 ? -value : value);
        return value >= 0.0 ? 1.0 / (1.0 + e) : e / (1.0 + e);
    }

    void evaluateLocalDensity(const Vec3f &self_pos, const double query_wt,
                              const vector<SwarmPrediction> &predictions,
                              const double radius_x, const double radius_y, const double radius_z,
                              const double smooth_eps, const double time_inflation,
                              double &neighbor_count, Vec3f &grad_pos, double &grad_time) {
        neighbor_count = 0.0;
        grad_pos.setZero();
        grad_time = 0.0;
        const double inflation = std::max(0.0, time_inflation);
        const std::array<double, 3> offsets{{-inflation, 0.0, inflation}};
        const std::array<double, 3> weights = inflation > 1.0e-6
                                              ? std::array<double, 3>{{0.25, 0.5, 0.25}}
                                              : std::array<double, 3>{{0.0, 1.0, 0.0}};
        const double eps = std::max(1.0e-3, smooth_eps);
        const Vec3f radii(std::max(1.0e-3, radius_x), std::max(1.0e-3, radius_y),
                          std::max(1.0e-3, radius_z));
        for (const auto &prediction: predictions) {
            double count = 0.0, valid_weight = 0.0, time_grad = 0.0;
            Vec3f pos_grad = Vec3f::Zero();
            for (size_t k = 0; k < offsets.size(); ++k) {
                if (weights[k] <= 0.0) continue;
                Vec3f other_pos, other_vel;
                if (!prediction.sample(query_wt + offsets[k], other_pos, other_vel)) continue;
                const Vec3f delta = other_pos - self_pos;
                const double distance =
                        std::sqrt(delta.cwiseQuotient(radii).squaredNorm() + eps * eps);
                const double occupancy = sigmoid((1.0 - distance) / eps);
                const Vec3f occupancy_grad = occupancy * (1.0 - occupancy) *
                        delta.cwiseQuotient(radii.cwiseProduct(radii)) / (distance * eps);
                count += weights[k] * occupancy;
                pos_grad += weights[k] * occupancy_grad;
                time_grad += weights[k] * (-occupancy_grad.dot(other_vel));
                valid_weight += weights[k];
            }
            if (valid_weight <= 0.0) continue;
            neighbor_count += count / valid_weight;
            grad_pos += pos_grad / valid_weight;
            grad_time += time_grad / valid_weight;
        }
    }

    void evaluateStaticDensity(const Vec3f &position, const PolyhedronH &h_poly,
                               const double clearance_radius, const double smooth_eps,
                               double &density, Vec3f &grad_pos) {
        density = 0.0;
        grad_pos.setZero();
        if (h_poly.rows() == 0 || clearance_radius <= 0.0) return;

        const double eps = std::max(1.0e-3, smooth_eps);
        for (int row = 0; row < h_poly.rows(); ++row) {
            Vec3f normal = h_poly.block<1, 3>(row, 0);
            const double normal_norm = normal.norm();
            if (normal_norm <= 1.0e-6) continue;
            normal /= normal_norm;
            const double clearance = -(normal.dot(position) + h_poly(row, 3) / normal_norm);
            const double occupancy = sigmoid((clearance_radius - clearance) / eps);
            density += occupancy;
            grad_pos += occupancy * (1.0 - occupancy) * normal / eps;
        }
        density /= static_cast<double>(h_poly.rows());
        grad_pos /= static_cast<double>(h_poly.rows());
    }
}

//========================================================================
void BackupTrajOpt::constraintsFunctional(const Eigen::VectorXd &T,
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
                                          VecDf &pena_log) {
//    opt_vars.magnitudeBounds
//            << cfg_.max_vel, cfg_.max_acc, cfg_.max_jerk, cfg_.max_omg, cfg_.min_acc_thr, cfg_.max_acc_thr;
//    opt_vars.penaltyWeights << cfg_.penna_pos, cfg_.penna_vel,
//            cfg_.penna_acc, cfg_.penna_jerk,
//            cfg_.penna_attract, cfg_.penna_omg,
//            cfg_.penna_thr;
    const double vmax = magnitudeBounds[0];
    const double amax = magnitudeBounds[1];
    const double jmax = magnitudeBounds[2];
    const double omgmax = magnitudeBounds[3];
    const double accthrmin = magnitudeBounds[4];
    const double accthrmax = magnitudeBounds[5];

    const double vmaxSqr = vmax * vmax;
    const double amaxSqr = amax * amax;
    const double jmaxSqr = jmax * jmax;
    const double omgmaxSqr = omgmax * omgmax;

    const double thrustMean = 0.5 * (accthrmax + accthrmin);
    const double thrustRadi = 0.5 * std::abs(accthrmax - accthrmin);
    const double thrustSqrRadi = thrustRadi * thrustRadi;

    const double weightPos = penaltyWeights[0];
    const double weightVel = penaltyWeights[1];
    const double weightAcc = penaltyWeights[2];
    const double weightJer = penaltyWeights[3];
    // const double weightAtt = penaltyWeights[4];
    const double weightOmg = penaltyWeights[5];
    const double weightAccThr = penaltyWeights[6];


    Eigen::Vector3d pos, vel, acc, jer, sna;
    Eigen::Vector3d totalGradPos, totalGradVel, totalGradAcc, totalGradJer;
    double totalGradPsi, totalGradPsiD;
    double thr;
    Eigen::Vector4d quat;
    Eigen::Vector3d omg;
    double gradThr;
    Eigen::Vector3d gradPos, gradVel, gradAcc, gradJer, gradOmg;

    double step, alpha;
    double s1, s2, s3, s4, s5, s6, s7;
    Eigen::Matrix<double, 8, 1> beta0, beta1, beta2, beta3, beta4;
    Eigen::Vector3d outerNormal;

    double violaPos, violaVel, violaAcc, violaJer, violaOmg, violaThrust;
    double violaPosPenaD, violaVelPenaD, violaAccPenaD, violaJerPenaD, violaOmgPenaD, violaThrustPenaD;
    double violaPosPena, violaVelPena, violaAccPena, violaJerPena, violaOmgPena, violaThrustPena;
    double node, pena;
    const auto pieceNum = T.size();
    const double integralFrac = 1.0 / integralResolution;
    double pos_penna_log = 0.0;
    double max_pos_viola_log = 0.0;
    double vel_penna_log = 0.0;
    double max_vel_viola_log = 0.0;
    double acc_penna_log = 0.0;
    double max_acc_viola_log = 0.0;
    double jer_penna_log = 0.0;
    double max_jer_viola_log = 0.0;
    double omg_penna_log = 0.0;
    double max_omg_viola_log = 0.0;
    double thr_penna_log = 0.0;
    double max_thr_viola_log = 0.0;

    double piece_start_t = 0.0;
    grad_start_wt = 0.0;
    for (int i = 0; i < pieceNum; i++) {
        const Eigen::Matrix<double, 8, 3> &c = coeffs.block<8, 3>(i * 8, 0);

        step = T(i) * integralFrac;
        for (int j = 0; j <= integralResolution; j++) {
            s1 = j * step;
            s2 = s1 * s1;
            s3 = s2 * s1;
            s4 = s2 * s2;
            s5 = s4 * s1;
            s6 = s4 * s2;
            s7 = s4 * s3;
            beta0 << 1.0, s1, s2, s3, s4, s5, s6, s7;
            beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4, 6.0 * s5, 7.0 * s6;
            beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3, 30.0 * s4, 42.0 * s5;
            beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2, 120.0 * s3, 210.0 * s4;
            beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1, 360.0 * s2, 840.0 * s3;
//            beta5 << 0.0, 0.0, 0.0, 0., 0.0, 120.0, 720.0 * s1, 2520.0 * s2;
            pos = c.transpose() * beta0;
            vel = c.transpose() * beta1;
            acc = c.transpose() * beta2;
            jer = c.transpose() * beta3;
            sna = c.transpose() * beta4;

            const auto K = hPoly.rows();

            violaVel = vel.squaredNorm() - vmaxSqr;
            violaAcc = acc.squaredNorm() - amaxSqr;
            violaJer = jer.squaredNorm() - jmaxSqr;
            gradThr = 0.0;
//            gradQuat.setZero();
            gradPos << 0, 0, 0;
            gradVel << 0, 0, 0;
            gradAcc << 0, 0, 0;
            gradJer << 0, 0, 0;
            gradOmg << 0, 0, 0;
            pena = 0.0;
            double density_time_grad = 0.0;

            if (weightPos > 0) {
                for (int k = 0; k < K; k++) {
                    outerNormal = hPoly.block<1, 3>(k, 0);
                    violaPos = outerNormal.dot(pos) + hPoly(k, 3);
                    if (gcopter::smoothedL1(violaPos, smoothFactor, violaPosPena, violaPosPenaD)) {
                        gradPos += weightPos * violaPosPenaD * outerNormal;
                        pena += weightPos * violaPosPena;
                        pos_penna_log += weightPos * violaPosPena;
                    }
                }
            }

            if (local_density_en && local_density_weight > 0.0 && !swarm_predictions.empty()) {
                double neighbor_count;
                Vec3f density_pos_grad;
                evaluateLocalDensity(pos, trajectory_start_wt + piece_start_t + s1, swarm_predictions,
                                     local_density_radius_x, local_density_radius_y, local_density_radius_z,
                                     local_density_smooth_eps, local_density_time_inflation,
                                     neighbor_count, density_pos_grad, density_time_grad);
                const double density_scale = 2.0 * local_density_weight * neighbor_count;
                pena += local_density_weight * neighbor_count * neighbor_count;
                gradPos += density_scale * density_pos_grad;
                density_time_grad *= density_scale;
            }

            if (weightVel > 0 && gcopter::smoothedL1(violaVel, smoothFactor, violaVelPena, violaVelPenaD)) {
                gradVel += weightVel * violaVelPenaD * 2.0 * vel;
                pena += weightVel * violaVelPena;
                vel_penna_log += weightVel * violaVelPena;
            }

            if (weightAcc > 0 && gcopter::smoothedL1(violaAcc, smoothFactor, violaAccPena, violaAccPenaD)) {
                gradAcc += weightAcc * violaAccPenaD * 2.0 * acc;
                pena += weightAcc * violaAccPena;
                acc_penna_log += weightAcc * violaAccPena;
            }

            if (weightJer > 0 && gcopter::smoothedL1(violaJer, smoothFactor, violaJerPena, violaJerPenaD)) {
                gradJer += weightJer * violaJerPenaD * 2.0 * jer;
                pena += weightJer * violaJerPena;
                jer_penna_log += weightJer * violaJerPena;
            }

            if (weightOmg > 0 && weightAccThr > 0) {
                flatMap.forward(vel, acc, jer, 0.0, 0.0, thr, quat, omg);
                violaOmg = omg.squaredNorm() - omgmaxSqr;
                violaThrust = (thr - thrustMean) * (thr - thrustMean) - thrustSqrRadi;

                if (gcopter::smoothedL1(violaOmg, smoothFactor, violaOmgPena, violaOmgPenaD)) {
                    gradOmg += weightOmg * violaOmgPenaD * 2.0 * omg;
                    pena += weightOmg * violaOmgPena;
                    omg_penna_log += weightOmg * violaOmgPena;
                }

                if (gcopter::smoothedL1(violaThrust, smoothFactor, violaThrustPena, violaThrustPenaD)) {
                    gradThr += weightAccThr * violaThrustPenaD * 2.0 * (thr - thrustMean);
                    pena += weightAccThr * violaThrustPena;
                    thr_penna_log += weightAccThr * violaThrustPena;
                }

//            if (smoothedL1(violaTheta, smoothFactor, violaThetaPena, violaThetaPenaD))
//            {
//                gradQuat += weightTheta * violaThetaPenaD /
//                            sqrt(1.0 - cos_theta * cos_theta) * 4.0 *
//                            Eigen::Vector4d(0.0, quat(1), quat(2), 0.0);
//                pena += weightTheta * violaThetaPena;
//            }


                flatMap.backward(gradPos, gradVel, gradAcc, gradJer, gradThr, Vec4f(0, 0, 0, 0), gradOmg,
                                 totalGradPos, totalGradVel, totalGradAcc, totalGradJer,
                                 totalGradPsi, totalGradPsiD);

            } else {
                totalGradPos = gradPos;
                totalGradVel = gradVel;
                totalGradAcc = gradAcc;
                totalGradJer = gradJer;
            }

            {
                // log the max violation
                if (violaVel > max_vel_viola_log) max_vel_viola_log = violaVel;
                if (violaAcc > max_acc_viola_log) max_acc_viola_log = violaAcc;
                if (violaJer > max_jer_viola_log) max_jer_viola_log = violaJer;
                if (violaOmg > max_omg_viola_log) max_omg_viola_log = violaOmg;
                if (violaThrust > max_thr_viola_log) max_thr_viola_log = violaThrust;
            }

            node = (j == 0 || j == integralResolution) ? 0.5 : 1.0;
            alpha = j * integralFrac;
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
                        node * integralFrac * pena;
            for (int previous_piece = 0; previous_piece < i; ++previous_piece) {
                gradT(previous_piece) += node * step * density_time_grad;
            }
            gradT(i) += node * step * alpha * density_time_grad;
            grad_start_wt += node * step * density_time_grad;
            cost += node * step * pena;
        }
        piece_start_t += T(i);
    }

//    pena_log(1) = pos_penna_log;
//    pena_log(2) = vel_penna_log;
//    pena_log(3) = acc_penna_log;
//    pena_log(4) = jer_penna_log;
//    pena_log(5) = att_penna_log;
//    pena_log(6) = omg_penna_log;
//    pena_log(7) = thr_penna_log;
    pena_log(1) = max_pos_viola_log;
    pena_log(2) = max_vel_viola_log;
    pena_log(3) = max_acc_viola_log;
    pena_log(4) = max_jer_viola_log;
    pena_log(5) = 0.0;
    pena_log(6) = max_omg_viola_log;
    pena_log(7) = max_thr_viola_log;
}


double BackupTrajOpt::costFunctional(void *ptr, const Eigen::VectorXd &x, Eigen::VectorXd &g) {
//            TimeConsuming t_("cost functional");
    auto &obj = *static_cast<OptimizationVariables *>(ptr);
    obj.iter_num++;
    const long dimTau = obj.temporalDim;
    const long dimXi = obj.spatialDim;
    const double weightT = obj.rho;
    const double weight_ts = obj.weight_ts;
    const bool optimize_ts = obj.weight_ts > 0.0 ||
                             obj.ts_delay_weight > 0.0 ||
                             obj.ts_delay_high_order_weight > 0.0 ||
                             obj.ts_anchor_weight > 0.0 ||
                             obj.trigger_density_weight > 0.0;
    Eigen::Map<const Eigen::VectorXd> tau(x.data(), dimTau);
    Eigen::Map<const Eigen::VectorXd> xi(x.data() + dimTau, dimXi);
    double tau_s = x(x.size() - 1);
    Eigen::Map<Eigen::VectorXd> gradTau(g.data(), dimTau);
    Eigen::Map<Eigen::VectorXd> gradXi(g.data() + dimTau, dimXi);
    double gradTaus = 0.0;
    forwardMapTauToBoundedT(tau, obj.times);
    if (!obj.times.allFinite()) {
        g.setZero();
        obj.penalty_log.setZero();
        obj.penalty_log(0) = kInvalidCost;
        return kInvalidCost;
    }

    switch (obj.pos_constraint_type) {
        case 1: {
            VecDf xi_e = xi;
            obj.points = Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>>(xi_e.data(), 3, xi_e.size() / 3);
            break;
        }
        default: {
            gcopter::forwardP(xi, obj.vPolytope, obj.points);
            break;
        }
    }
    if (!obj.points.allFinite()) {
        g.setZero();
        obj.penalty_log.setZero();
        obj.penalty_log(0) = kInvalidCost;
        return kInvalidCost;
    }


    if (optimize_ts) {
        gcopter::mapInfToInterval(obj.min_ts, obj.max_ts, tau_s, obj.ts);
    }

    StatePVAJ headPVAJ, tailPVAJ;
    tailPVAJ.setZero();
    headPVAJ = obj.exp_traj.getState(obj.ts);
    tailPVAJ.col(0) = obj.points.rightCols(1);
    obj.minco.setConditions(headPVAJ, tailPVAJ);
    // points在这里是没有用的
    obj.minco.setParameters(obj.points.leftCols(obj.piece_num - 1), obj.times);
    if (!obj.minco.getCoeffs().allFinite()) {
        g.setZero();
        obj.penalty_log.setZero();
        obj.penalty_log(0) = kInvalidCost;
        return kInvalidCost;
    }
    double cost = 0;
    obj.partialGradByCoeffs.setZero();
    obj.partialGradByTimes.setZero();
    if (!obj.block_energy_cost) {
        obj.minco.getEnergy(cost);
        obj.minco.getEnergyPartialGradByCoeffs(obj.partialGradByCoeffs);
        obj.minco.getEnergyPartialGradByTimes(obj.partialGradByTimes);
    }
    if (!std::isfinite(cost) || !obj.partialGradByCoeffs.allFinite() || !obj.partialGradByTimes.allFinite()) {
        g.setZero();
        obj.penalty_log.setZero();
        obj.penalty_log(0) = kInvalidCost;
        return kInvalidCost;
    }
    obj.penalty_log.setZero();
    obj.penalty_log(0) = cost;
    double density_grad_ts = 0.0;
    constraintsFunctional(obj.times, obj.minco.getCoeffs(), obj.hPolytope,
                          obj.smooth_eps, obj.integral_res,
                          obj.magnitudeBounds, obj.penaltyWeights,
                          obj.quadrotor_flatness, obj.swarm_predictions,
                          obj.exp_traj.start_WT + obj.ts,
                          obj.local_density_en, obj.local_density_radius_x, obj.local_density_radius_y,
                          obj.local_density_radius_z, obj.local_density_smooth_eps,
                          obj.local_density_time_inflation, obj.local_density_weight,
                          cost, obj.partialGradByTimes, obj.partialGradByCoeffs,
                          density_grad_ts,
                          obj.penalty_log);
    if (!std::isfinite(cost) || !obj.partialGradByCoeffs.allFinite() || !obj.partialGradByTimes.allFinite()) {
        g.setZero();
        obj.penalty_log.setZero();
        obj.penalty_log(0) = kInvalidCost;
        return kInvalidCost;
    }

    StatePVAJ partGradOfHeadPVAJ, partGradOfTailPVAJ;
    Mat3Df partGradOfWaypts;
    obj.minco.propagateGradOfWayptsAndState(obj.partialGradByCoeffs, obj.partialGradByTimes,
                                            obj.gradByTimes,
                                            partGradOfHeadPVAJ,
                                            partGradOfWaypts,
                                            partGradOfTailPVAJ);
    if (!obj.gradByTimes.allFinite() || !partGradOfHeadPVAJ.allFinite() ||
        !partGradOfWaypts.allFinite() || !partGradOfTailPVAJ.allFinite()) {
        g.setZero();
        obj.penalty_log.setZero();
        obj.penalty_log(0) = kInvalidCost;
        return kInvalidCost;
    }
    cost += weightT * obj.times.sum();
    obj.gradByTimes.array() += weightT;
    obj.gradByPoints.leftCols(obj.piece_num - 1) = partGradOfWaypts;
    obj.gradByPoints.rightCols(1) = partGradOfTailPVAJ.col(0);

    if (obj.tail_anchor_weight > 0.0 && obj.tail_anchor_pos.allFinite()) {
        const Vec3f tail_delta = obj.points.col(obj.points.cols() - 1) - obj.tail_anchor_pos;
        const double tail_deadband = std::max(0.0, obj.tail_anchor_deadband);
        const double viola_tail = tail_delta.squaredNorm() - tail_deadband * tail_deadband;
        double tail_pena, tail_pena_d;
        if (gcopter::smoothedL1(viola_tail, obj.smooth_eps, tail_pena, tail_pena_d)) {
            cost += obj.tail_anchor_weight * tail_pena;
            obj.gradByPoints.rightCols(1) += obj.tail_anchor_weight * tail_pena_d * 2.0 * tail_delta;
        }
    }

    if (obj.time_balance_weight > 0.0 && obj.times.size() > 1) {
        const double mean_time = obj.times.mean();
        const Eigen::VectorXd time_delta = (obj.times.array() - mean_time).matrix();
        cost += obj.time_balance_weight * time_delta.squaredNorm();
        obj.gradByTimes += 2.0 * obj.time_balance_weight * time_delta;
    }

    gcopter::propagateGradientTToTau(tau, obj.gradByTimes, gradTau);
    switch (obj.pos_constraint_type) {
        case 1: {
            MatDf gp = obj.gradByPoints;
            gradXi = Eigen::Map<Eigen::VectorXd>(gp.data(), gp.size());
            break;
        }
        default: {
            gcopter::backwardGradP(xi, obj.vPolytope, obj.gradByPoints, gradXi);
            gcopter::normRetrictionLayer(xi, obj.vPolytope, cost, gradXi);
            break;
        }
    }
    if (!std::isfinite(cost) || !gradTau.allFinite() || !gradXi.allFinite()) {
        g.setZero();
        obj.penalty_log.setZero();
        obj.penalty_log(0) = kInvalidCost;
        return kInvalidCost;
    }
    // Add ts cost and gradient;
    if (optimize_ts) {
        // square cost
//        cost += weight_ts * pow(obj.bod_.t_e - obj.ts, 2);
//        obj.gradTs =
//                partGradOfHeadPVAJ.col(0).dot(obj.bod_.exp_traj.getVel(obj.ts)) +
//                partGradOfHeadPVAJ.col(1).dot(obj.bod_.exp_traj.getAcc(obj.ts)) +
//                partGradOfHeadPVAJ.col(2).dot(obj.bod_.exp_traj.getJer(obj.ts)) +
//                partGradOfHeadPVAJ.col(3).dot(obj.bod_.exp_traj.getSnap(obj.ts)) +
//                -weight_ts * 2 * (obj.bod_.t_e - obj.ts);
//        gcopter::propagateGradIntervalToInf(obj.bod_.t_0, obj.bod_.t_e, tau_s, obj.gradTs, gradTaus);
//        g(g.size() - 1) = gradTaus;
        // linear cost
//        double vioTs = (obj.bod_.t_e - obj.ts);
//        double vioTsPen, vioTsPenD;
//        gcopter::smoothedL1(vioTs, obj.smoothEps, vioTsPen, vioTsPenD);
//        cost += weight_ts * vioTsPen;
//        double gradTs = partGradOfHeadPVAJ.col(0).dot(obj.bod_.exp_traj.getVel(obj.ts)) +
//                        partGradOfHeadPVAJ.col(1).dot(obj.bod_.exp_traj.getAcc(obj.ts)) +
//                        partGradOfHeadPVAJ.col(2).dot(obj.bod_.exp_traj.getJer(obj.ts)) +
//                        partGradOfHeadPVAJ.col(3).dot(obj.bod_.exp_traj.getSnap(obj.ts)) +
//                        -weight_ts * vioTsPenD;
//        gcopter::propagateGradIntervalToInf(obj.bod_.t_0, obj.bod_.t_e, tau_s, gradTs, gradTaus);
//        g(g.size() - 1) = gradTaus;

        cost += weight_ts > 0.0 ? weight_ts * (obj.max_ts - obj.ts) : 0.0;
        obj.gradTs =
                partGradOfHeadPVAJ.col(0).dot(obj.exp_traj.getVel(obj.ts)) +
                partGradOfHeadPVAJ.col(1).dot(obj.exp_traj.getAcc(obj.ts)) +
                partGradOfHeadPVAJ.col(2).dot(obj.exp_traj.getJer(obj.ts)) +
                partGradOfHeadPVAJ.col(3).dot(obj.exp_traj.getSnap(obj.ts)) +
                (weight_ts > 0.0 ? -weight_ts : 0.0) + density_grad_ts;

        if (obj.ts_delay_weight > 0.0) {
            const double target_ts = obj.max_ts - std::max(0.0, obj.ts_delay_deadband);
            const double early_switch = target_ts - obj.ts;
            if (early_switch > 0.0) {
                cost += obj.ts_delay_weight * early_switch * early_switch;
                obj.gradTs += -2.0 * obj.ts_delay_weight * early_switch;
            }
        }

        if (obj.ts_delay_high_order_weight > 0.0) {
            const double target_ts = obj.max_ts - std::max(0.0, obj.ts_delay_deadband);
            const double early_switch = target_ts - obj.ts;
            if (early_switch > 0.0) {
                const double early_switch_sqr = early_switch * early_switch;
                cost += obj.ts_delay_high_order_weight * early_switch_sqr * early_switch_sqr;
                obj.gradTs += -4.0 * obj.ts_delay_high_order_weight * early_switch_sqr * early_switch;
            }
        }

        if (obj.ts_anchor_weight > 0.0 && std::isfinite(obj.reference_ts)) {
            const double ts_delta = obj.ts - obj.reference_ts;
            const double ts_deadband = std::max(0.0, obj.ts_anchor_deadband);
            const double viola_ts = ts_delta * ts_delta - ts_deadband * ts_deadband;
            double ts_pena, ts_pena_d;
            if (gcopter::smoothedL1(viola_ts, obj.smooth_eps, ts_pena, ts_pena_d)) {
                cost += obj.ts_anchor_weight * ts_pena;
                obj.gradTs += obj.ts_anchor_weight * ts_pena_d * 2.0 * ts_delta;
            }
        }

        if (obj.trigger_density_weight > 0.0) {
            const Vec3f trigger_pos = obj.exp_traj.getPos(obj.ts);
            const Vec3f trigger_vel = obj.exp_traj.getVel(obj.ts);
            double static_density = 0.0, swarm_density = 0.0, swarm_time_grad = 0.0;
            Vec3f static_grad_pos = Vec3f::Zero(), swarm_grad_pos = Vec3f::Zero();
            evaluateStaticDensity(trigger_pos, obj.hPolytope, obj.trigger_static_clearance,
                                  obj.local_density_smooth_eps, static_density, static_grad_pos);
            if (!obj.swarm_predictions.empty()) {
                evaluateLocalDensity(trigger_pos, obj.exp_traj.start_WT + obj.ts, obj.swarm_predictions,
                                     obj.local_density_radius_x, obj.local_density_radius_y,
                                     obj.local_density_radius_z, obj.local_density_smooth_eps,
                                     obj.local_density_time_inflation, swarm_density,
                                     swarm_grad_pos, swarm_time_grad);
            }

            const double trigger_density =
                    obj.trigger_static_weight * static_density + obj.trigger_swarm_weight * swarm_density;
            const double trigger_density_grad_ts =
                    obj.trigger_static_weight * static_grad_pos.dot(trigger_vel) +
                    obj.trigger_swarm_weight * (swarm_grad_pos.dot(trigger_vel) + swarm_time_grad);
            const double trigger_window = std::max(1.0e-3, obj.max_ts - obj.min_ts);
            const double trigger_progress = (obj.ts - obj.min_ts) / trigger_window;
            const double trigger_progress_sqr = trigger_progress * trigger_progress;
            const double trigger_cost =
                    obj.trigger_density_weight * trigger_density * trigger_progress_sqr;

            cost += trigger_cost;
            obj.gradTs += obj.trigger_density_weight *
                    (trigger_density_grad_ts * trigger_progress_sqr +
                     2.0 * trigger_density * trigger_progress / trigger_window);
        }

        gcopter::propagateGradIntervalToInf(obj.min_ts, obj.max_ts, tau_s, obj.gradTs, gradTaus);
        g(g.size() - 1) = gradTaus;

    } else {
        g(g.size() - 1) = 0;
    }
    if (!std::isfinite(cost) || !g.allFinite()) {
        g.setZero();
        obj.penalty_log.setZero();
        obj.penalty_log(0) = kInvalidCost;
        return kInvalidCost;
    }
    return cost;
}

bool BackupTrajOpt::processCorridor() {
    PolyhedronV curIV, curIOB; // 走廊的顶点
    if (!geometry_utils::enumerateVs(opt_vars.hPolytope, curIV)) {
        std::cout << YELLOW << " -- [MINCO] enumerateVs failed." << RESET << std::endl;
        return false;
    }
    long nv = curIV.cols();
    curIOB.resize(3, nv);
    // 第一个点存储第一个顶点
    curIOB.col(0) = curIV.col(0);
    // 后面的点都归一化到第一个点坐标系下
    curIOB.rightCols(nv - 1) = curIV.rightCols(nv - 1).colwise() - curIV.col(0);
    opt_vars.vPolytope = curIOB;
    return true;
}

bool BackupTrajOpt::setupProblemAndCheck() {
    // 1. Check if the corridor is feasible
    const Eigen::ArrayXd norms = opt_vars.hPolytope.leftCols<3>().rowwise().norm();
    opt_vars.hPolytope.array().colwise() /= norms;

    if (!processCorridor()) {
        std::cout << YELLOW << " -- [processCorridor] Failed to get Overlap enumerateVs ." << RESET << std::endl;
        return false;
    }

    // 2. Reset the problem dimension
    opt_vars.temporalDim = opt_vars.piece_num;
    switch (opt_vars.pos_constraint_type) {
        case 1: {
            opt_vars.spatialDim = 3 * opt_vars.piece_num;
            break;
        }
        default: {
            opt_vars.spatialDim = opt_vars.vPolytope.cols() * opt_vars.piece_num;
        }
    }

    opt_vars.minco.setConditions(opt_vars.headPVAJ, opt_vars.tailPVAJ, opt_vars.piece_num);
    opt_vars.points.resize(3, opt_vars.piece_num);
    opt_vars.gradByPoints.resize(3, opt_vars.piece_num);
    opt_vars.partialGradByCoeffs.resize(8 * opt_vars.piece_num, 3);
    opt_vars.partialGradByTimes.resize(opt_vars.piece_num);
    return true;
}

double BackupTrajOpt::optimize(Trajectory &traj, const double &relCostTol) {
    // 1. Initialize the trajectory
    //      the optimization varibles include time allocation [1] * pieceN + tailWaypoints [vPoly_size] * pieceN + split time [1]
    Eigen::VectorXd x(opt_vars.temporalDim + opt_vars.spatialDim + 1);
    Eigen::Map<Eigen::VectorXd> tau(x.data(), opt_vars.temporalDim);
    Eigen::Map<Eigen::VectorXd> xi(x.data() + opt_vars.temporalDim, opt_vars.spatialDim);

    // 2. Initialize the optimization problem
    // 初始化均匀时间分配，均匀waypoint分配
    Vec3f step = (opt_vars.tailPVAJ.col(0) - opt_vars.headPVAJ.col(0)) / opt_vars.piece_num;
    for (int i = 0; i < opt_vars.piece_num - 1; i++) {
        opt_vars.points.col(i) = step * (i + 1) + opt_vars.headPVAJ.col(0);
    }
    opt_vars.points.rightCols(1) = opt_vars.tailPVAJ.col(0);

    if(opt_vars.given_init_ts_and_ps){
        opt_vars.times = opt_vars.given_init_t_vec;
        for (int i = 0; i < opt_vars.given_init_ps.size(); i++) {
            opt_vars.points.col(i) = opt_vars.given_init_ps[i];
        }
        opt_vars.ts = opt_vars.given_init_ts;
    }

    if (!opt_vars.headPVAJ.allFinite() || !opt_vars.tailPVAJ.allFinite() ||
        !opt_vars.times.allFinite() || opt_vars.times.minCoeff() < kMinStablePieceTime ||
        !opt_vars.points.allFinite() || !std::isfinite(opt_vars.ts)) {
        cout << YELLOW << " -- [TrajOpt] Error, invalid backup init state/time/points, force return." << RESET << endl;
        cout << " -- Head PVAJ: " << endl;
        cout << opt_vars.headPVAJ << endl;
        cout << " -- Tail PVAJ: " << endl;
        cout << opt_vars.tailPVAJ << endl;
        cout << " -- Times: " << endl;
        cout << opt_vars.times.transpose() << endl;
        return INFINITY;
    }

    backwardMapBoundedTToTau(opt_vars.times, tau);
    Eigen::VectorXd tt;
    forwardMapTauToBoundedT(tau, tt);
    switch (opt_vars.pos_constraint_type) {
        case 1: {
            MatDf p_e = opt_vars.points;
            xi = Eigen::Map<const VecDf>(p_e.data(), p_e.size());
            break;
        }
        default: {
            gcopter::backwardP(opt_vars.points, opt_vars.vPolytope, xi);
            break;
        }
    }

    double tau_s;
    gcopter::mapIntervalToInf(opt_vars.min_ts, opt_vars.max_ts, opt_vars.ts, tau_s);
    x(x.size() - 1) = tau_s;
    double minCostFunctional;
    lbfgs::lbfgs_parameter_t lbfgs_params;
    lbfgs_params.mem_size = 256;
    lbfgs_params.past = 3;
    lbfgs_params.min_step = 1.0e-32;
    lbfgs_params.g_epsilon = 0.0;
    lbfgs_params.delta = relCostTol;
    int ret;
    opt_vars.penalty_log.resize(8);
    opt_vars.penalty_log.setZero();
    opt_vars.iter_num = 0;

    opt_vars.init_ts = opt_vars.ts;
    opt_vars.init_t_vec = opt_vars.times;
    opt_vars.init_ps.clear();
    for (int col = 0; col < opt_vars.points.cols(); col++) {
        opt_vars.init_ps.emplace_back(opt_vars.points.col(col));
    }

    TimeConsuming ttt(" -- [BackupTrajOpt]", false);
    if (opt_vars.debug_en) {
        throw std::runtime_error(" -- [BackupTrajOpt] Debug mode is not supported yet.");
    } else {
        ret = lbfgs::lbfgs_optimize(x,
                                    minCostFunctional,
                                    &BackupTrajOpt::costFunctional,
                                    nullptr,
                                    nullptr,
                                    &this->opt_vars,
                                    lbfgs_params);

    }
    using namespace std;
    if (cfg_.print_optimizer_log) {
        cout << " -- [BaclOpt] Opt finish, with iter num: " << opt_vars.iter_num << "\n";
        cout << "\tEnergy: " << opt_vars.penalty_log(0) << endl;
        cout << "\tPos: " << opt_vars.penalty_log(1) << endl;
        cout << "\tVel: " << opt_vars.penalty_log(2) << endl;
        cout << "\tAcc: " << opt_vars.penalty_log(3) << endl;
        cout << "\tJerk: " << opt_vars.penalty_log(4) << endl;
        cout << "\tAttract: " << opt_vars.penalty_log(5) << endl;
        cout << "\tOmg: " << opt_vars.penalty_log(6) << endl;
        cout << "\tThr: " << opt_vars.penalty_log(7) << endl;
        cout << "\tTs: " << opt_vars.ts << endl;
    }
    const double vel_violation_tol = squaredBoundViolationTolerance(cfg_.max_vel, cfg_.penna_margin);
    const double acc_violation_tol = squaredBoundViolationTolerance(cfg_.max_acc, cfg_.penna_margin);
    const double omg_violation_tol = squaredBoundViolationTolerance(cfg_.max_omg, cfg_.penna_margin);
    const double thr_violation_tol = thrustViolationTolerance(cfg_);
    if ((cfg_.penna_pos > 0 && opt_vars.penalty_log(1) > 0.2) ||
        (cfg_.penna_vel > 0 && opt_vars.penalty_log(2) > vel_violation_tol) ||
        (cfg_.penna_acc > 0 && opt_vars.penalty_log(3) > acc_violation_tol) ||
        (cfg_.penna_omg > 0 && opt_vars.penalty_log(6) > omg_violation_tol) ||
        (cfg_.penna_thr > 0 && opt_vars.penalty_log(7) > thr_violation_tol)) {
        ret = -1;
        if (cfg_.print_optimizer_log) {
            cout << " -- [BaclOpt] Opt finish, with iter num: " << opt_vars.iter_num << "\n";
            cout << "\tEnergy: " << opt_vars.penalty_log(0) << endl;
            cout << "\tPos: " << opt_vars.penalty_log(1) << endl;
            cout << "\tVel: " << opt_vars.penalty_log(2) << endl;
            cout << "\tAcc: " << opt_vars.penalty_log(3) << endl;
            cout << "\tJerk: " << opt_vars.penalty_log(4) << endl;
            cout << "\tAttract: " << opt_vars.penalty_log(5) << endl;
            cout << "\tOmg: " << opt_vars.penalty_log(6) << endl;
            cout << "\tThr: " << opt_vars.penalty_log(7) << endl;
            cout << "\tTs: " << opt_vars.ts << endl;
        }
        ros_ptr_->warn(" -- [BackOpt] Opt failed, Omg or thr or Pos violation.");
    }

    if (ret >= 0) {
        VecDf Ts;
        forwardMapTauToBoundedT(tau, opt_vars.times);
        switch (opt_vars.pos_constraint_type) {
            case 1: {
                VecDf xi_e = xi;
                opt_vars.points = Eigen::Map<Eigen::Matrix<double, 3, Eigen::Dynamic>>(xi_e.data(), 3, xi_e.size() / 3);
                break;
            }
            default: {
                gcopter::forwardP(xi, opt_vars.vPolytope, opt_vars.points);
                break;
            }
        }

        opt_vars.tailPVAJ.setZero();
        opt_vars.headPVAJ = opt_vars.exp_traj.getState(opt_vars.ts);
        opt_vars.tailPVAJ.col(0) = opt_vars.points.rightCols(1);
        opt_vars.minco.setConditions(opt_vars.headPVAJ, opt_vars.tailPVAJ, opt_vars.piece_num);
        opt_vars.minco.setEndPosition(opt_vars.points.rightCols(1));
        opt_vars.minco.setParameters(opt_vars.points.leftCols(opt_vars.piece_num - 1), opt_vars.times);
        opt_vars.minco.getTrajectory(traj);
    } else {
        traj.clear();
        minCostFunctional = INFINITY;
        std::cout << YELLOW << " -- [MINCO] TrajOpt failed, " << lbfgs::lbfgs_strerror(ret) << RESET << std::endl;
    }
    return minCostFunctional;
}

BackupTrajOpt::BackupTrajOpt(const traj_opt::Config &cfg, const ros_interface::RosInterface::Ptr &ros_ptr)
        : cfg_(cfg), ros_ptr_(ros_ptr) {
    using namespace std;

    cfg_ = cfg;
    std::string filename = "back_opt_log.csv";
    if(cfg_.save_log_en){
        failed_traj_log.open(DEBUG_FILE_DIR(filename), std::ios::out | std::ios::trunc);
        penalty_log.open(DEBUG_FILE_DIR("back_opt_penna.csv"), std::ios::out | std::ios::trunc);
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
    opt_vars.weight_ts = cfg_.penna_ts;
    opt_vars.local_density_en = cfg_.local_density_en;
    opt_vars.local_density_radius_x = cfg_.local_density_radius_x;
    opt_vars.local_density_radius_y = cfg_.local_density_radius_y;
    opt_vars.local_density_radius_z = cfg_.local_density_radius_z;
    opt_vars.local_density_smooth_eps = cfg_.local_density_smooth_eps;
    opt_vars.local_density_time_inflation = cfg_.local_density_time_inflation;
    opt_vars.local_density_weight = cfg_.local_density_weight;
    opt_vars.trigger_density_weight = cfg_.trigger_density_weight;
    opt_vars.trigger_static_clearance = cfg_.trigger_static_clearance;
    opt_vars.trigger_static_weight = cfg_.trigger_static_weight;
    opt_vars.trigger_swarm_weight = cfg_.trigger_swarm_weight;
    opt_vars.ts_delay_weight = cfg_.ts_delay_weight;
    opt_vars.ts_delay_high_order_weight = cfg_.ts_delay_high_order_weight;
    opt_vars.ts_delay_deadband = cfg_.ts_delay_deadband;
    opt_vars.ts_anchor_weight = cfg_.ts_anchor_weight;
    opt_vars.ts_anchor_deadband = cfg_.ts_anchor_deadband;
    opt_vars.tail_anchor_weight = cfg_.tail_anchor_weight;
    opt_vars.tail_anchor_deadband = cfg_.tail_anchor_deadband;
    opt_vars.time_balance_weight = cfg_.time_balance_weight;
}

bool BackupTrajOpt::checkTrajMagnitudeBound(Trajectory &out_traj) {
    if (out_traj.empty() || !std::isfinite(out_traj.getTotalDuration()) ||
        out_traj.getTotalDuration() <= kMinStablePieceTime) {
        std::cout << YELLOW << " -- [TrajOpt] Minco backup opt produced invalid trajectory." << RESET << std::endl;
        return false;
    }

    const double max_vel_rate = out_traj.getMaxVelRate();
    if (!std::isfinite(max_vel_rate) || (cfg_.penna_vel > 0 && max_vel_rate > 1.2 * cfg_.max_vel)) {
        std::cout << YELLOW << " -- [TrajOpt] Minco backup opt failed." << RESET << std::endl;
        std::cout << YELLOW << "\t\tBackend Max vel:\t" << max_vel_rate << " m/s" << RESET << std::endl;
        return false;
    }

    const double max_acc_rate = out_traj.getMaxAccRate();
    if (!std::isfinite(max_acc_rate) || (cfg_.penna_acc > 0 && max_acc_rate > 1.2 * cfg_.max_acc)) {
        std::cout << YELLOW << " -- [TrajOpt] Minco backup opt failed." << RESET << std::endl;
        std::cout << YELLOW << "\t\tBackend Max Acc:\t" << max_acc_rate << " m/s" << RESET << std::endl;
        return false;
    }
    return true;
}

bool
BackupTrajOpt::optimize(const Trajectory &exp_traj,
                        const double &t_0,
                        const double &t_e,
                        const double &heu_ts,
                        const VecDf &heu_end_pt,
                        double &heu_dur,
                        const Polytope &sfc,
                        Trajectory &out_traj,
                        double &out_ts,
                        const vector<SwarmPrediction> &swarm_predictions,
                        const bool &debug) {
    opt_vars.hPolytope = sfc.GetPlanes();
    out_traj.clear();
    out_ts = heu_ts;
    if (!std::isfinite(t_0) || !std::isfinite(t_e) || !std::isfinite(heu_ts) ||
        !std::isfinite(heu_dur) || t_e <= t_0 + kMinStablePieceTime ||
        heu_ts < t_0 || heu_ts > t_e || heu_dur <= kMinStablePieceTime ||
        cfg_.piece_num <= 0 || !heu_end_pt.allFinite() || exp_traj.empty() ||
        !std::isfinite(exp_traj.getTotalDuration()) ||
        exp_traj.getTotalDuration() <= kMinStablePieceTime ||
        opt_vars.hPolytope.size() == 0 || !opt_vars.hPolytope.allFinite()) {
        std::cout << YELLOW << " -- [BackTrajOpt] Invalid backup optimization input." << RESET << std::endl;
        return false;
    }

    opt_vars.debug_en = debug;
    /// Setup optimization problems
    opt_vars.default_init = true;
    opt_vars.given_init_ts_and_ps = false;
    if (!exp_traj.getState(heu_ts, opt_vars.headPVAJ)) {
        std::cout << YELLOW << " -- [BackTrajOpt] Invalid backup head state." << RESET << std::endl;
        return false;
    }
    opt_vars.tailPVAJ.setZero();
    opt_vars.guide_path.clear();
    opt_vars.guide_t.clear();
    opt_vars.exp_traj = exp_traj;
    opt_vars.swarm_predictions = swarm_predictions;
    opt_vars.piece_num = cfg_.piece_num;
    opt_vars.max_ts = t_e;
    opt_vars.min_ts = t_0;
    opt_vars.reference_ts = heu_ts;
    opt_vars.tail_anchor_pos = heu_end_pt;
    opt_vars.tailPVAJ.col(0) = heu_end_pt;
    opt_vars.times.resize(opt_vars.piece_num);
    const double min_init_duration =
            static_cast<double>(opt_vars.piece_num) * (kMinStablePieceTime + kMinMappedPieceTime);
    heu_dur = std::max(heu_dur, min_init_duration);
    opt_vars.times.setConstant(heu_dur / opt_vars.piece_num);
    opt_vars.ts = heu_ts;

    PolyhedronH planes = sfc.GetPlanes();
    bool success{true};

    if (!setupProblemAndCheck()) {
        std::cout << YELLOW << " -- [TrajOpt] Minco corridor preprocess error." << RESET << std::endl;
        success = false;
    }

    const double opt_cost = success ? optimize(out_traj, cfg_.opt_accuracy) : INFINITY;
    if (success && !std::isfinite(opt_cost)) {
        std::cout << YELLOW << " -- [SUPER] Minco backup_traj opt failed." << RESET << std::endl;
        success = false;
    }

    if (success && opt_vars.penalty_log.size() > 1 &&
        opt_vars.penalty_log(1) > cfg_.penna_pos * 0.05) {
        std::cout << YELLOW << " -- [SUPER] Minco backup_traj out of corridor." << RESET << std::endl;
        success = false;
    }
    out_ts = opt_vars.ts;

    if (success && !checkTrajMagnitudeBound(out_traj)) {
        success = false;
    }

    if (!success && cfg_.save_log_en) {
        // log the optimization problem
        failed_traj_log << 123321 << std::endl;
        failed_traj_log << t_0 << std::endl;
        failed_traj_log << t_e << std::endl;
        failed_traj_log << heu_ts << std::endl;
        failed_traj_log << heu_end_pt.transpose() << std::endl;
        failed_traj_log << heu_dur << std::endl;
        failed_traj_log << 0 << std::endl;
        failed_traj_log << sfc.GetPlanes() << std::endl;
        failed_traj_log << exp_traj.getPieceNum() << std::endl;
        failed_traj_log << exp_traj.getDurations().transpose() << std::endl;
        for (int i = 0; i < exp_traj.getPieceNum(); i++) {
            failed_traj_log << exp_traj[i].getCoeffMat() << std::endl;
        }
        out_ts = heu_ts;
    }

    return success;
}

bool
BackupTrajOpt::optimize(const Trajectory &exp_traj,
                        const double &t_0,
                        const double &t_e,
                        const double &heu_ts,
                        const Polytope &sfc,
                        const VecDf & init_t_vec,
                        const vec_Vec3f &init_ps,
                        Trajectory &out_traj,
                        double & out_ts) {
    opt_vars.hPolytope = sfc.GetPlanes();
    out_traj.clear();
    out_ts = heu_ts;
    if (!std::isfinite(t_0) || !std::isfinite(t_e) || !std::isfinite(heu_ts) ||
        t_e <= t_0 + kMinStablePieceTime || heu_ts < t_0 || heu_ts > t_e ||
        cfg_.piece_num <= 0 || exp_traj.empty() ||
        !std::isfinite(exp_traj.getTotalDuration()) ||
        exp_traj.getTotalDuration() <= kMinStablePieceTime ||
        init_ps.empty() || init_t_vec.size() != cfg_.piece_num ||
        init_ps.size() != static_cast<size_t>(cfg_.piece_num) ||
        !init_t_vec.allFinite() || init_t_vec.minCoeff() <= kMinStablePieceTime ||
        opt_vars.hPolytope.size() == 0 || !opt_vars.hPolytope.allFinite()) {
        std::cout << YELLOW << " -- [BackTrajOpt] Invalid backup optimization input." << RESET << std::endl;
        return false;
    }

    opt_vars.debug_en = false;
    /// Setup optimization problems
    opt_vars.default_init = true;

    if (!exp_traj.getState(heu_ts, opt_vars.headPVAJ)) {
        std::cout << YELLOW << " -- [BackTrajOpt] Invalid backup head state." << RESET << std::endl;
        return false;
    }
    opt_vars.tailPVAJ.setZero();
    opt_vars.guide_path.clear();
    opt_vars.guide_t.clear();
    opt_vars.exp_traj = exp_traj;
    opt_vars.piece_num = cfg_.piece_num;
    opt_vars.max_ts = t_e;
    opt_vars.min_ts = t_0;
    opt_vars.reference_ts = heu_ts;
    opt_vars.tail_anchor_pos = init_ps.back();
    opt_vars.tailPVAJ.col(0) = init_ps.back();
    opt_vars.times.resize(opt_vars.piece_num);
    const double heu_dur = init_t_vec.sum();
    opt_vars.times.setConstant(heu_dur / opt_vars.piece_num);
    opt_vars.ts = heu_ts;

    opt_vars.given_init_ts_and_ps = true;
    opt_vars.given_init_t_vec = init_t_vec;
    opt_vars.given_init_ps = init_ps;
    opt_vars.given_init_ts = heu_ts;

    PolyhedronH planes = sfc.GetPlanes();
    bool success{true};

    if (!setupProblemAndCheck()) {
        std::cout << YELLOW << " -- [TrajOpt] Minco corridor preprocess error." << RESET << std::endl;
        success = false;
    }

    const double opt_cost = success ? optimize(out_traj, cfg_.opt_accuracy) : INFINITY;
    if (success && !std::isfinite(opt_cost)) {
        std::cout << YELLOW << " -- [SUPER] Minco backup_traj opt failed." << RESET << std::endl;
        success = false;
    }

    if (success && opt_vars.penalty_log.size() > 1 &&
        opt_vars.penalty_log(1) > cfg_.penna_pos * 0.05) {
        std::cout << YELLOW << " -- [SUPER] Minco backup_traj out of corridor." << RESET << std::endl;
        success = false;
    }
    out_ts = opt_vars.ts;

    if (success && !checkTrajMagnitudeBound(out_traj)) {
        success = false;
    }


    return success;
}
