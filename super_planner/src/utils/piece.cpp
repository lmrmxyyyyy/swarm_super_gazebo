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

#include <data_structure/base/piece.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <limits>


using namespace geometry_utils;

namespace {
    constexpr double kMinPieceDuration = 1e-6;
    constexpr double kCoeffEps = 1e-12;

    bool isValidPieceData(double duration, int degree, const Eigen::MatrixXd& coeffMat) {
        return std::isfinite(duration) && duration > kMinPieceDuration &&
               degree >= 2 && coeffMat.rows() == 3 && coeffMat.cols() == degree + 1 &&
               coeffMat.allFinite();
    }

    Eigen::VectorXd trimLeadingZeros(const Eigen::VectorXd& coeffs) {
        int first = 0;
        while (first < coeffs.size() && std::fabs(coeffs(first)) < kCoeffEps) {
            ++first;
        }
        if (first >= coeffs.size()) {
            return Eigen::VectorXd();
        }
        return coeffs.tail(coeffs.size() - first).eval();
    }
}

// Piece==================================================

int Piece::getDim() const {
    return 3;
}

int Piece::getDegree() const {
    return D;
}

double Piece::getDuration() const {
    return duration;
}

void Piece::setDuration(double dur) {
    duration = dur;
}

const Eigen::MatrixXd &Piece::getCoeffMat() const {
    return coeffMat;
}

Eigen::Vector3d Piece::getPos(const double &t) const {
    Eigen::Vector3d pos(0.0, 0.0, 0.0);
    double tn = 1.0;
    for (int i = D; i >= 0; i--) {
        pos += tn * coeffMat.col(i);
        tn *= t;
    }
    return pos;
}

Eigen::Vector3d Piece::getVel(const double &t) const {
    Eigen::Vector3d vel(0.0, 0.0, 0.0);
    double tn = 1.0;
    int n = 1;
    for (int i = D - 1; i >= 0; i--) {
        vel += n * tn * coeffMat.col(i);
        tn *= t;
        n++;
    }
    return vel;
}

Eigen::Vector3d Piece::getAcc(const double &t) const {
    Eigen::Vector3d acc(0.0, 0.0, 0.0);
    double tn = 1.0;
    int m = 1;
    int n = 2;
    for (int i = D - 2; i >= 0; i--) {
        acc += m * n * tn * coeffMat.col(i);
        tn *= t;
        m++;
        n++;
    }
    return acc;
}

Eigen::Vector3d Piece::getJer(const double &t) const {
    Eigen::Vector3d jer(0.0, 0.0, 0.0);
    double tn = 1.0;
    int l = 1;
    int m = 2;
    int n = 3;
    for (int i = D - 3; i >= 0; i--) {
        jer += l * m * n * tn * coeffMat.col(i);
        tn *= t;
        l++;
        m++;
        n++;
    }
    return jer;
}

Eigen::Vector3d Piece::getSnap(const double &t) const {
    Eigen::Vector3d sna(0.0, 0.0, 0.0);
    double tn = 1.0;
    int l = 1;
    int m = 2;
    int n = 3;
    int j = 4;
    for (int i = D - 4; i >= 0; i--) {
        sna += l * m * n * j * tn * coeffMat.col(i);
        tn *= t;
        l++;
        m++;
        n++;
        j++;
    }
    return sna;
}


Eigen::MatrixXd Piece::normalizePosCoeffMat() const {
    Eigen::MatrixXd nPosCoeffsMat(3, D + 1);
    double t = 1.0;
    for (int i = D; i >= 0; i--) {
        nPosCoeffsMat.col(i) = coeffMat.col(i) * t;
        t *= duration;
    }
    return nPosCoeffsMat;
}

Eigen::MatrixXd Piece::normalizeVelCoeffMat() const {
    Eigen::MatrixXd nVelCoeffMat(3, D);
    int n = 1;
    double t = duration;
    for (int i = D - 1; i >= 0; i--) {
        nVelCoeffMat.col(i) = n * coeffMat.col(i) * t;
        t *= duration;
        n++;
    }
    return nVelCoeffMat;
}

Eigen::MatrixXd Piece::normalizeAccCoeffMat() const {
    Eigen::MatrixXd nAccCoeffMat(3, D - 1);
    int n = 2;
    int m = 1;
    double t = duration * duration;
    for (int i = D - 2; i >= 0; i--) {
        nAccCoeffMat.col(i) = n * m * coeffMat.col(i) * t;
        n++;
        m++;
        t *= duration;
    }
    return nAccCoeffMat;
}

double Piece::getMaxVelRate() const {
    if (!isValidPieceData(duration, D, coeffMat)) {
        return std::numeric_limits<double>::infinity();
    }
    Eigen::MatrixXd nVelCoeffMat = normalizeVelCoeffMat();
    if (!nVelCoeffMat.allFinite()) {
        return std::numeric_limits<double>::infinity();
    }
    Eigen::VectorXd coeff = math_utils::RootFinder::polySqr(nVelCoeffMat.row(0)) +
                            math_utils::RootFinder::polySqr(nVelCoeffMat.row(1)) +
                            math_utils::RootFinder::polySqr(nVelCoeffMat.row(2));
    if (!coeff.allFinite()) {
        return std::numeric_limits<double>::infinity();
    }
    int N = coeff.size();
    if (N <= 1) {
        return std::max(getVel(0.0).norm(), getVel(duration).norm());
    }
    int n = N - 1;
    for (int i = 0; i < N; i++) {
        coeff(i) *= n;
        n--;
    }
//        print("begin solve poly. {}\n", coeff.head(N - 1).squaredNorm());
    Eigen::VectorXd dcoeff = trimLeadingZeros(coeff.head(N - 1));
    if (dcoeff.size() <= 1 || dcoeff.squaredNorm() < 1e-6) {
        return std::max(getVel(0.0).norm(), getVel(duration).norm());
    } else {
        double l = -0.0625;
        double r = 1.0625;
        int cnt = 0;
        while (fabs(math_utils::RootFinder::polyVal(dcoeff, l)) < DBL_EPSILON) {
            l = 0.5 * l;
            if (cnt++ > 100) {
                std::cout << "math_utils::RootFinder stuck in inf-loop" << std::endl;
                return std::numeric_limits<double>::infinity();
            }
        }
        cnt = 0;
        while (fabs(math_utils::RootFinder::polyVal(dcoeff, r)) < DBL_EPSILON) {
            r = 0.5 * (r + 1.0);
            if (cnt++ > 100) {
                std::cout << "math_utils::RootFinder stuck in inf-loop" << std::endl;
                return std::numeric_limits<double>::infinity();
            }
        }
        std::set<double> candidates = math_utils::RootFinder::solvePolynomial(dcoeff, l, r,
                                                                              FLT_EPSILON / duration);
        candidates.insert(0.0);
        candidates.insert(1.0);
        double maxVelRateSqr = -INFINITY;
        double tempNormSqr;
        for (std::set<double>::const_iterator it = candidates.begin();
             it != candidates.end();
             it++) {
            if (0.0 <= *it && 1.0 >= *it) {
                tempNormSqr = getVel((*it) * duration).squaredNorm();
                if (std::isfinite(tempNormSqr)) {
                    maxVelRateSqr = maxVelRateSqr < tempNormSqr ? tempNormSqr : maxVelRateSqr;
                }
            }
        }
        if (!std::isfinite(maxVelRateSqr) || maxVelRateSqr < 0.0) {
            return std::numeric_limits<double>::infinity();
        }
        return sqrt(maxVelRateSqr);
    }
}

double Piece::getMaxAccRate() const {
    if (!isValidPieceData(duration, D, coeffMat)) {
        return std::numeric_limits<double>::infinity();
    }
    Eigen::MatrixXd nAccCoeffMat = normalizeAccCoeffMat();
    if (!nAccCoeffMat.allFinite()) {
        return std::numeric_limits<double>::infinity();
    }
    Eigen::VectorXd coeff = math_utils::RootFinder::polySqr(nAccCoeffMat.row(0)) +
                            math_utils::RootFinder::polySqr(nAccCoeffMat.row(1)) +
                            math_utils::RootFinder::polySqr(nAccCoeffMat.row(2));
    if (!coeff.allFinite()) {
        return std::numeric_limits<double>::infinity();
    }
    int N = coeff.size();
    if (N <= 1) {
        return std::max(getAcc(0.0).norm(), getAcc(duration).norm());
    }
    int n = N - 1;
    for (int i = 0; i < N; i++) {
        coeff(i) *= n;
        n--;
    }
    Eigen::VectorXd dcoeff = trimLeadingZeros(coeff.head(N - 1));
    if (dcoeff.size() <= 1 || dcoeff.squaredNorm() < DBL_EPSILON) {
        return std::max(getAcc(0.0).norm(), getAcc(duration).norm());
    } else {
        double l = -0.0625;
        double r = 1.0625;
        int cnt = 0;
        while (fabs(math_utils::RootFinder::polyVal(dcoeff, l)) < DBL_EPSILON) {
            l = 0.5 * l;
            if (cnt++ > 100) {
                std::cout << "math_utils::RootFinder stuck in inf-loop" << std::endl;
                return std::numeric_limits<double>::infinity();
            }
        }
        cnt = 0;
        while (fabs(math_utils::RootFinder::polyVal(dcoeff, r)) < DBL_EPSILON) {
            r = 0.5 * (r + 1.0);
            if (cnt++ > 100) {
                std::cout << "math_utils::RootFinder stuck in inf-loop" << std::endl;
                return std::numeric_limits<double>::infinity();
            }
        }
        std::set<double> candidates = math_utils::RootFinder::solvePolynomial(dcoeff, l, r,
                                                                              FLT_EPSILON / duration);
        candidates.insert(0.0);
        candidates.insert(1.0);
        double maxAccRateSqr = -INFINITY;
        double tempNormSqr;
        for (std::set<double>::const_iterator it = candidates.begin();
             it != candidates.end();
             it++) {
            if (0.0 <= *it && 1.0 >= *it) {
                tempNormSqr = getAcc((*it) * duration).squaredNorm();
                if (std::isfinite(tempNormSqr)) {
                    maxAccRateSqr = maxAccRateSqr < tempNormSqr ? tempNormSqr : maxAccRateSqr;
                }
            }
        }
        if (!std::isfinite(maxAccRateSqr) || maxAccRateSqr < 0.0) {
            return std::numeric_limits<double>::infinity();
        }
        return sqrt(maxAccRateSqr);
    }
}

Eigen::MatrixXd Piece::getState(const double &t) const {
    MatDf out_mat;
    out_mat.resize(3, 4);
    out_mat << getPos(t), getVel(t), getAcc(t), getJer(t);
    return out_mat;
}

bool Piece::checkMaxVelRate(const double &maxVelRate) const {
    double sqrMaxVelRate = maxVelRate * maxVelRate;
    if (getVel(0.0).squaredNorm() >= sqrMaxVelRate ||
        getVel(duration).squaredNorm() >= sqrMaxVelRate) {
        return false;
    } else {
        Eigen::MatrixXd nVelCoeffMat = normalizeVelCoeffMat();
        Eigen::VectorXd coeff = math_utils::RootFinder::polySqr(nVelCoeffMat.row(0)) +
                                math_utils::RootFinder::polySqr(nVelCoeffMat.row(1)) +
                                math_utils::RootFinder::polySqr(nVelCoeffMat.row(2));
        double t2 = duration * duration;
        coeff.tail<1>()(0) -= sqrMaxVelRate * t2;
        return math_utils::RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
    }
}

bool Piece::checkMaxAccRate(const double &maxAccRate) const {
    double sqrMaxAccRate = maxAccRate * maxAccRate;
    if (getAcc(0.0).squaredNorm() >= sqrMaxAccRate ||
        getAcc(duration).squaredNorm() >= sqrMaxAccRate) {
        return false;
    } else {
        Eigen::MatrixXd nAccCoeffMat = normalizeAccCoeffMat();
        Eigen::VectorXd coeff = math_utils::RootFinder::polySqr(nAccCoeffMat.row(0)) +
                                math_utils::RootFinder::polySqr(nAccCoeffMat.row(1)) +
                                math_utils::RootFinder::polySqr(nAccCoeffMat.row(2));
        double t2 = duration * duration;
        double t4 = t2 * t2;
        coeff.tail<1>()(0) -= sqrMaxAccRate * t4;
        return math_utils::RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
    }
}
