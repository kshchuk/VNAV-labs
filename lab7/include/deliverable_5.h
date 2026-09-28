/*
 * @file gtsam.h
 * @brief Include file for gtsam.cpp
 * Simultaneous Localization and Mapping.
 * Feel free to use a header file to organize your solution.
 */
#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace gtsam {

// TODO: Define your Frobenius Norm Factor
// Insert code below:

    class FrobeniusNormFactor : public NoiseModelFactor1<Rot3> {
    private:
        Rot3 measured_;
    public:
        FrobeniusNormFactor(Key key, const Rot3& measured, const SharedNoiseModel& model)
        : NoiseModelFactor1<Rot3>(model, key), measured_(measured) {}
        Vector evaluateError(const Rot3& R, boost::optional<Matrix&> H = boost::none) const
        override {
            const Matrix3 Rm = R.matrix();
            if (H) {
                // Rot3 retraction: R * Exp(w) ~ R (I + [w]x), so d vec(R Exp(w)) / dw_k
                // = vec(R [e_k]x). Columns of R [e_k]x: [0, r3, -r2], [-r3, 0, r1], [r2, -r1, 0].
                const Vector3 r1 = Rm.col(0), r2 = Rm.col(1), r3 = Rm.col(2);
                Matrix J = Matrix::Zero(9, 3);
                J.block<3, 1>(3, 0) = r3;
                J.block<3, 1>(6, 0) = -r2;
                J.block<3, 1>(0, 1) = -r3;
                J.block<3, 1>(6, 1) = r1;
                J.block<3, 1>(0, 2) = r2;
                J.block<3, 1>(3, 2) = -r1;
                *H = J;
            }
            const Matrix3 D = Rm - measured_.matrix();
            return Eigen::Map<const Vector9>(D.data());
        }
    };

}  // namespace gtsam
