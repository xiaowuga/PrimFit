//
// Created by 小乌嘎 on 2023/6/7.
//

#include "quadrics.h"

namespace PrimFit {
    double Quadrics::distance(const Eigen::Vector3d &p) {
        return abs(signed_distance(p));
    }

    QuadricsType Quadrics::get_type() const {
        return type_;
    }

    void Quadrics::set_type(QuadricsType type) {
        type_ = type;
    }

    const Eigen::Matrix4d& Quadrics::get_matrix() {

        if(flag_ == UpdateFlag::MATRIXOUTDATED)
            update();

        return A_;
    }

    void Quadrics::set_matrix(const Eigen::Matrix4d& A) {
        A_ = A;
        flag_ = UpdateFlag::PARAOUTDATED;
    }
}