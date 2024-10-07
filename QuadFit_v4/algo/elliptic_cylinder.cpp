//
// Created by 小乌嘎 on 2023/6/9.
//

#include "elliptic_cylinder.h"

namespace QuadFit {
    Eigen::Vector3d EllipticCylinder::project(const Eigen::Vector3d& p) {

    }

    double EllipticCylinder::signed_distance(const Eigen::Vector3d& p) {

    }

    Eigen::Vector3d EllipticCylinder::normal(const Eigen::Vector3d& p) {

    }

    void EllipticCylinder::update() {

    }

    void EllipticCylinder::set_rotation(const Eigen::Quaterniond& qua) {
        qua_ = qua;
        flag_ = UpdateFlag::MATRIXOUTDATED;
    }

    void EllipticCylinder::set_center(const Eigen::Vector3d& center) {
        center_ = center;
        flag_ = UpdateFlag::MATRIXOUTDATED;
    }

    void EllipticCylinder::set_axis_length(const Eigen::Vector2d& axis_length) {
        axis_length_ = axis_length;
        flag_ = UpdateFlag::MATRIXOUTDATED;
    }

    const Eigen::Quaterniond& EllipticCylinder::get_rotation() {
        return qua_;
    }

    const Eigen::Vector3d EllipticCylinder::get_center() {
        return center_;
    }

    const Eigen::Vector2d EllipticCylinder::get_axis_length() {
        return axis_length_;
    }
}

