//
// Created by 小乌嘎 on 2023/6/9.
//

#include "ellipsoid.h"

namespace QuadFit {
    Eigen::Vector3d Ellipsoid::project(const Eigen::Vector3d& p) {

    }

    double Ellipsoid::signed_distance(const Eigen::Vector3d& p) {

    }

    Eigen::Vector3d Ellipsoid::normal(const Eigen::Vector3d& p) {

    }

    void Ellipsoid::update() {

    }

    void Ellipsoid::set_rotation(const Eigen::Quaterniond& qua) {
        qua_ = qua;
        flag_ = UpdateFlag::MATRIXOUTDATED;
    }

    void Ellipsoid::set_center(const Eigen::Vector3d& center) {
        center_ = center;
        flag_ = UpdateFlag::MATRIXOUTDATED;
    }

    void Ellipsoid::set_axis_length(const Eigen::Vector3d& axis_length) {
        axis_length_ = axis_length;
        flag_ = UpdateFlag::MATRIXOUTDATED;
    }

    const Eigen::Quaterniond& Ellipsoid::get_rotation() {
        return qua_;
    }

    const Eigen::Vector3d Ellipsoid::get_center() {
        return center_;
    }

    const Eigen::Vector3d Ellipsoid::get_axis_length() {
        return axis_length_;
    }
}