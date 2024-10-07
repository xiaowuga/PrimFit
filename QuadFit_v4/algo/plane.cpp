//
// Created by 小乌嘎 on 2023/6/8.
//

#include "plane.h"

namespace QuadFit {
    Eigen::Vector3d Plane::project(const Eigen::Vector3d& p) {
        if(flag_ == UpdateFlag::PARAOUTDATED)
            update();
        return p - signed_distance(p) * m_normal_;
    }

    double Plane::signed_distance(const Eigen::Vector3d& p) {
        if(flag_ == UpdateFlag::PARAOUTDATED)
            update();
        return m_normal_.dot(p) + m_d_;
    }

    Eigen::Vector3d Plane::normal(const Eigen::Vector3d &p) {
        if(flag_ == UpdateFlag::PARAOUTDATED)
            update();
        return m_normal_;
    }


    void Plane::update() {
//        TODO: make matrix and parameters consistent

        flag_ = UpdateFlag::CONSISTENT;
    }

    void Plane::set_m_pos(const Eigen::Vector3d &pos) {
        m_pos_ = pos;
        flag_ = UpdateFlag::MATRIXOUTDATED;
    }

    void Plane::set_m_normal(const Eigen::Vector3d &normal)  {
        m_normal_ = normal;
        flag_ = UpdateFlag::MATRIXOUTDATED;
    }

    void Plane::set_m_d(double d) {
        m_d_ = d;
        flag_ = UpdateFlag::MATRIXOUTDATED;
    };

    const Eigen::Vector3d& Plane::get_m_pos() {
        if(flag_ == UpdateFlag::PARAOUTDATED)
            update();
        return m_pos_;
    }

    const Eigen::Vector3d& Plane::get_m_normal() {
        if(flag_ == UpdateFlag::PARAOUTDATED)
            update();
        return m_normal_;
    }

    double Plane::get_m_d() {
        if(flag_ == UpdateFlag::PARAOUTDATED)
            update();
        return m_d_;
    }
}
