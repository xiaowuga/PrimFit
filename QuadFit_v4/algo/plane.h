//
// Created by 小乌嘎 on 2023/6/8.
//

#ifndef QUADFIT_PLANE_H
#define QUADFIT_PLANE_H

#include <quadrics.h>

namespace QuadFit {

    class Plane : public Quadrics {
    public:
        Plane(const Eigen::Vector3d& point
              , const Eigen::Vector3d& normal) : Quadrics(QuadricsType::PLANE)
        , m_pos_(point),m_normal_(normal.normalized()) {
            m_d_ = -m_normal_.dot(m_pos_);
            flag_ = UpdateFlag::MATRIXOUTDATED;
        }
        Plane(double distance_to_origin, const Eigen::Vector3d& normal) : m_d_(distance_to_origin)
        , Quadrics(QuadricsType::PLANE), m_normal_(normal.normalized()) {
            m_pos_ = -m_d_ * m_normal_;
            flag_ = UpdateFlag::MATRIXOUTDATED;
        }

        Eigen::Vector3d project(const Eigen::Vector3d& p) override;

        double signed_distance(const Eigen::Vector3d& p) override;

        Eigen::Vector3d normal(const Eigen::Vector3d& p) override;

        void update() override;

        void set_m_pos(const Eigen::Vector3d& pos);

        void set_m_normal(const Eigen::Vector3d& normal);

        void set_m_d(double d);

        const Eigen::Vector3d& get_m_pos();

        const Eigen::Vector3d& get_m_normal();

        double get_m_d();
    private:
        Eigen::Vector3d m_pos_, m_normal_;
        double m_d_;
    };
}


#endif //QUADFIT_PLANE_H
