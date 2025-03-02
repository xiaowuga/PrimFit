//
// Created by 小乌嘎 on 2023/6/9.
//

#ifndef QUADFIT_ELLIPSOID_H
#define QUADFIT_ELLIPSOID_H
#include <quadrics.h>

namespace QuadFit {
    class Ellipsoid : public Quadrics {
    public:
        Ellipsoid(const Eigen::Quaterniond& qua, const Eigen::Vector3d& center
                  , const Eigen::Vector3d& axis_length) : Quadrics(QuadricsType::ELLIPSOID)
                  , qua_(qua), center_(center), axis_length_(axis_length){
            flag_ = UpdateFlag::MATRIXOUTDATED;
        }

        Eigen::Vector3d project(const Eigen::Vector3d& p) override;

        double signed_distance(const Eigen::Vector3d& p) override;

        Eigen::Vector3d normal(const Eigen::Vector3d& p) override;

        void update() override;

        void set_rotation(const Eigen::Quaterniond& qua);

        void set_center(const Eigen::Vector3d& center);

        void set_axis_length(const Eigen::Vector3d& axis_length);

        const Eigen::Quaterniond& get_rotation();

        const Eigen::Vector3d get_center();

        const Eigen::Vector3d get_axis_length();

    private:
        Eigen::Quaterniond qua_;
        Eigen::Vector3d center_;
        Eigen::Vector3d axis_length_;

    };
}


#endif //QUADFIT_ELLIPSOID_H
