//
// Created by 小乌嘎 on 2023/6/7.
//

#ifndef QUADFIT_QUADRICS_H
#define QUADFIT_QUADRICS_H

#include <Eigen/Core>
#include <Eigen/Dense>

namespace PrimFit {
    enum class QuadricsType {
        UNKNOWN,
        PLANE,
        ELLIPTICCYLINDER,
        ELLIPTICCONE,
        ELLIPSOID,
        TORUS
    };
    enum class UpdateFlag{
        CONSISTENT,
        MATRIXOUTDATED,
        PARAOUTDATED
    };

    class Quadrics {
    public:
        Quadrics(QuadricsType type = QuadricsType::UNKNOWN) : type_(type),flag_(UpdateFlag::CONSISTENT) {}

        virtual Eigen::Vector3d project(const Eigen::Vector3d& p) = 0;

        virtual Eigen::Vector3d normal(const Eigen::Vector3d& p) = 0;

        virtual double signed_distance(const Eigen::Vector3d& p) = 0;

        virtual void update() = 0;

        double distance(const Eigen::Vector3d& p);

        QuadricsType get_type() const;

        void set_type(QuadricsType type);

        const Eigen::Matrix4d& get_matrix();

        void set_matrix(const Eigen::Matrix4d& A);


    protected:
        Eigen::Matrix4d A_;
        QuadricsType type_;
        UpdateFlag flag_;
    };
}


#endif //QUADFIT_QUADRICS_H
