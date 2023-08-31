//
// Created by 小乌嘎 on 2023/8/31.
//
#include <Eigen/Core>
#include <Eigen/Dense>

#ifndef PRIMFIT_BASE_H
#define PRIMFIT_BASE_H
namespace PrimFit {
    typedef double FLOAT;
    typedef Eigen::Matrix<FLOAT, Eigen::Dynamic, 3> MatX3f;
    typedef Eigen::Matrix<int, Eigen::Dynamic, 3> MatX3i;
    typedef Eigen::Matrix<int, Eigen::Dynamic, 2> MatX2i;
    typedef Eigen::Matrix<int, 2, Eigen::Dynamic> RowMatX2i;
    typedef Eigen::Matrix<int, Eigen::Dynamic, 1> VecXi;
    typedef Eigen::Matrix<int, 1, Eigen::Dynamic> RowVecXi;
    typedef Eigen::Matrix<FLOAT, Eigen::Dynamic, 1> VecXf;
    typedef Eigen::Matrix<FLOAT, 1, Eigen::Dynamic> RowVecXf;
    typedef Eigen::Matrix<FLOAT, 3, 1> Vec3f;

    struct PF_Mesh {
        MatX3f V;
        MatX3i F;
    };

    class BBox {
    public:
        BBox() = default;
        ~BBox() = default;

        Vec3f bbox_min;
        Vec3f bbox_max;

        double get_bbox_area() const {
            double dx = fabs(bbox_max.x() - bbox_min.x());
            double dy = fabs(bbox_max.y() - bbox_min.y());
            double dz = fabs(bbox_max.z() - bbox_min.z());
            return 2 * (dx*dy + dx*dz + dy*dz);
        }
    };
}
#endif //PRIMFIT_BASE_H
