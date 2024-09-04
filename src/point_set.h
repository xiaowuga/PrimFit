//
// Created by xiaowuga on 2024/9/4.
//

#ifndef POINTSET_H
#define POINTSET_H

#include <Eigen/Dense>
#include <vector>

namespace PrimFit {
    class PointSet {
    public:
        Eigen::MatrixXd points_;
        Eigen::MatrixXd colors_;
        Eigen::MatrixXd normals_;
        Eigen::Vector3d minn_;
        Eigen::Vector3d maxx_;
        std::vector<size_t> types_;
        std::vector<std::vector<double>> parameters_;
        std::vector<std::vector<int>> groups_;
    public:
        void load_vg(const std::string& file_name);
        void load_ply(const std::string& file_name);

    };
}



#endif //POINTSET_H
