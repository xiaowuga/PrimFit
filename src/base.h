//
// Created by 小乌嘎 on 2023/8/31.
//


#ifndef PRIMFIT_BASE_H
#define PRIMFIT_BASE_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>

namespace PrimFit {
    struct Mesh {
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
    };

    const Mesh bbox_mesh(const Eigen::Vector3d& minn,
                          const Eigen::Vector3d& maxx,
                          double eps = 0.01,
                          int div = 5) {
        double len = (maxx - minn).norm() * eps;
        double xmin = minn.x() - len;
        double xmax = maxx.x() + len;
        double xstep = (xmax - xmin) / div;

        double ymin = minn.y() - len;
        double ymax = maxx.y() + len;
        double ystep = (ymax - ymin) / div;

        double zmin = minn.x() - len;
        double zmax = maxx.y() + len;
        double zstep = (zmax - zmin) / div;

        std::vector<Eigen::Vector3d> points;
        for(int i = 0; i <= div; i++) {
            for(int j = 0; j <= div; j++) {
                for(int k = 0; k <= div; k++) {
                    if(i == 0 || i == div || j == 0 || j == div || k == 0 || k == div)
                        points.emplace_back(Eigen::Vector3d(xmin + i * xstep, ymin + j * ystep, zmin + k * zstep));
                }
            }
        }

        Mesh bbox;
        bbox.V.resize(points.size(), 3);

        std::cout << points.size() << std::endl;

        for(size_t i = 0; i < points.size(); i++) {
            bbox.V(i,0) = points[i].x();
            bbox.V(i,1) = points[i].y();
            bbox.V(i,2) = points[i].z();
        }

        return bbox;

    }


}
#endif //PRIMFIT_BASE_H
