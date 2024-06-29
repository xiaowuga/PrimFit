//
// Created by 小乌嘎 on 2023/8/31.
//


#ifndef PRIMFIT_BASE_H
#define PRIMFIT_BASE_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <iostream>
#include <map>

namespace PrimFit {
    struct Mesh {
        Eigen::MatrixXd V;
        Eigen::MatrixXi F;
    };

    Mesh bbox_mesh(const Eigen::Vector3d& minn,
                          const Eigen::Vector3d& maxx,
                          int div = 10,
                          double eps = 0.01) {
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

        std::map<int, std::map<int, std::map<int, int>>> indMap;
        std::vector<Eigen::Vector3d> points;
        for(int i = 0; i <= div; i++) {
            for(int j = 0; j <= div; j++) {
                for(int k = 0; k <= div; k++) {
                    if(i == 0 || i == div || j == 0 || j == div || k == 0 || k == div) {
                        points.emplace_back(Eigen::Vector3d(xmin + i * xstep, ymin + j * ystep, zmin + k * zstep));
                        indMap[i][j][k] = points.size() - 1;
                    }
                }
            }
        }

        Mesh bbox;
        bbox.V.resize(points.size(), 3);

        for(size_t i = 0; i < points.size(); i++) {
            bbox.V(i,0) = points[i].x();
            bbox.V(i,1) = points[i].y();
            bbox.V(i,2) = points[i].z();
        }

        std::vector<Eigen::Vector3i> faces;
        int ct = 0;

        for(int i = 0; i < div; i++) {
            for(int j = 0; j < div; j++) {
                int a = indMap[i][j][0];
                int b = indMap[i + 1][j][0];
                int c = indMap[i + 1][j + 1][0];
                int d = indMap[i][j + 1][0];

                faces.emplace_back(Eigen::Vector3i(c, b, a));
                faces.emplace_back(Eigen::Vector3i(d, c, a));

            }
        }

        for(int i = 0; i < div; i++) {
            for(int j = 0; j < div; j++) {
                int a = indMap[i][j][div];
                int b = indMap[i + 1][j][div];
                int c = indMap[i + 1][j + 1][div];
                int d = indMap[i][j + 1][div];

                faces.emplace_back(Eigen::Vector3i(a, b, c));
                faces.emplace_back(Eigen::Vector3i(a, c, d));

            }
        }

        for(int i = 0; i < div; i++) {
            for(int j = 0; j < div; j++) {
                int a = indMap[i][0][j];
                int b = indMap[i + 1][0][j];
                int c = indMap[i + 1][0][j + 1];
                int d = indMap[i][0][j + 1];

                faces.emplace_back(Eigen::Vector3i(a, b, c));
                faces.emplace_back(Eigen::Vector3i(a, c, d));

            }
        }

        for(int i = 0; i < div; i++) {
            for(int j = 0; j < div; j++) {
                int a = indMap[i][div][j];
                int b = indMap[i + 1][div][j];
                int c = indMap[i + 1][div][j + 1];
                int d = indMap[i][div][j + 1];

                faces.emplace_back(Eigen::Vector3i(c, b, a));
                faces.emplace_back(Eigen::Vector3i(d, c, a));

            }
        }

        for(int i = 0; i < div; i++) {
            for(int j = 0; j < div; j++) {
                int a = indMap[0][i][j];
                int b = indMap[0][i + 1][j];
                int c = indMap[0][i + 1][j + 1];
                int d = indMap[0][i][j + 1];

                faces.emplace_back(Eigen::Vector3i(c, b, a));
                faces.emplace_back(Eigen::Vector3i(d, c, a));

            }
        }

        for(int i = 0; i < div; i++) {
            for(int j = 0; j < div; j++) {
                int a = indMap[div][i][j];
                int b = indMap[div][i + 1][j];
                int c = indMap[div][i + 1][j + 1];
                int d = indMap[div][i][j + 1];

                faces.emplace_back(Eigen::Vector3i(a, b, c));
                faces.emplace_back(Eigen::Vector3i(a, c, d));

            }
        }

        bbox.F.resize(faces.size(), 3);

        for(size_t i = 0; i < faces.size(); i++) {
            bbox.F(i, 0) = faces[i].x();
            bbox.F(i, 1) = faces[i].y();
            bbox.F(i, 2) = faces[i].z();
        }

        return bbox;
    }

    Mesh bbox_mesh(const Eigen::MatrixXd& V, int div = 10, double eps = 0.01) {
        Eigen::Vector3d minn = V.colwise().minCoeff();
        Eigen::Vector3d maxx = V.colwise().maxCoeff();
        std::cout << minn << std::endl;
        std::cout << maxx << std::endl;
        return bbox_mesh(minn, maxx, div, eps);
    }


    Mesh plane_mesh(const Eigen::Vector3d& pos, const Eigen::Vector3d& dir, double len, int div = 10) {
        Eigen::Vector3d aix1;
        if(std::abs(dir.x()) < std::abs(dir.y()) && std::abs(dir.x()) < std::abs(dir.z())) {
            aix1 = Eigen::Vector3d(1,0,0);
        } else if(std::abs(dir.y()) < std::abs(dir.z())) {
            aix1 = Eigen::Vector3d(0,1,0);
        } else {
            aix1 = Eigen::Vector3d(0,0,1);
        }

        aix1 = dir.cross(aix1).normalized();
        Eigen::Vector3d aix2 = dir.cross(aix1).normalized();

        Eigen::Vector3d minn = pos - aix1 * len - aix2 * len;
        Eigen::Vector3d maxx = pos + aix1 * len + aix2 * len;

        double step = len * 2 / div;

        std::vector<Eigen::Vector3d> points;
        for(int i = 0; i <= div; i++) {
            for(int j = 0; j <= div; j++) {
                points.emplace_back(minn + i * step * aix1 + j * step * aix2);
            }
        }

        Mesh plane;
        plane.V.resize(points.size(), 3);

        for(size_t i = 0; i < points.size(); i++) {
            plane.V(i,0) = points[i].x();
            plane.V(i,1) = points[i].y();
            plane.V(i,2) = points[i].z();
        }

        std::vector<Eigen::Vector3i> faces;

        for(int i = 0; i < div; i++) {
            for(int j = 0; j < div; j++) {
                int a = i * (div + 1) + j;
                int b = i * (div + 1) + j + 1;
                int c = (i + 1) * (div + 1) + j;
                int d = (i + 1) * (div + 1) + j + 1;
                faces.emplace_back(Eigen::Vector3i(a, b, d));
                faces.emplace_back(Eigen::Vector3i(a, d, c));
            }
        }

        plane.F.resize(faces.size(), 3);

        for(size_t i = 0; i < faces.size(); i++) {
            plane.F(i, 0) = faces[i].x();
            plane.F(i, 1) = faces[i].y();
            plane.F(i, 2) = faces[i].z();
        }

        return plane;
    }

}
#endif //PRIMFIT_BASE_H
