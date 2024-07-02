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
//        std::cout << minn << std::endl;
//        std::cout << maxx << std::endl;
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

    Mesh cylinder_mesh(const Eigen::Vector3d& pos, const Eigen::Vector3d& dir, double radius, double len, int div1 = 30, int div2 = 10) {
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

        Eigen::Vector3d minn = pos - len * dir;

        double step = len * 2 / div2;

        std::vector<Eigen::Vector3d> points;

        for(int i = 0; i <= div2; i++) {
            Eigen::Vector3d p = minn + i * step * dir;
            for(int j = 0; j < div1; j++) {
                points.emplace_back(p + radius * cos(j * 2 * EIGEN_PI / div1) * aix1
                                      + radius * sin(j * 2 * EIGEN_PI / div1) * aix2);
            }
        }

        std::vector<Eigen::Vector3i> faces;

        for(int i = 0; i < div2; i++) {
            for(int j = 0; j < div1; j++) {
                int a = i * div1 + j;
                int b = i * div1 + (j + 1) % div1;
                int c = (i + 1) * div1 + j;
                int d = (i + 1) * div1 + (j + 1) % div1;
                faces.emplace_back(Eigen::Vector3i(a, b, d));
                faces.emplace_back(Eigen::Vector3i(a, d, c));
            }
        }

        Mesh cylinder;
        cylinder.V.resize(points.size(), 3);

        for(size_t i = 0; i < points.size(); i++) {
            cylinder.V(i,0) = points[i].x();
            cylinder.V(i,1) = points[i].y();
            cylinder.V(i,2) = points[i].z();
        }
        cylinder.F.resize(faces.size(), 3);

        for(size_t i = 0; i < faces.size(); i++) {
            cylinder.F(i, 0) = faces[i].x();
            cylinder.F(i, 1) = faces[i].y();
            cylinder.F(i, 2) = faces[i].z();
        }

        return cylinder;

    }

    Mesh cone_mesh(const Eigen::Vector3d& pos, const Eigen::Vector3d& dir, double angle, double len, int div1 = 30, int div2 = 10) {
        Eigen::Vector3d aix1;
        if(std::abs(dir.x()) < std::abs(dir.y()) && std::abs(dir.x()) < std::abs(dir.z())) {
            aix1 = Eigen::Vector3d(1,0,0);
        } else if(std::abs(dir.y()) < std::abs(dir.z())) {
            aix1 = Eigen::Vector3d(0,1,0);
        } else {
            aix1 = Eigen::Vector3d(0,0,1);
        }

        Eigen::Vector3d aix2 = dir.cross(aix1).normalized();

        std::vector<Eigen::Vector3d> points(div1 * div2 + 1);
        for(int i = 0; i < div2; i++) {
            Eigen::Vector3d center = pos + (1.0 * len * (i +  1) / div2) * dir;
            double r = std::abs(1.0 * len * (i + 1) / div2 * tan(angle));
            for(int j = 0; j < div1; j++) {
                points[i * div1 + j] = center + r * cos(-j * 2 * EIGEN_PI / div1) * aix1
                                              + r * sin(-j * 2 * EIGEN_PI / div1) * aix2;
            }
        }
        points[div1 * div2] = pos;

        std::vector<Eigen::Vector3i> faces;

        for(int i = 0; i < div1; i++) {
            faces.emplace_back(Eigen::Vector3i(div1 * div2, i,  (i +1) % div1));
        }

        for(int i = 0; i < div2 - 1; i++) {
            for(int j = 0; j < div1; j++) {
                faces.emplace_back(Eigen::Vector3i(j + div1 * i, j + div1 * (i + 1), (j + 1) % div1 + div1 * i));
                faces.emplace_back(Eigen::Vector3i((j + 1) % div1 + div1 * i, j + div1 * (i + 1), (j + 1) % div1 + div1 * (i + 1)));
            }
        }


        Mesh cone;
        cone.V.resize(points.size(), 3);

        for(size_t i = 0; i < points.size(); i++) {
            cone.V(i,0) = points[i].x();
            cone.V(i,1) = points[i].y();
            cone.V(i,2) = points[i].z();
        }
        cone.F.resize(faces.size(), 3);

        for(size_t i = 0; i < faces.size(); i++) {
            cone.F(i, 0) = faces[i].x();
            cone.F(i, 1) = faces[i].y();
            cone.F(i, 2) = faces[i].z();
        }

        return cone;
    }


    Mesh sphere_mesh(const Eigen::Vector3d& pos, double radius, int n_latitude = 30, int n_longitude = 30) {
        std::vector<Eigen::Vector3d> points((2*n_longitude-1)*n_latitude+2);

        double latitude_delta = 2 * EIGEN_PI / n_latitude;
        double longtitude_delta = EIGEN_PI / (2 * n_longitude);

        for (int i = 1; i < 2*n_longitude; ++i) {
            double longtitude_angle = -EIGEN_PI/2 + i * longtitude_delta;
            for (int j = 0; j < n_latitude; ++j) {
                double latitude_angle = j * latitude_delta;
                points[(i-1)*n_latitude + j] = pos + Eigen::Vector3d(cos(longtitude_angle) * cos(latitude_angle),
                                                    cos(longtitude_angle) * sin(latitude_angle),
                                                    sin(longtitude_angle)) * radius;
            }
        }

        int south_pole = (2*n_longitude-1)*n_latitude;
        points[south_pole] =  pos + Eigen::Vector3d(0, 0, -1) * radius;
        int north_pole = (2*n_longitude-1)*n_latitude + 1;
        points[north_pole] = pos + Eigen::Vector3d(0, 0, 1) * radius;


        std::vector<Eigen::Vector3i> faces;
        for (int i = 1; i < 2*n_longitude-1; ++i) {
            for (int j = 0; j < n_latitude; ++j) {
                int idx1 = 3 * ((i-1)*2*n_latitude + j);
                faces.emplace_back(Eigen::Vector3i((i-1)*n_latitude+j, (i-1)*n_latitude + (j+1)%n_latitude, i*n_latitude+j));
                faces.emplace_back(Eigen::Vector3i(i*n_latitude+j, (i-1)*n_latitude + (j+1)%n_latitude, i*n_latitude + (j+1)%n_latitude));
            }
        }

        for (int j = 0; j < n_latitude; ++j) {
            int idx = 3 * ((2*n_longitude-2)*2*n_latitude + j);
            faces.emplace_back(Eigen::Vector3i(j, south_pole, (j+1)%n_latitude));
        }

        for (int j = 0; j < n_latitude; ++j) {
            int idx = 3 * ((2*n_longitude-2)*2*n_latitude + n_latitude + j);
            faces.emplace_back(Eigen::Vector3i(north_pole, (2*n_longitude-2)*n_latitude + j, (2*n_longitude-2)*n_latitude + (j+1)%n_latitude));
        }

        Mesh sphere;
        sphere.V.resize(points.size(), 3);

        for(size_t i = 0; i < points.size(); i++) {
            sphere.V(i,0) = points[i].x();
            sphere.V(i,1) = points[i].y();
            sphere.V(i,2) = points[i].z();
        }
        sphere.F.resize(faces.size(), 3);

        for(size_t i = 0; i < faces.size(); i++) {
            sphere.F(i, 0) = faces[i].x();
            sphere.F(i, 1) = faces[i].y();
            sphere.F(i, 2) = faces[i].z();
        }

        return sphere;

    }

    Mesh torus_mesh(const Eigen::Vector3d& pos, const Eigen::Vector3d& dir, double major_radius, double minor_radius, int n_major = 60, int n_minor = 30) {

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
        std::vector<Eigen::Vector3d> points(n_major * n_minor);

        for(int i = 0; i < n_major; ++i) {
            Eigen::Vector3d m_center = pos + major_radius * cos(i*2* EIGEN_PI / n_major) * aix1
                                    + major_radius * sin(i * 2 * EIGEN_PI / n_major) * aix2;

            Eigen::Vector3d m_axis = (m_center - pos);
            m_axis.normalize();
            for(int j = 0; j < n_minor; ++j) {
                points[i * n_minor + j] = m_center
                                          + minor_radius * cos(j * 2 * EIGEN_PI / n_minor) * m_axis
                                          + minor_radius * sin(j * 2 * EIGEN_PI / n_minor) * dir;
            }
        }

        std::vector<Eigen::Vector3i> faces;
        for(int i = 0; i < n_major; ++i) {
            for(int j = 0; j < n_minor; ++j) {
                faces.emplace_back(Eigen::Vector3i(i*n_minor+((j+1)%n_minor), i*n_minor+j, ((i+1)%n_major)*n_minor+j));

                faces.emplace_back(Eigen::Vector3i(((i+1)%n_major)*n_minor+((j+1)%n_minor),i*n_minor+((j+1)%n_minor), ((i+1)%n_major)*n_minor+j));
            }
        }

        Mesh torus;
        torus.V.resize(points.size(), 3);

        for(size_t i = 0; i < points.size(); i++) {
            torus.V(i,0) = points[i].x();
            torus.V(i,1) = points[i].y();
            torus.V(i,2) = points[i].z();
        }
        torus.F.resize(faces.size(), 3);

        for(size_t i = 0; i < faces.size(); i++) {
            torus.F(i, 0) = faces[i].x();
            torus.F(i, 1) = faces[i].y();
            torus.F(i, 2) = faces[i].z();
        }

        return torus;
    }
}
#endif //PRIMFIT_BASE_H
