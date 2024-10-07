//
// Created by 小乌嘎 on 2022/11/21.
//

#include "mesh_generater.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/convex_hull_2.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <vector>
//#include <easy3d/core/quat.h>

using namespace easy3d;

namespace QuadFit {
    typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
    typedef K::Point_3 Point3;

    void Mesh_Generater::set_bbox(const easy3d::Box3 &bbox_) {
        bbox = bbox_;
        double len = bbox.diagonal_length() * 0.3;
        float xmin = bbox.min_coord(0) - len;
        float xmax = bbox.max_coord(0) + len;
        float ymin = bbox.min_coord(1) - len;
        float ymax = bbox.max_coord(1) + len;
        float zmin = bbox.min_coord(2) - len;
        float zmax = bbox.max_coord(2) + len;
        bbox_points.clear();
        bbox_points = {
                easy3d::vec3(xmin, ymin, zmax), easy3d::vec3(xmax, ymin, zmax),
                easy3d::vec3(xmin, ymax, zmax), easy3d::vec3(xmax, ymax, zmax),
                easy3d::vec3(xmin, ymin, zmin), easy3d::vec3(xmax, ymin, zmin),
                easy3d::vec3(xmin, ymax, zmin), easy3d::vec3(xmax, ymax, zmin)
        };
        bbox_indices.clear();
        bbox_indices = {
                0, 1, 2, 3, 4, 5, 6, 7,
                0, 2, 4, 6, 1, 3, 5, 7,
                0, 4, 2, 6, 1, 5, 3, 7
        };
    }


    bool Mesh_Generater::generate_proxy_mesh(std::vector<SurfacePrimitive *> &shapes,
                                             std::vector<std::vector<easy3d::vec3>> &proxy_points,
                                             std::vector<std::vector<int>> &proxy_indices) {

        int num = shapes.size();
        proxy_points.resize(num);
        proxy_indices.resize(num);
        for(int i = 0; i < num; i++) {
            QuadFit::SurfaceType type = shapes[i]->getType();
            if(type == QuadFit::SurfaceType::PLANE) {
                generate_plane_mesh(shapes[i], proxy_points[i], proxy_indices[i]);
            } else if(type == QuadFit::SurfaceType::CYLINDER) {
                QuadFit::SurfaceParameters para = shapes[i]->getParameters();
                vec3 p = para.pos;
                vec3 d = para.dir;
                double r = para.r1;
                std::vector<QuadFit::SurfacePrimitive*> tangent_planes;
//                std::cout << "cylinder"<<i <<std::endl;
                for(size_t j = 0; j < shapes.size(); j++) {

                    QuadFit::SurfaceType tmp = shapes[j]->getType();
                    if(tmp == QuadFit::SurfaceType::PLANE) {
                        double dis = fabs(shapes[j]->distance(p) - r);
                        vec3 d1 = (shapes[j]->project(p) - p).normalize();
                        vec3 d2 = shapes[j]->getParameters().dir;
                        double angle  = fabs(dot(d1,d));
                        SurfaceParameters pp = shapes[j]->getParameters();
                        if(dis < dis_eps && angle < 0.02) {
                            tangent_planes.emplace_back(shapes[j]);
                        }
                    }
                }
//                if(i == 4) {
//                    generate_cylinder_mesh(shapes[i], tangent_planes, proxy_points[i], proxy_indices[i], 36, 5);
//
//                } else {
                    generate_cylinder_mesh(shapes[i], tangent_planes, proxy_points[i], proxy_indices[i], 36, 5);
//                }
            } else if(type == QuadFit::SurfaceType::CONE) {
                std::vector<QuadFit::SurfacePrimitive*> tangent_planes;
                generate_cone_mesh(shapes[i], tangent_planes, proxy_points[i], proxy_indices[i]);
            } else if(type == QuadFit::SurfaceType::SPHERE) {
                std::vector<QuadFit::SurfacePrimitive*> tangent_planes;
//                if(i == 31) {
//                    generate_sphere_mesh(shapes[i], tangent_planes, proxy_points[i], proxy_indices[i], 36, 144);
//                } else {
                    generate_sphere_mesh(shapes[i], tangent_planes, proxy_points[i], proxy_indices[i]);
//                }
            } else if(type == QuadFit::SurfaceType::TORUS) {
                std::vector<QuadFit::SurfacePrimitive*> tangent_planes;
                generate_torus_mesh(shapes[i], tangent_planes,proxy_points[i], proxy_indices[i]);
            }
        }
        return true;
    }


    bool Mesh_Generater::generate_plane_mesh(SurfacePrimitive* plane,
                                             std::vector<easy3d::vec3> &points,
                                             std::vector<int> &indices) {
        points.clear();
        indices.clear();
        std::vector<vec3> pts;
        SurfaceParameters para = plane->getParameters();
        vec3 p = para.pos, n = para.dir;
        for (int j = 0; j < bbox_indices.size(); j += 2) {
            const vec3 &s = bbox_points[bbox_indices[j]];
            const vec3 &t = bbox_points[bbox_indices[j + 1]];
            int ss = plane_orient(p, n, s);
            int st = plane_orient(p, n, t);
            if ((ss == 1 && st == -1) || (ss == -1 && st == 1)) {
                vec3 q;
                if (line_plane_intersection(p, n, s, t, q)) {
                    pts.emplace_back(q);
                } else
                    std::cout << "fatal error. Should have intersection" << std::endl;
            } else {
                if (ss == 0) {
                    pts.emplace_back(s);
                } else if (st == 0) {
                    pts.emplace_back(t);
                } else {
                    // no intersection with the plane
                }
            }
        }

        if (pts.size() >= 3) {

            std::list<Point3> tmp;
            for (std::size_t j = 0; j < pts.size(); ++j) {
                vec2 q = plane_to_2d(p, n, pts[j]);
                tmp.emplace_back(Point3(q.x, q.y, double(j))); // trick: put the point index as the 'z' component
            }

            typedef CGAL::Projection_traits_xy_3<K> Projection;

            std::list<Point3> hull;
            CGAL::convex_hull_2(tmp.begin(), tmp.end(), std::back_inserter(hull), Projection());
            if (hull.size() > 3) {
                for (std::list<Point3>::iterator it = hull.begin(); it != hull.end(); ++it) {
                    int idx = int(it->z());
                    points.emplace_back(pts[idx]);
                }
                for (std::size_t j = 1; j < points.size() - 1; j++) {
                    indices.emplace_back(0);
                    indices.emplace_back(j);
                    indices.emplace_back(j + 1);
                }
            } else {
                std::cout << "fatal error. Check if this is a degenerate case" << std::endl;
            }
        } else {
            std::cout << "fatal error. Check if this is a degenerate case" << std::endl;
        }

        return true;
    }

    bool Mesh_Generater::generate_cylinder_mesh(SurfacePrimitive* cylinder,
                                                std::vector<SurfacePrimitive*>& tangent_planes,
                                                std::vector<easy3d::vec3> &points,
                                                std::vector<int> &indices,
                                                int n1, int n2) {
        SurfaceParameters para = cylinder->getParameters();
        double len = bbox.diagonal_length();
        double eps = len * 0.03;
        float xmin = bbox.min_coord(0) - eps;
        float xmax = bbox.max_coord(0) + eps;
        float ymin = bbox.min_coord(1) - eps;
        float ymax = bbox.max_coord(1) + eps;
        float zmin = bbox.min_coord(2) - eps;
        float zmax = bbox.max_coord(2) + eps;
        const vec3 &ap = para.pos;
        const vec3 &auv = para.dir;
        vec3 base1, base2;
        compute_bases(auv, base1, base2);
        const double radius = para.r1;

        vec3 p1, p2;
        double l = 0, r = len;
        while (r - l > 0.01) {
            double m = (l + r) / 2;
            vec3 tp1 = ap + m * auv;
            std::vector<vec3> tmp;
            tmp.emplace_back(tp1 + base1 * radius);
            tmp.emplace_back(tp1 + base2 * radius);
            tmp.emplace_back(tp1 - base1 * radius);
            tmp.emplace_back(tp1 - base2 * radius);
            bool flag = true;
            for (std::size_t j = 0; j < tmp.size(); j++) {
                if (tmp[j].x >= xmin && tmp[j].x <= xmax
                    && tmp[j].y >= ymin && tmp[j].y <= ymax
                    && tmp[j].z >= zmin && tmp[j].z <= zmax) {
                    flag = false;
                    break;
                }
            }
            if (flag) {
                r = m;
            } else {
                l = m;
                p1 = tp1;
            }
        }

        l = 0, r = len;
        while (r - l > 0.01) {
            double m = (l + r) / 2;
            vec3 tp2 = ap - m * auv;
            std::vector<vec3> tmp;
            tmp.emplace_back(tp2 + base1 * radius);
            tmp.emplace_back(tp2 + base2 * radius);
            tmp.emplace_back(tp2 - base1 * radius);
            tmp.emplace_back(tp2 - base2 * radius);
            bool flag = true;
            for (std::size_t j = 0; j < tmp.size(); j++) {
                if (tmp[j].x >= xmin && tmp[j].x <= xmax
                    && tmp[j].y >= ymin && tmp[j].y <= ymax
                    && tmp[j].z >= zmin && tmp[j].z <= zmax) {
                    flag = false;
                    break;
                }
            }
            if (flag) {
                r = m;

            } else {
                l = m;
                p2 = tp2;
            }
        }

        double h = (p1 - p2).norm();
//        std::cout << h << std::endl;
//        std::vector<int> indice;
        points.resize(n1 * (n2 + 1));
        for (int j = 0; j < n1; ++j) {
            points[j] = vec3(p2 + radius * cos(j * 2 * M_PI / n1) * base1
                                + radius * sin(j * 2 * M_PI / n1) * base2);

            for(int k = 1; k <= n2; k++) {
                points[j + k * n1] = points[j] + 1.0 * k * h / n2 * auv;
            }
        }

//        for (int j = 0; j < n1; ++j) {
//            points.emplace_back(points[j] + h * auv);
//        }
        for (std::size_t i = 0; i < tangent_planes.size(); i++) {
            SurfacePrimitive* plane = tangent_planes[i];

            vec3 d1 = (plane->project(p2) - p2).normalize();
            for (int j = 0; j < n1; j++) {
                double offset = 0;
                vec3 d2 = (points[j] - p2).normalize();
                double dis = plane->distance(points[j]);
                double angle = dot(d1, d2);
                if(dis < dis_eps) {
                    offset = dis_eps;
//                    std::cout << j << std::endl;
                }
                if(offset >  dis_eps) {
                    points[j] = p2 + (radius + offset) * cos(j * 2 * M_PI / n1) * base1
                                + (radius + offset) * sin(j * 2 * M_PI / n1) * base2;
                    for (int k = 1; k <= n2; k++) {
                        points[j + k * n1] = points[j] + 1.0 * k * h / n2 * auv;
                    }
                }
            }

        }

        for(int i = 0; i < n2; i++) {
            for(int j = 0; j < n1; j++) {
                indices.emplace_back(j + i * n1);
                indices.emplace_back((j + 1) % n1 + i * n1);
                indices.emplace_back(j + (i + 1) * n1);
                indices.emplace_back(j + (i + 1) * n1);
                indices.emplace_back((j + 1) % n1 + i * n1);
                indices.emplace_back((j + 1) % n1 + (i + 1) * n1);
            }
        }

        return true;
    }

    bool Mesh_Generater::generate_cone_mesh(SurfacePrimitive *cone, std::vector<SurfacePrimitive *> &tangent_plane,
                                            std::vector<easy3d::vec3> &points, std::vector<int> &indices,
                                            int n1, int n2) {
        SurfaceParameters para = cone->getParameters();
        vec3 auv = para.dir;
        vec3 apex = para.pos;
        float angle = para.r1;
        double len = bbox.diagonal_length();

        if(!bbox.contains(apex)) {
            std::cout << "ASD" <<std::endl;
        }
        double eps = len * 0.03;
        float xmin = bbox.min_coord(0) - eps;
        float xmax = bbox.max_coord(0) + eps;
        float ymin = bbox.min_coord(1) - eps;
        float ymax = bbox.max_coord(1) + eps;
        float zmin = bbox.min_coord(2) - eps;
        float zmax = bbox.max_coord(2) + eps;
        vec3 base1, base2;
        compute_bases(auv, base1, base2);

        vec3 base_center;

        double l = 0, r = len;
        while (r - l > 0.01) {
            double m = (l + r) / 2;
            vec3 tp1 = apex + m * auv;
            double radius = std::fabs(m * std::tan(angle));
            std::vector<vec3> tmp;
            tmp.emplace_back(tp1 + base1 * radius);
            tmp.emplace_back(tp1 + base2 * radius);
            tmp.emplace_back(tp1 - base1 * radius);
            tmp.emplace_back(tp1 - base2 * radius);
            bool flag = true;
            for (std::size_t j = 0; j < tmp.size(); j++) {
                if (tmp[j].x >= xmin && tmp[j].x <= xmax
                    && tmp[j].y >= ymin && tmp[j].y <= ymax
                    && tmp[j].z >= zmin && tmp[j].z <= zmax) {
                    flag = false;
                    break;
                }
            }
            if (flag) {
                r = m;
            } else {
                l = m;
                base_center = tp1;
            }
        }

        double h = (base_center - apex).norm();
//        h = len;

        points.resize(n1 * n2 + 1);

        for(int i = 0; i < n2; i++) {
            vec3 center = apex + (1.0 * h * (i + 1) / n2) * auv;
            float r = std::abs((1.0 * h * (i + 1) / n2) * tan(angle));
            for(int j = 0; j < n1; j++) {
                points[i * n1 + j] = center + r * cos(-j*2*M_PI/n1) * base1
                                            + r * sin(-j*2*M_PI/n1) * base2;
            }
        }
        points[n1 * n2] = apex;

        for(int i = 0; i < n1; i++) {
            indices.emplace_back(n1 * n2); indices.emplace_back(i); indices.emplace_back((i + 1) % n1);
//            indices.emplace_back(points[i]); indices.emplace_back(points[(i + 1) % n1 + n1);  indices.emplace_back(points[(i + 1) % n1]);
        }

        for(int i = 0; i < n2 - 1; i++) {
            for(int j = 0; j < n1 ; j++) {
                indices.emplace_back(j + n1 * i);
                indices.emplace_back(j + n1 * (i + 1));
                indices.emplace_back((j + 1) % n1 + n1 * i);

                indices.emplace_back((j + 1) % n1 + n1 * i);
                indices.emplace_back(j + n1 * (i + 1));
                indices.emplace_back((j + 1) % n1 + n1 * (i + 1));
            }
        }

        return true;
    }

    void Mesh_Generater::generate_unit_sphere(std::vector<easy3d::vec3> &points, std::vector<int> &indices,
                                              int n_latitude, int n_longitude) {
        points.resize((2*n_longitude-1)*n_latitude+2);

        double latitude_delta = 2 * M_PI / n_latitude;
        double longtitude_delta = M_PI / (2 * n_longitude);

        for (int i = 1; i < 2*n_longitude; ++i) {
            double longtitude_angle = -M_PI/2 + i * longtitude_delta;
            for (int j = 0; j < n_latitude; ++j) {
                double latitude_angle = j * latitude_delta;
                points[(i-1)*n_latitude + j] = vec3(cos(longtitude_angle) * cos(latitude_angle),
                                                    cos(longtitude_angle) * sin(latitude_angle),
                                                    sin(longtitude_angle));
            }
        }

        int south_pole = (2*n_longitude-1)*n_latitude;
        points[south_pole] =  vec3(0, 0, -1);
        int north_pole = (2*n_longitude-1)*n_latitude + 1;
        points[north_pole] = vec3(0, 0, 1);

        int num_tri = (2*n_longitude-1)*2*n_latitude;
        indices.resize(3 * num_tri);
        for (int i = 1; i < 2*n_longitude-1; ++i) {
            for (int j = 0; j < n_latitude; ++j) {
                int idx1 = 3 * ((i-1)*2*n_latitude + j);
                indices[idx1] = (i-1)*n_latitude+j;
                indices[idx1 + 1] = (i-1)*n_latitude + (j+1)%n_latitude;
                indices[idx1 + 2] = i*n_latitude+j;

                int idx2 = 3 * ((i-1)*2*n_latitude + n_latitude + j);
                indices[idx2] = i*n_latitude+j;
                indices[idx2 + 1] = (i-1)*n_latitude + (j+1)%n_latitude;
                indices[idx2 + 2] = i*n_latitude + (j+1)%n_latitude;

            }
        }

        for (int j = 0; j < n_latitude; ++j) {
            int idx = 3 * ((2*n_longitude-2)*2*n_latitude + j);
            indices[idx] = j;
            indices[idx + 1] = south_pole;
            indices[idx + 2] = (j+1)%n_latitude;
        }

        for (int j = 0; j < n_latitude; ++j) {
            int idx = 3 * ((2*n_longitude-2)*2*n_latitude + n_latitude + j);
            indices[idx] = north_pole;
            indices[idx + 1] = (2*n_longitude-2)*n_latitude + j;
            indices[idx + 2] = (2*n_longitude-2)*n_latitude + (j+1)%n_latitude;
        }
    }
    bool Mesh_Generater::generate_sphere_mesh(SurfacePrimitive *sphere, std::vector<SurfacePrimitive *> &tangent_plane,
                                              std::vector<easy3d::vec3> &points, std::vector<int> &indices,
                                              int n_latitude, int n_longitude) {
        generate_unit_sphere(points, indices, n_latitude, n_longitude);
        SurfaceParameters para = sphere->getParameters();
        vec3 center = para.pos;
        float radius = para.r1;

        for(size_t i = 0; i < points.size(); i++) {
            points[i] = vec3(points[i].x, -points[i].z, points[i].y);
            points[i] *= radius;
            points[i] += center;
        }
        return true;
    }

    bool Mesh_Generater::generate_torus_mesh(SurfacePrimitive *torus
                                             ,std::vector<SurfacePrimitive *> &tangent_plane
                                             ,std::vector<easy3d::vec3> &points
                                             ,std::vector<int>& indices
                                             ,int n_major
                                             ,int n_minor) {
        SurfaceParameters para = torus->getParameters();
        easy3d::vec3 center = para.pos;
        easy3d::vec3 auv = para.dir;
        double major_radius = para.r1;
        double minor_radius = para.r2;

        points.resize(n_major * n_minor);
        easy3d::vec3 base1, base2;
        compute_bases(auv, base1, base2);

        for(int i = 0; i < n_major; ++i) {
            easy3d::vec3 m_center = center + major_radius * cos(i*2*M_PI/n_major) * base1
                    + major_radius * sin(i*2*M_PI/n_major) * base2;

            easy3d::vec3 m_axis = (m_center - center);
            m_axis.normalize();
            for(int j = 0; j < n_minor; ++j) {
                points[i * n_minor + j] = m_center
                        + minor_radius * cos(j*2*M_PI/n_minor) * m_axis
                        + minor_radius * sin(j*2*M_PI/n_minor) * auv;
            }
        }

        indices.clear();
        for(int i = 0; i < n_major; ++i) {
            for(int j = 0; j < n_minor; ++j) {
                indices.emplace_back(((i+1)%n_major)*n_minor+j);
                indices.emplace_back(i*n_minor+j);
                indices.emplace_back(i*n_minor+((j+1)%n_minor));

                indices.emplace_back(((i+1)%n_major)*n_minor+j);
                indices.emplace_back(i*n_minor+((j+1)%n_minor));
                indices.emplace_back(((i+1)%n_major)*n_minor+((j+1)%n_minor));
            }
        }
        return true;
    }




}