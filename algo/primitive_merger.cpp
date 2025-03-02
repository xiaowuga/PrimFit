//
// Created by 小乌嘎 on 2022/12/2.
//

#include "primitive_merger.h"
#include <cmath>

namespace PrimFit {
    void Primitive_Merger::bcj_get(std::vector<std::vector<int>> &set) {
        std::map<int, std::set<int>> mp;
        for(int i = 0; i < bcj_.size(); i++) {
            int r = find_root(i);
            mp[r].insert(i);
        }
        set.clear();
        int count = 0;
        set.resize(mp.size());
        for(auto item : mp) {
            for(auto& jtem : item.second) {
                set[count].emplace_back(jtem);
            }
            count++;
        }

    }


    bool Primitive_Merger::merge_primitves(std::vector<SurfacePrimitive *> &shapes,
                                           std::vector<SurfacePrimitive *> &shapes_merged,
                                           std::vector<easy3d::vec3> &points, std::vector<easy3d::vec3> &normals,
                                           std::vector<std::vector<int>> &segments,
                                           std::vector<std::vector<int>> &segments_merged,
                                           std::vector<std::vector<int>> & merge_pair) {
        std::vector<int> plane_indices, cylinder_indices, cone_indices, sphere_indices, torus_indices;


        for(auto s : shapes_merged)
            delete s;

        shapes_merged.clear();

        segments_merged.clear();

        for(size_t i = 0; i < shapes.size(); i++) {
//            if(i == 9 || i == 16 || i == 24 || i == 26 || i == 27 || i == 31) {
//                continue;
//            }
            SurfaceType type = shapes[i]->getType();
            switch (type) {
                case SurfaceType::PLANE :
                    plane_indices.emplace_back(i); break;
                case SurfaceType::CYLINDER :
                    cylinder_indices.emplace_back(i); break;
                case SurfaceType::CONE :
                    cone_indices.emplace_back(i); break;
                case SurfaceType::SPHERE :
                    sphere_indices.emplace_back(i); break;
                case SurfaceType::TORUS :
                    torus_indices.emplace_back(i); break;
            }
        }

        merge_cylinder(cylinder_indices, shapes, shapes_merged, points, normals, segments, segments_merged);
        merge_cone(cone_indices, shapes, shapes_merged, points, normals, segments, segments_merged);
        merge_sphere(sphere_indices, shapes, shapes_merged, points, normals, segments, segments_merged);
        merge_torus(torus_indices, shapes, shapes_merged, points, normals, segments, segments_merged);
        merge_planes(plane_indices, shapes, shapes_merged, points, normals, segments, segments_merged);

        return true;
    }

    bool Primitive_Merger::merge_planes(std::vector<int> &plane_indices,
                                        std::vector<SurfacePrimitive *> &shapes,
                                        std::vector<SurfacePrimitive *> &shapes_merged,
                                        std::vector<easy3d::vec3>& points,
                                        std::vector<easy3d::vec3>& normals,
                                        std::vector<std::vector<int>> &segments,
                                        std::vector<std::vector<int>> &segments_merged) {

        int num = plane_indices.size();
        for(int i = 0; i < num; i++) {
            int idx = plane_indices[i];
            if(shapes[idx]->getType() != SurfaceType::PLANE) {
                return false;
            }
        }

        init_bcj(num);

        for(int i = 0; i < num; i++) {
            int idx_i = plane_indices[i];
            SurfaceParameters para_i = shapes[idx_i]->getParameters();
            const easy3d::vec3 &ni = para_i.dir;
            const double di = para_i.r1;
            for(int j = i + 1; j < num; j++) {
                int idx_j = plane_indices[j];
                SurfaceParameters para_j = shapes[idx_j]->getParameters();
                const easy3d::vec3 &nj = para_j.dir;
                const double dj = para_j.r1;
                int x = idx_i, y = idx_j;
                if(x > y) {
                    std::swap(x, y);
                }
                if (std::fabs(easy3d::dot(ni, nj)) >= normal_eps_) {
                    if (std::fabs(di - dj) < 0.003 && false) {
                        int ri = find_root( i);
                        int rj = find_root( j);
                        if (ri != rj) {
                            merge_bcj(ri, rj);
                        }
                    }
                }
            }
        }

        std::vector<std::vector<int>>bcj_set;
        bcj_get(bcj_set);

        for(size_t i = 0; i < bcj_set.size(); i++) {
            int sz = bcj_set[i].size();
            if(sz == 1) {
                int idx = plane_indices[bcj_set[i][0]];
                shapes_merged.emplace_back(PrimFit::clone_surface_primitive(shapes[idx]));
                segments_merged.emplace_back(segments[idx]);
            } else {
                segments_merged.emplace_back(std::vector<int>());
                std::vector<Point> pts;
                for(size_t j = 0; j < bcj_set[i].size(); j++) {
                    int seg_id = plane_indices[bcj_set[i][j]];
                    for(size_t k = 0; k < segments[seg_id].size(); k++) {
                        segments_merged.back().emplace_back(segments[seg_id][k]);
                        Point pp;
                        pp.p = points[segments[seg_id][k]];
                        pp.n = normals[segments[seg_id][k]];
                        pts.emplace_back(pp);
                    }
                }
                int idx = plane_indices[bcj_set[i][0]];
//                SurfacePrimitive* sp = fit_surface(pts, SurfaceType::PLANE);
                SurfacePrimitive* sp = clone_surface_primitive(shapes[idx]);
                sp->setColor(shapes[idx]->getColor());
                shapes_merged.emplace_back(sp);
            }
        }
        return true;
    }

    bool Primitive_Merger::merge_cylinder(std::vector<int> &cylinder_indices,
                                          std::vector<SurfacePrimitive *> &shapes,
                                          std::vector<SurfacePrimitive *> &shapes_merged,
                                          std::vector<easy3d::vec3> &points,
                                          std::vector<easy3d::vec3> &normals,
                                          std::vector<std::vector<int>> &segments,
                                          std::vector<std::vector<int>> &segments_merged) {
//        std::cout << normal_eps_ << std::endl;
        int num = cylinder_indices.size();
        for(int i = 0; i < num; i++) {
            int idx = cylinder_indices[i];
            if(shapes[idx]->getType() != SurfaceType::CYLINDER) {
                return false;
            }
        }

        init_bcj(num);
        for(int i = 0; i < num; i++) {
            int idx_i = cylinder_indices[i];
            SurfaceParameters para_i = shapes[idx_i]->getParameters();
            const easy3d::vec3 &ni = para_i.dir;
            const easy3d::vec3 &pi = para_i.pos;
            const double radi = para_i.r1;;
            for(int j = i + 1; j < num; j++) {
                int idx_j = cylinder_indices[j];
                SurfaceParameters para_j = shapes[idx_j]->getParameters();
                const easy3d::vec3 &nj = para_j.dir;
                const easy3d::vec3 &pj = para_j.pos;
                const double radj = para_j.r1;
                easy3d::vec3 tv = normalize(pj - pi);
                int x = idx_i, y = idx_j;
                if(x > y) {
                    std::swap(x, y);
                }
                if (std::fabs(easy3d::dot(ni, nj)) >= normal_eps_
                    && std::fabs(easy3d::dot(tv, ni)) >= normal_eps_
                    && std::fabs(radi - radj) < dis_eps_ && false) {
                        int ri = find_root( i);
                        int rj = find_root( j);
                        if (ri != rj) {
                            merge_bcj(ri, rj);
                        }
                    }
            }
        }


        std::vector<std::vector<int>>bcj_set;
        bcj_get(bcj_set);

        for(size_t i = 0; i < bcj_set.size(); i++) {
            int sz = bcj_set[i].size();
//            std::cout << "cylinder " << i << ' ' << sz << std::endl;
            if(sz == 1) {
                int idx = cylinder_indices[bcj_set[i][0]];
                shapes_merged.emplace_back(PrimFit::clone_surface_primitive(shapes[idx]));
                segments_merged.emplace_back(segments[idx]);
            } else {
                segments_merged.emplace_back(std::vector<int>());
                int idx = cylinder_indices[bcj_set[i][0]];
                std::vector<Point> pts;
                for(size_t j = 0; j < bcj_set[i].size(); j++) {
                    int seg_id = cylinder_indices[bcj_set[i][j]];
                    for(size_t k = 0; k < segments[seg_id].size(); k++) {
                        segments_merged.back().emplace_back(segments[seg_id][k]);
                        Point pp;
                        pp.p = points[segments[seg_id][k]];
                        pp.n = normals[segments[seg_id][k]];
                        pts.emplace_back(pp);
                    }
                }

//                SurfacePrimitive* sp = fit_surface(pts, SurfaceType::CYLINDER);
                SurfacePrimitive* sp = clone_surface_primitive(shapes[idx]);
                sp->setColor(shapes[idx]->getColor());
                shapes_merged.emplace_back(sp);
            }
        }
        return true;

    }

    bool Primitive_Merger::merge_cone(std::vector<int> &cone_indices,
                                      std::vector<SurfacePrimitive *> &shapes,
                                      std::vector<SurfacePrimitive *> &shapes_merged, std::vector<easy3d::vec3> &points,
                                      std::vector<easy3d::vec3> &normals,
                                      std::vector<std::vector<int>> &segments,
                                      std::vector<std::vector<int>> &segments_merged) {
        int num = cone_indices.size();
        for(int i = 0; i < num; i++) {
            int idx = cone_indices[i];
            if(shapes[idx]->getType() != SurfaceType::CONE) {
                return false;
            }
        }

        init_bcj(num);

        for(int i = 0; i < num; i++) {
            int idx_i = cone_indices[i];
            SurfaceParameters para_i = shapes[idx_i]->getParameters();
            const easy3d::vec3 &di = para_i.dir;
            const easy3d::vec3 &pi = para_i.pos;
            const float ang_i = para_i.r1;
            for(int j = i + 1; j < num; j++) {
                int idx_j = cone_indices[j];
                SurfaceParameters para_j = shapes[idx_j]->getParameters();
                const easy3d::vec3 &dj = para_j.dir;
                const easy3d::vec3 &pj = para_j.pos;
                const float ang_j = para_j.r1;
                if (easy3d::dot(di, dj) >= normal_eps_
                    && (pi - pj).norm() < dis_eps_
                    && cos(std::fabs(ang_i - ang_j)) >= normal_eps_) {
                        int ri = find_root( i);
                        int rj = find_root( j);
                        if (ri != rj) {
                            merge_bcj(ri, rj);
                        }
                }
            }
        }

        std::vector<std::vector<int>>bcj_set;
        bcj_get(bcj_set);

        for(size_t i = 0; i < bcj_set.size(); i++) {
            int sz = bcj_set[i].size();
            if(sz == 1) {
                int idx = cone_indices[bcj_set[i][0]];
                shapes_merged.emplace_back(PrimFit::clone_surface_primitive(shapes[idx]));
                segments_merged.emplace_back(segments[idx]);
            } else {
                segments_merged.emplace_back(std::vector<int>());
                int idx = cone_indices[bcj_set[i][0]];
                std::vector<Point> pts;
                for(size_t j = 0; j < bcj_set[i].size(); j++) {
                    int seg_id = cone_indices[bcj_set[i][j]];
                    for(size_t k = 0; k < segments[seg_id].size(); k++) {
                        segments_merged.back().emplace_back(segments[seg_id][k]);
                        Point pp;
                        pp.p = points[segments[seg_id][k]];
                        pp.n = normals[segments[seg_id][k]];
                        pts.emplace_back(pp);
                    }
                }

//                SurfacePrimitive* sp = fit_surface(pts, SurfaceType::CONE);
                SurfacePrimitive* sp = clone_surface_primitive(shapes[idx]);
                sp->setColor(shapes[idx]->getColor());
                shapes_merged.emplace_back(sp);
            }
        }
        return true;
    }

    bool Primitive_Merger::merge_sphere(std::vector<int> &sphere_indices,
                                        std::vector<SurfacePrimitive *> &shapes,
                                        std::vector<SurfacePrimitive *> &shapes_merged,
                                        std::vector<easy3d::vec3> &points, std::vector<easy3d::vec3> &normals,
                                        std::vector<std::vector<int>> &segments,
                                        std::vector<std::vector<int>> &segments_merged) {
        int num = sphere_indices.size();
        for(int i = 0; i < num; i++) {
            int idx = sphere_indices[i];
            if(shapes[idx]->getType() != SurfaceType::SPHERE) {
                return false;
            }
        }

        init_bcj(num);

        for(int i = 0; i < num; i++) {
            int idx_i = sphere_indices[i];
            SurfaceParameters para_i = shapes[idx_i]->getParameters();
            const easy3d::vec3 &pi = para_i.pos;
            const float rad_i = para_i.r1;
            for(int j = i + 1; j < num; j++) {
                int idx_j = sphere_indices[j];
                SurfaceParameters para_j = shapes[idx_j]->getParameters();
                const easy3d::vec3 &pj = para_j.pos;
                const float rad_j = para_j.r1;
                easy3d::vec3 tv = normalize(pj - pi);
                if ((pi-pj).norm() <= dis_eps_
                && std::fabs(rad_i - rad_j) <= dis_eps_) {
                    int ri = find_root( i);
                    int rj = find_root( j);
                    if (ri != rj) {
                        merge_bcj(ri, rj);
                    }
                }
            }
        }

        std::vector<std::vector<int>>bcj_set;
        bcj_get(bcj_set);

        for(size_t i = 0; i < bcj_set.size(); i++) {
            int sz = bcj_set[i].size();
            if(sz == 1) {
                int idx = sphere_indices[bcj_set[i][0]];
                shapes_merged.emplace_back(PrimFit::clone_surface_primitive(shapes[idx]));
                segments_merged.emplace_back(segments[idx]);
            } else {
                segments_merged.emplace_back(std::vector<int>());
                int idx = sphere_indices[bcj_set[i][0]];
                std::vector<Point> pts;
                for(size_t j = 0; j < bcj_set[i].size(); j++) {
                    int seg_id = sphere_indices[bcj_set[i][j]];
                    for(size_t k = 0; k < segments[seg_id].size(); k++) {
                        segments_merged.back().emplace_back(segments[seg_id][k]);
                        Point pp;
                        pp.p = points[segments[seg_id][k]];
                        pp.n = normals[segments[seg_id][k]];
                        pts.emplace_back(pp);
                    }
                }

//                SurfacePrimitive* sp = fit_surface(pts, SurfaceType::SPHERE);
                SurfacePrimitive* sp = clone_surface_primitive(shapes[idx]);
                sp->setColor(shapes[idx]->getColor());
                shapes_merged.emplace_back(sp);
            }
        }
        return true;
    }

    bool Primitive_Merger::merge_torus(std::vector<int> &torus_indices
                                       ,std::vector<SurfacePrimitive *> &shapes
                                       ,std::vector<SurfacePrimitive *> &shapes_merged
                                       ,std::vector<easy3d::vec3> &points
                                       ,std::vector<easy3d::vec3> &normal
                                       ,std::vector<std::vector<int>> &segments
                                       ,std::vector<std::vector<int>> &segments_merged) {
        for(size_t i = 0; i < torus_indices.size(); i++) {
            int idx = torus_indices[i];
            SurfacePrimitive* sp = clone_surface_primitive(shapes[idx]);
            sp->setColor(shapes[idx]->getColor());
            shapes_merged.emplace_back(sp);
        }

        for(size_t i = 0; i < torus_indices.size(); i++) {
            int idx = torus_indices[i];
            segments_merged.emplace_back(segments[idx]);
        }
        return true;
    }

}

