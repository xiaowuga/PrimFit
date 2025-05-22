//
// Created by 小乌嘎 on 2022/11/21.
//

#ifndef QUADFIT_MESH_GENERATER_H
#define QUADFIT_MESH_GENERATER_H

#include <easy3d/core/types.h>
#include <fitting.h>

namespace QuadFit {
    class Mesh_Generater {
    public:
        Mesh_Generater() = default;

        Mesh_Generater(const easy3d::Box3 &bbox_) {
            set_bbox(bbox_);
        }

        void set_bbox(const easy3d::Box3 &bbox_);

        bool generate_plane_mesh(SurfacePrimitive* plane,
                                 std::vector <easy3d::vec3> &points,
                                 std::vector<int> &indices);

        bool generate_cylinder_mesh(SurfacePrimitive* cylinder,
                                    std::vector <SurfacePrimitive*>& tangent_planes,
                                    std::vector <easy3d::vec3> &points,
                                    std::vector<int> &indices,
                                    int n1 = 36, int n2 = 10);

        bool generate_cone_mesh(SurfacePrimitive* cone,
                                std::vector <SurfacePrimitive*>& tangent_plane,
                                std::vector <easy3d::vec3> &points,
                                std::vector<int> &indices,
                                int n1 = 36, int n2 = 10);

        bool generate_sphere_mesh(SurfacePrimitive* sphere,
                                  std::vector <SurfacePrimitive*>& tangent_plane,
                                  std::vector <easy3d::vec3> &points,
                                  std::vector<int> &indices,
                                  int n1 = 36, int n2 = 36);

        void generate_unit_sphere(std::vector<easy3d::vec3> &points,
                                  std::vector<int> &indices,
                                  int n_latitude, int n_longitude);

        bool generate_torus_mesh(SurfacePrimitive *torus
                                                 ,std::vector<SurfacePrimitive *> &tangent_plane
                                                 ,std::vector<easy3d::vec3> &points
                                                 ,std::vector<int>& indices
                                                 ,int n_major = 36
                                                 ,int n_minor = 36);

        bool generate_proxy_mesh(std::vector<SurfacePrimitive*>& shapes
                                 , std::vector<std::vector<easy3d::vec3>>& proxy_points
                                 , std::vector<std::vector<int>>& proxy_indices);

    private:
        easy3d::Box3 bbox;
        std::vector <easy3d::vec3> bbox_points;
        std::vector<int> bbox_indices;

        double bbox_eps = 0.005;
    };

};
#endif //QUADFIT_MESH_GENERATER_H
