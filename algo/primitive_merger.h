//
// Created by 小乌嘎 on 2022/12/2.
//

#ifndef QUADFIT_PRIMITIVE_MERGER_H
#define QUADFIT_PRIMITIVE_MERGER_H
#include <fitting.h>
#include <easy3d/core/types.h>
#include <easy3d/core/point_cloud.h>
#include <numeric>

namespace PrimFit {

    class Primitive_Merger {
    public:
        Primitive_Merger() = default;
        Primitive_Merger(float normals_angle, float dis_eps)
        : normal_eps_(cos(normals_angle * M_PI / 180.0))
        , dis_eps_(dis_eps){}

        ~Primitive_Merger() = default;

        bool merge_primitves(std::vector<SurfacePrimitive*>& shapes
                             ,std::vector<SurfacePrimitive*>& shapes_merged
                             ,std::vector<easy3d::vec3>& points
                             ,std::vector<easy3d::vec3>& normals
                             ,std::vector<std::vector<int>>& segments
                             ,std::vector<std::vector<int>>& segments_merged
                             ,std::vector<std::vector<int>>& merge_pair);
        void set_normals_angle(float angle) {
            normal_eps_ = cos(angle * M_PI / 180.0);
        }

        void set_dis_eps(float dis) {
            dis_eps_ = dis;
        }

    private:
        void init_bcj(int n) {
            bcj_.resize(n);
            std::iota(bcj_.begin(), bcj_.end(), 0);
        }
        void merge_bcj(int x, int y) {
            if(x > y) std::swap(x, y);
            bcj_[y] = x;
        }


        int find_root(int k) {
            return bcj_[k] == k ? k : bcj_[k] = find_root(bcj_[k]);
        }

        void bcj_get(std::vector<std::vector<int>>& set);
        bool merge_planes(std::vector<int>& plane_indices
                          ,std::vector<SurfacePrimitive*>& shapes
                          ,std::vector<SurfacePrimitive*>& shapes_merged
                          ,std::vector<easy3d::vec3>& points
                          ,std::vector<easy3d::vec3>& normals
                          ,std::vector<std::vector<int>>& segments
                          ,std::vector<std::vector<int>>& segments_merged);

        bool merge_cylinder(std::vector<int>& cylinder_indices
                            ,std::vector<SurfacePrimitive*>& shapes
                            ,std::vector<SurfacePrimitive*>& shapes_merged
                            ,std::vector<easy3d::vec3>& points
                            ,std::vector<easy3d::vec3>& normals
                            ,std::vector<std::vector<int>>& segments
                            ,std::vector<std::vector<int>>& segments_merged);

        bool merge_cone(std::vector<int>& cone_indices
                        ,std::vector<SurfacePrimitive*>& shapes
                        ,std::vector<SurfacePrimitive*>& shapes_merged
                        ,std::vector<easy3d::vec3>& points
                        ,std::vector<easy3d::vec3>& normals
                        ,std::vector<std::vector<int>>& segments
                        ,std::vector<std::vector<int>>& segments_merged);

        bool merge_sphere(std::vector<int>& sphere_indices
                          ,std::vector<SurfacePrimitive*>& shapes
                          ,std::vector<SurfacePrimitive*>& shapes_merged
                          ,std::vector<easy3d::vec3>& points
                          ,std::vector<easy3d::vec3>& normals
                          ,std::vector<std::vector<int>>& segments
                          ,std::vector<std::vector<int>>& segments_merged);

        bool merge_torus(std::vector<int>& torus_indices
                         ,std::vector<SurfacePrimitive*>& shapes
                         ,std::vector<SurfacePrimitive*>& shapes_merged
                         ,std::vector<easy3d::vec3>& points
                         ,std::vector<easy3d::vec3>& normals
                         ,std::vector<std::vector<int>>& segments
                         ,std::vector<std::vector<int>>& segments_merged);

    private:
        std::vector<int> bcj_;
        std::map<int, int> merp;
        float normal_eps_ = 0.98; // angle;
        float dis_eps_ = 0.003;    // length
    };
}


#endif //QUADFIT_PRIMITIVE_MERGER_H
