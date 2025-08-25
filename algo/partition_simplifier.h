//
// Created by 小乌嘎 on 2023/1/25.
//

#ifndef QUADFIT_PARTITION_SIMPLIFIER_H
#define QUADFIT_PARTITION_SIMPLIFIER_H

#include <util.h>
#include <EigenTypedef.h>
namespace PrimFit {

    struct node{
        double w;
        int id;
        int ct;
        node(double cw, int fid, int x)
                : w(cw), id(fid), ct(x) {}
        bool operator < (const node& n) const {
            return w > n.w;
        }
    };

    class Partition_Simplifier3 {
    public:
        Partition_Simplifier3() : is_init(false){};
        void init(MatrixDr& points_
                  ,MatrixDr& m_vertices_
                  ,MatrixIr& m_face_
                  ,VectorI& m_face_patch_
                  ,VectorI& m_face_label_
                  ,std::vector<std::vector<int>>& segments
                  ,double eps = 1e-3);

        double cover_cost(int fid, int eid);

        std::vector<int> extract_face_edge_index(int fid);

        int forward(int cur, int edge, int& ct);

        void simplify(float coverage = 0.8);

    public:
        MatrixDr points;
        MatrixDr m_vertices;
        MatrixIr m_face;
        VectorI m_face_patch;
        VectorI m_face_label;
        VectorI m_face_point_num;
        VectorD m_face_fit;
        VectorD m_face_valid_area;
        VectorD m_face_area;
        VectorI m_patch_label;
        std::vector<std::vector<int>> edge;
        std::vector<float> edge_len;
        std::vector<std::vector<int>> edge_face;
        std::vector<std::vector<int>> curve;
        std::vector<float> curve_len;
        std::map<int, std::map<int, int>> edge_map;
        VectorI vis;
        bool is_init;
    };
}


#endif //QUADFIT_PARTITION_SIMPLIFIER_H
