//
// Created by 小乌嘎 on 2023/1/25.
//

#include "partition_simplifier.h"
#include <igl/doublearea.h>
#include <igl/per_face_normals.h>
#include <igl/point_mesh_squared_distance.h>
#include <igl/knn.h>
#include <igl/octree.h>
#include <igl/copyleft/cgal/point_areas.h>
#include <queue>
#include <Eigen/StdVector>
#include <igl/writeOBJ.h>
#include <easy3d/util/stop_watch.h>
namespace PrimFit {
    void Partition_Simplifier3::init(MatrixDr &points_
                                    , MatrixDr &m_vertices_
                                    , MatrixIr &m_face_
                                    , VectorI &m_face_patch_
                                    , VectorI &m_face_label_
                                    , std::vector<std::vector<int>>& segments
                                    ,double eps) {
        double xx = 0;
        points = points_;
        m_vertices = m_vertices_;
        m_face = m_face_;
        m_face_patch = m_face_patch_;
        m_face_label = m_face_label_;

        int num_points = points.rows();
        int num_vertices = m_vertices.rows();
        int num_face = m_face.rows();
        int num_seg = segments.size();

        VectorD sqrD;
        MatrixD C;
        VectorI index;

        igl::point_mesh_squared_distance(points, m_vertices, m_face_, sqrD, index, C);
        MatrixD FN;
        igl::per_face_normals(m_vertices, m_face, FN);
        m_face_point_num = VectorI::Zero(num_face);
        m_face_fit = VectorD::Zero(num_face);
        m_face_valid_area = VectorD::Zero(num_face);

        igl::doublearea(m_vertices, m_face, m_face_area);
        m_face_area = m_face_area.array() / 2;

        MatrixDr P; P.resize(num_points, 3);
        MatrixDr PN; PN.resize(num_points, 3);
        VectorI PF;  PF.resize(num_points);

        int ct = 0;
        for(int i = 0; i < num_points; i++) {
            double dis = sqrt(sqrD[i]);
            if (dis < eps) {
                int idx = index[i];
                m_face_point_num[idx] += 1;
                m_face_fit[idx] += sqrt(sqrD[i]);
                P.row(ct) = points.row(i);
                PN.row(ct) = FN.row(idx);
                PF[ct] = idx;
                ct++;
            }
        }

        P.conservativeResize(ct, 3);
        PN.conservativeResize(ct, 3);
        PF.conservativeResize(ct);

        std::vector<std::vector<int > > O_PI;
        Eigen::MatrixXi O_CH;
        Eigen::MatrixXd O_CN;
        Eigen::VectorXd O_W;

        igl::octree(P,O_PI,O_CH,O_CN,O_W);

        Eigen::VectorXf A;
        {
            Eigen::MatrixXi I;
            igl::knn(points_,8,O_PI,O_CH,O_CN,O_W,I);
            // CGAL is only used to help get point areas
            igl::copyleft::cgal::point_areas(P,I,PN,A);
        }

        for(int i = 0; i < ct; i++) {
            int idx = PF[i];
            m_face_valid_area[idx] += A[i];
        }

        for(int i = 0; i < num_face; i++) {
            if(m_face_valid_area[i] > m_face_area[i]) {
                m_face_valid_area[i] = m_face_area[i];
            }
            if(m_face_point_num[i] != 0) {
                m_face_fit[i] /= m_face_point_num[i];
            }
        }

        int num_patch = m_face_patch.maxCoeff() + 1;
        m_patch_label = VectorI::Ones(num_patch);

        edge.clear();
        edge_face.clear();
        curve.clear();
        curve_len.clear();
        Util::extract_curve_info(m_vertices, m_face, m_face_patch, m_patch_label
                                 ,edge, edge_face, curve, curve_len);

        edge_map.clear();
        edge_len.resize(edge.size());
        for(size_t i = 0; i < edge.size(); i++) {
            int s = edge[i][0], t = edge[i][1];
            edge_len[i] = (m_vertices.row(s) - m_vertices.row(t)).norm();
            if(s > t) {
                std::swap(s, t);
            }
            edge_map[s][t] = i;
        }
        vis = VectorI::Zero(num_face);
        is_init = true;
    }

    double Partition_Simplifier3::cover_cost(int fid, int eid) {
        return m_face_area[fid] * 2 / (edge_len[eid] + 1e-8);
    }

    std::vector<int> Partition_Simplifier3::extract_face_edge_index(int fid) {
        std::vector<int> res;
        if(!is_init) {
            return res;
        }

        for(int i = 0; i < 3; i++) {
            int s = m_face(fid, i);
            int t = m_face(fid, (i + 1) % 3);
            if(s > t) {
                std::swap(s, t);
            }
            int eid = edge_map[s][t];
            res.emplace_back(eid);
        }

        return res;
    }

    int Partition_Simplifier3::forward(int cur, int edge, int& ct) {
        int res = -1;
        int cur_label = m_face_label[cur];
        int cur_patch = m_face_patch[cur];
        int count = 0;
        if(edge_face[edge].size() == 3) {
            return res;
        }
        for(size_t i = 0; i < edge_face[edge].size(); i++) {
            int tar = edge_face[edge][i];
            int tar_label = m_face_label[tar];
            if(cur != tar && cur_label == tar_label && vis[tar] == 0) {
                res = tar;
            }
            if(cur_label != tar_label && vis[tar] != 0) {
                count++;
            }
        }
        if(count >= 1 && res != -1) {
            if(ct >= 2) {
                res = -1;
            } else {
                ct += 1;
            }
        }
        return res;
    }

    void Partition_Simplifier3::simplify(float coverage) {
        if(!is_init) {
            return;
        }
        int num_face = m_face.rows();
        std::priority_queue<node> que;
        for(int i = 0; i < num_face; i++) {
            double fcov = m_face_valid_area[i] / m_face_area[i];
            if(fcov >= coverage) {
                que.push(node(0, i, 0));
                vis[i] = 1;
            }
        }
        while(!que.empty()) {
            double w = que.top().w;
//            std::cout << w <<std::endl;
            int cur = que.top().id;
            int ct = que.top().ct;

            que.pop();
            std::vector<int> e = extract_face_edge_index(cur);

            for(size_t i = 0; i < e.size(); i++) {
                int tmp = ct;
                int tar = forward(cur, e[i], tmp);
                if(tar >= 0) {
                    double cc = cover_cost(tar, e[i]);
                    vis[tar] = 1;
                    que.push(node(w + cc, tar, tmp));
                }
            }
        }
        m_patch_label.setZero();
        for(int i = 0; i < num_face; i++) {
            int p = m_face_patch[i];
            if(vis[i] > 0) {
                m_patch_label[p] = 1;
            }
        }
    }
}