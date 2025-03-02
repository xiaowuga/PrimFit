//
// Created by 小乌嘎 on 2023/1/25.
//

#include "Partition_Simplifier2.h"
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
    void Partition_Simplifier2::init(MatrixDr &points_
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

        m_face_point_num = VectorI::Zero(num_face);
        m_face_fit = VectorD::Zero(num_face);
        m_face_valid_area = VectorD::Zero(num_face);

        igl::doublearea(m_vertices, m_face, m_face_area);
        m_face_area = m_face_area.array() / 2;

        std::vector<std::vector<int>> seg_face(num_seg);

        for(int i = 0; i < num_face; i++) {
            int idx = m_face_label[i];
            seg_face[idx].emplace_back(i);
        }
        for(int i = 0; i < num_seg; i++) {
            int num_p = segments[i].size();
//            std::cout << i << ' ' << num_p <<std::endl;
            MatrixDr P; P.resize(num_p, 3);
            for(int j = 0; j < num_p; j++) {
                int idx = segments[i][j];
                P.row(j) = points.row(idx);
            }
            int ct = 0;
            std::map<int, int> mp;
            int num_f = seg_face[i].size();
            MatrixDr V; V.resize(num_f * 3, 3);
            MatrixIr F; F.resize(num_f, 3);
            VectorI FI; FI.resize(num_f);
            for(int j = 0; j < num_f; j++) {
                int fidx = seg_face[i][j];
                for(int k = 0; k < 3; k++) {
                    int idx = m_face(fidx, k);
                    if(mp.find(idx) == mp.end()) {
                        mp[idx] = ct;
                        V.row(ct) = m_vertices.row(idx);
                        ct++;
                    }
                }

                int x = mp[m_face(fidx, 0)];
                int y = mp[m_face(fidx, 1)];
                int z = mp[m_face(fidx, 2)];
                F.row(j) = Vector3I(x, y, z);
                FI[j] = fidx;
            }
            V.conservativeResize(ct, 3);
            VectorD sqrD;
            MatrixD C;
            VectorI index;
            igl::point_mesh_squared_distance(P, V, F, sqrD, index, C);
            MatrixD FN;
            igl::per_face_normals(V, F, FN);
//            std::string out_path = "D:\\ICCV2023\\Selected_Fit4CAD\\test\\PC13\\tmp\\" + std::to_string(i) + ".obj";
//            std::string out_path_cloud = "D:\\ICCV2023\\Selected_Fit4CAD\\test\\PC13\\tmp\\" + std::to_string(i)+"_point1" + ".obj";
//            igl::writeOBJ(out_path, V, F);
            MatrixIr asd; asd.resize(0, 3);

            MatrixDr rP; rP.resize(num_p, 3);
            MatrixDr rPN; rPN.resize(num_p, 3);
            VectorI rPI;  rPI.resize(num_p);
            ct = 0;
            for(int j = 0; j < num_p; j++) {
                double dis = std::sqrt(sqrD[j]);
                int fid1 = index[j];
                int fid2 = FI[fid1];
                if(dis < eps) {
                    rP.row(ct) = C.row(j);
                    rPN.row(ct) = FN.row(fid1);
                    rPI[ct] = fid1;
                    ct++;
                    m_face_point_num[fid2]++;
                    m_face_fit[fid2] += dis;
                }
            }
            int num_rp = ct;
            rP.conservativeResize(num_rp, 3);
            rPN.conservativeResize(num_rp, 3);
            rPI.conservativeResize(num_rp);

            easy3d::StopWatch w; w.start();
            std::vector<std::vector<int > > O_PI;
            Eigen::MatrixXi O_CH;
            Eigen::MatrixXd O_CN;
            Eigen::VectorXd O_W;

            igl::octree(rP,O_PI,O_CH,O_CN,O_W);

            Eigen::VectorXf A;
            {
                Eigen::MatrixXi I;
                igl::knn(rP,20,O_PI,O_CH,O_CN,O_W,I);
                // CGAL is only used to help get point areas
                igl::copyleft::cgal::point_areas(rP,I,rPN,A);
            }
            xx += w.elapsed_seconds(3);
            for(int j = 0; j < num_rp; j++) {
                int fid1 = rPI[j];
                int fid2 = FI[fid1];
                m_face_valid_area[fid2] += A[j];
//                m_face_valid_area[fid2] += 0.01;
//                std::cout << A[j] <<std::endl;
            }
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
        std::cout << xx <<std::endl;
        vis = VectorI::Zero(num_face);
        is_init = true;
    }

    double Partition_Simplifier2::cover_cost(int fid, int eid) {
        return m_face_area[fid] * 2 / (edge_len[eid] + 1e-8);
    }

    std::vector<int> Partition_Simplifier2::extract_face_edge_index(int fid) {
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

    int Partition_Simplifier2::forward(int cur, int edge, int& ct) {
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
            if(ct >= 0) {
                res = -1;
            } else {
                ct += 1;
            }
        }
        return res;
    }

    void Partition_Simplifier2::simplify(float coverage) {
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
                if(tar >= 1) {
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