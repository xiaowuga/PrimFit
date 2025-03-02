//
// Created by 小乌嘎 on 2023/2/4.
//
#include <io.h>
#include <util.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/fileio/surface_mesh_io.h>
#include <igl/writePLY.h>

int main() {
    std::string input_path = "D:\\ICCV2023\\experiment\\E1\\chamfer\\Ours\\chamfer.pblp";
    std::vector<easy3d::vec3> vertex;
    std::vector<int> face;
    std::vector<int> face_label;
    std::vector<int> face_patch;
    std::vector<int> face_point;
    std::vector<float> face_fit;
    std::vector<float> face_valid_area;
    std::vector<float> face_area;
    std::vector<easy3d::vec3> colors;
    std::vector<int> patch_label;
    QuadFit::IO::load_pblp(input_path, vertex, face, face_label, face_patch, face_point, face_fit, face_valid_area,
                           face_area, colors, patch_label);
    MatrixDr m_vertives;
    MatrixIr m_faces;
    VectorI m_face_labels;
    VectorI m_face_patches;
    VectorI m_face_point_num;
    VectorD m_face_fit;
    VectorD m_face_valid_area;
    VectorD m_face_area;
    MatrixDr m_face_color;
    VectorI m_patch_label;
    QuadFit::Util::construct_pblp_from_std(m_vertives, m_faces, m_face_labels, m_face_patches, m_face_point_num,
                                           m_face_fit, m_face_valid_area, m_face_area, m_face_color, m_patch_label,
                                           vertex, face, face_label, face_patch, face_point, face_fit, face_valid_area,
                                           face_area, colors, patch_label);

    std::string out_dir = "D:\\ICCV2023\\Figure\\Overview\\grow_curve";
    int num_face = m_faces.rows();
    {
//        std::map<int, int> mp;
//        MatrixDr V;
//        V.resize(m_vertives.rows(), 3);
//        MatrixIr F;
//        F.resize(m_faces.rows(), 3);
//        int num_v = 0, num_f = 0;
//        for (int i = 0; i < num_face; i++) {
//            int p = m_face_patches[i];
//            if (patch_label[p]) {
//                for (int j = 0; j < 3; j++) {
//                    int idx = m_faces(i, j);
//                    if (mp.find(idx) == mp.end()) {
//                        V.row(num_v) = m_vertives.row(idx);
//                        mp[idx] = num_v;
//                        num_v++;
//                    }
//                }
//            }
//        }
//
//        for (int i = 0; i < num_face; i++) {
//            int p = m_face_patches[i];
//            if (!patch_label[p]) {
//                int x = mp[m_faces(i, 0)];
//                int y = mp[m_faces(i, 1)];
//                int z = mp[m_faces(i, 2)];
//                F.row(num_f) = Vector3I(x, y, z);
//                num_f++;
//            }
//        }
//        V.conservativeResize(num_v, 3);
//        F.conservativeResize(num_f, 3);
//        std::string name = "/grow_active.ply";
//        std::string out_path = out_dir + name;
//        igl::writePLY(out_path, V, F);
    }


    {
        int num_seg = m_face_labels.maxCoeff() + 1;
        int num_points = m_vertives.rows();
        std::vector<std::vector<int>> seg_face;
        seg_face.resize(num_seg);
        for (int i = 0; i < num_face; i++) {
            int idx = m_face_labels[i];
            seg_face[idx].emplace_back(i);
        }
//        std::cout << num_points << std::endl;
        for (int i = 0; i < num_seg; i++) {
            std::map<int, int> mp;
            int ct = 0;
            MatrixDr P;
            P.resize(num_points, 3);
            for (size_t j = 0; j < seg_face[i].size(); j++) {
                int idx = seg_face[i][j];
                for (int k = 0; k < 3; k++) {
                    int id = m_faces(idx, k);
                    if (mp.find(id) == mp.end()) {
                        P.row(ct) = m_vertives.row(id);
                        mp[id] = ct++;
                    }
                }
            }

            P.conservativeResize(ct, 3);
            MatrixI F;
            F.resize(seg_face[i].size(), 3);
            VectorI L;
            L.resize(seg_face[i].size());
            VectorI PL;
            PL.resize(seg_face[i].size());
            for (size_t j = 0; j < seg_face[i].size(); j++) {
                int idx = seg_face[i][j];
                int x = mp[m_faces(idx, 0)];

                int y = mp[m_faces(idx, 1)];

                int z = mp[m_faces(idx, 2)];

                F.row(j) = Vector3I(x, y, z);
                L[j] = m_face_patches[idx];
            }
            std::cout << F.rows() << std::endl;
            std::map<int, std::map<int, std::set<int>>> mpp;

            std::map<int, std::map<int, std::map<int, int>>> ss;
            for (int j = 0; j < F.rows(); j++) {
                std::vector<int> tmp = {F(j, 0), F(j, 1), F(j, 2)};
                std::sort(tmp.begin(), tmp.end());
                mpp[tmp[0]][tmp[1]].insert(j);
                mpp[tmp[0]][tmp[2]].insert(j);
                mpp[tmp[1]][tmp[2]].insert(j);
            }
            int asd = 0;
            for (auto it: ss) {
                for (auto jt: it.second) {
                    for (auto kt: jt.second) {
                        asd++;
                    }
                }
            }
//            std::cout << asd <<std::endl;
            std::string name = "\\Primitive" + std::to_string(i + 1) + "_grow_curve" + ".obj";

            std::string out_path = out_dir + name;
            std::ofstream outine(out_path.c_str());
            if (outine.is_open()) {
                for (int j = 0; j < P.rows(); j++) {
                    outine << "v " << P(j, 0) << ' '
                           << P(j, 1) << ' ' << P(j, 2) << std::endl;
                }

                for (auto item: mpp) {
                    int x = item.first;
                    for (auto jtem: item.second) {
                        int y = jtem.first;
                        std::vector<int> tmp(jtem.second.begin(), jtem.second.end());
                        int flag = false;
                        if (tmp.size() == 2) {
                            int f1 = tmp[0], f2 = tmp[1];
                            if (L[f1] != L[f2] && (m_patch_label[L[f1]] == 1 || m_patch_label[L[f2]] == 1)) {
                                flag = true;
                            }
                        } else if (tmp.size() == 1 && m_patch_label[L[tmp[0]]] == 1) {
                            flag = true;
                        }
                        if (flag) {
                            outine << "l " << x + 1 << ' ' << y + 1 << std::endl;
                        }

                    }
                }
                outine.close();
            }

        }
    }
        return 0;
}
