//
// Created by 小乌嘎 on 2023/2/3.
//
#include <io.h>
#include <util.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/fileio/surface_mesh_io.h>
#include <igl/writeOBJ.h>

int main() {

    std::string input_path = "D:\\ICCV2023\\experiment\\E1\\chamfer\\Ours\\chamfer_arrangement.qf";
    int cur_step;
    std::vector<int> type;
    std::vector<easy3d::vec3> pos;
    std::vector<easy3d::vec3> dir;
    std::vector<float> r1;
    std::vector<float> r2;
    std::vector<easy3d::vec3> points;
    std::vector<easy3d::vec3> normals;
    std::vector<std::vector<int>> segments;
    std::vector<easy3d::vec3> colors;
    std::vector<std::vector<easy3d::vec3>> proxy_points;
    std::vector<std::vector<int>> proxy_indices;
    std::vector<easy3d::vec3> arr_points;
    std::vector<int> arr_indices;
    std::vector<int> arr_patches;
    std::vector<int> arr_labels;
    std::vector<int> arr_cells;
    QuadFit::IO::load_quadfit(input_path, cur_step, type, pos, dir, r1, r2, points, normals, segments, colors,
                              proxy_points, proxy_indices, arr_points, arr_indices, arr_cells, arr_patches, arr_labels);

    MatrixDr m_vertices;
    MatrixIr m_faces;
    VectorI m_face_labels;
    MatrixIr m_cells;
    VectorI m_patches;

    QuadFit::Util::construct_arrangement_from_std(m_vertices, m_faces, m_face_labels, m_cells, m_patches, arr_points,
                                                  arr_indices, arr_labels, arr_patches, arr_cells);
    int num_patch = m_patches.maxCoeff() + 1;
    int num_face = m_faces.rows();
    std::vector<std::vector<int>> patch_face;
    patch_face.resize(num_patch);
    std::vector<int> patch_seg;
    patch_seg.resize(num_patch, -1);
    for (int i = 0; i < num_face; i++) {
        int seg_id = m_face_labels[i];
        int pat_id = m_patches[i];
        if (patch_seg[pat_id] == -1) {
            patch_seg[pat_id] = seg_id;
        } else {
            if (patch_seg[pat_id] != seg_id) {
                std::cout << "error!" << std::endl;
            }
        }
        patch_face[pat_id].emplace_back(i);
    }
    std::string out_dir = "D:\\ICCV2023\\Figure\\Overview\\arrangement_mesh";

    {
        int num_seg = segments.size();
        int num_points = m_vertices.rows();
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
                        P.row(ct) = m_vertices.row(id);
                        mp[id] = ct++;
                    }
                }
            }

            P.conservativeResize(ct, 3);
            MatrixI F;
            F.resize(seg_face[i].size(), 3);
            VectorI L;
            L.resize(seg_face[i].size());
            for (size_t j = 0; j < seg_face[i].size(); j++) {
                int idx = seg_face[i][j];
                int x = mp[m_faces(idx, 0)];
//                auto vx = easy3d::SurfaceMesh::Vertex(x);
                int y = mp[m_faces(idx, 1)];
//                auto vy = easy3d::SurfaceMesh::Vertex(y);
                int z = mp[m_faces(idx, 2)];
//                auto vz = easy3d::SurfaceMesh::Vertex(z);
                F.row(j) = Vector3I(x, y, z);
                L[j] = m_patches[idx];
            }
            std::cout << F.rows() <<std::endl;
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
            for(auto it : ss) {
                for(auto jt : it.second) {
                    for(auto kt : jt.second) {
                        asd++;
                    }
                }
            }
//            std::cout << asd <<std::endl;
            std::string name = "\\Primitive" + std::to_string(i + 1) + "_arr_mesh" + ".obj";

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
                            if (L[f1] != L[f2]) {
                                flag = true;
                            }
                        } else if (tmp.size() == 1) {
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


        return 0;
    }
}