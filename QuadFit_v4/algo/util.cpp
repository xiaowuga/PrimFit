//
// Created by 小乌嘎 on 2023/1/13.
//

#include "util.h"
#include <algorithm>
#include <map>
#include <set>

namespace QuadFit {
    namespace Util {
        void construct_SurfacePrimitive_from_std(std::vector<SurfacePrimitive*>& shapes
                ,std::vector<int>& type
                ,std::vector<easy3d::vec3>& pos
                ,std::vector<easy3d::vec3>& dir
                ,std::vector<float>& r1
                ,std::vector<float>& r2
                ,std::vector<easy3d::vec3>& color) {

            for(size_t i = 0; i < shapes.size(); i++) {
                delete shapes[i];
            }
            shapes.clear();

            int num = type.size();
            for(int i = 0; i < num; i++) {
                SurfaceParameters para(pos[i], dir[i], r1[i], r2[i]);
                SurfaceType surfaceType = to_type(type[i]);
                shapes.emplace_back(construct_bytype(surfaceType, para));
                shapes.back()->setColor(color[i]);
            }
        }

        void construct_std_from_SurfacePrimitive(std::vector<SurfacePrimitive*>& shapes
                ,std::vector<int>& type
                ,std::vector<easy3d::vec3>& pos
                ,std::vector<easy3d::vec3>& dir
                ,std::vector<float>& r1
                ,std::vector<float>& r2
                ,std::vector<easy3d::vec3>& color) {

            int num = shapes.size();
            type.resize(num); pos.resize(num); dir.resize(num);
            r1.resize(num); r2.resize(num); color.resize(num);
            for(int i = 0; i < num; i++) {
                type[i] = to_int_label(shapes[i]->getType());
                SurfaceParameters para = shapes[i]->getParameters();
                pos[i] = para.pos; dir[i] = para.dir;
                r1[i] = para.r1; r2[i] = para.r2;
                color[i] = shapes[i]->getColor();
            }
        }

        void construct_arrangement_from_std(MatrixDr& m_vertives
                                            ,MatrixIr& m_faces
                                            ,VectorI& m_face_labels
                                            ,MatrixIr& m_cells
                                            ,VectorI& m_patches
                                            ,std::vector<easy3d::vec3>& arr_points
                                            ,std::vector<int>& arr_indices
                                            ,std::vector<int>& arr_labels
                                            ,std::vector<int>& arr_patches
                                            ,std::vector<int>& arr_cells) {
            construct_vertices_list_from_std_to_igl(m_vertives, arr_points);

            construct_vec_list_from_std_to_igl(m_faces, arr_indices, 3);

            construct_list_from_std_to_igl(m_face_labels, arr_labels);

            construct_list_from_std_to_igl(m_patches, arr_patches);

            construct_vec_list_from_std_to_igl(m_cells, arr_cells, 2);
        }

        void construct_std_from_arrangement(MatrixDr& m_vertives
                ,MatrixIr& m_faces
                ,VectorI& m_face_labels
                ,MatrixIr& m_cells
                ,VectorI& m_patches
                ,std::vector<easy3d::vec3>& arr_points
                ,std::vector<int>& arr_indices
                ,std::vector<int>& arr_labels
                ,std::vector<int>& arr_patches
                ,std::vector<int>& arr_cells) {

            construct_vertices_list_from_igl_to_std(m_vertives, arr_points);

            construct_vec_list_from_igl_to_std(m_faces, arr_indices);

            construct_list_from_igl_to_std(m_face_labels, arr_labels);

            construct_list_from_igl_to_std(m_patches, arr_patches);

            construct_list_from_igl_to_std(m_patches, arr_patches);

            construct_vec_list_from_igl_to_std(m_cells, arr_cells);
        }


        void construct_pblp_from_std(MatrixDr& m_vertives
                                     ,MatrixIr& m_faces
                                     ,VectorI& m_face_labels
                                     ,VectorI& m_face_patches
                                     ,VectorI& m_face_point_num
                                     ,VectorD& m_face_fit
                                     ,VectorD& m_face_valid_area
                                     ,VectorD& m_face_area
                                     ,MatrixDr& m_face_color
                                     ,VectorI& m_patch_label
                                     ,std::vector<easy3d::vec3>& points
                                     ,std::vector<int>& face
                                     ,std::vector<int>& face_label
                                     ,std::vector<int>& face_patch
                                     ,std::vector<int>& face_point_num
                                     ,std::vector<float>& face_fit
                                     ,std::vector<float>& face_valid_area
                                     ,std::vector<float>& face_area
                                     ,std::vector<easy3d::vec3>& colors
                                     ,std::vector<int>& patch_label) {
            construct_vertices_list_from_std_to_igl(m_vertives, points);

            construct_vec_list_from_std_to_igl(m_faces, face, 3);

            construct_list_from_std_to_igl(m_face_labels, face_label);
            construct_list_from_std_to_igl(m_face_patches, face_patch);
            construct_list_from_std_to_igl(m_face_point_num, face_point_num);
            construct_list_from_std_to_igl(m_face_fit, face_fit);
            construct_list_from_std_to_igl(m_face_valid_area, face_valid_area);
            construct_list_from_std_to_igl(m_face_area, face_area);

            construct_vertices_list_from_std_to_igl(m_face_color, colors);

            construct_list_from_std_to_igl(m_patch_label, patch_label);
        }

        void construct_std_from_pblp(MatrixDr& m_vertives
                                     ,MatrixIr& m_faces
                                     ,VectorI& m_face_labels
                                     ,VectorI& m_face_patches
                                     ,VectorI& m_face_point_num
                                     ,VectorD& m_face_fit
                                     ,VectorD& m_face_valid_area
                                     ,VectorD& m_face_area
                                     ,MatrixDr& m_face_color
                                     ,VectorI& m_patch_label
                                     ,std::vector<easy3d::vec3>& points
                                     ,std::vector<int>& face
                                     ,std::vector<int>& face_label
                                     ,std::vector<int>& face_patch
                                     ,std::vector<int>& face_point_num
                                     ,std::vector<float>& face_fit
                                     ,std::vector<float>& face_valid_area
                                     ,std::vector<float>& face_area
                                     ,std::vector<easy3d::vec3>& colors
                                     ,std::vector<int>& patch_label) {
            construct_vertices_list_from_igl_to_std(m_vertives, points);

            construct_vec_list_from_igl_to_std(m_faces, face);

            construct_list_from_igl_to_std(m_face_labels, face_label);
            construct_list_from_igl_to_std(m_face_patches, face_patch);
            construct_list_from_igl_to_std(m_face_point_num, face_point_num);
            construct_list_from_igl_to_std(m_face_fit, face_fit);
            construct_list_from_igl_to_std(m_face_valid_area, face_valid_area);
            construct_list_from_igl_to_std(m_face_area, face_area);

            construct_vertices_list_from_igl_to_std(m_face_color, colors);

            construct_list_from_igl_to_std(m_patch_label, patch_label);

        }

        void merge_proxy_from_std(MatrixDr& V,MatrixIr& F, VectorI& L
                                  ,std::vector<std::vector<easy3d::vec3>>& points
                                  ,std::vector<std::vector<int>>& indices) {
            int num_vertices = 0, num_faces = 0;
            for(size_t i = 0; i < points.size(); i++) {
                num_vertices += points[i].size();
            }
            for(size_t i = 0; i < indices.size(); i++) {
                num_faces += indices[i].size() / 3;
            }
            V.resize(num_vertices, 3);
            F.resize(num_faces, 3);
            L.resize(num_faces);
            std::vector<unsigned int> beg_index(points.size());
            int idx = 0;

            for(size_t i = 0; i < points.size(); ++i) {
                beg_index[i] = idx;
                for(size_t j = 0; j < points[i].size(); ++j) {
                    V.row(idx++) = construct_vertices_from_easy3d_to_igl(points[i][j]);
                }
            }
            idx = 0;
            for(size_t i = 0; i < indices.size(); ++i) {
                for(size_t j = 0; j < indices[i].size(); j += 3) {
                    F(idx, 0) = beg_index[i] + indices[i][j];
                    F(idx, 1) = beg_index[i] + indices[i][j + 1];
                    F(idx, 2) = beg_index[i] + indices[i][j + 2];
                    L[idx] = i;
                    idx++;
                }
            }
        }


        void construct_line_indices_from_triangle(std::vector<int>& line_indices
                                                  ,MatrixIr& m_faces) {
            std::map<int, std::set<int>> mp;
            int num_f = m_faces.rows();
            for(int i = 0; i < num_f; i++) {
                int x = m_faces(i, 0), y = m_faces(i, 1), z = m_faces(i, 2);
                if(x > y) std::swap(x, y);
                if(x > z) std::swap(x, z);
                if(y > z) std::swap(y, z);
                mp[x].insert(y); mp[x].insert(z);
                mp[y].insert(z);
            }

            for(auto& item : mp) {
                int x = item.first;
                for(auto& jtem : item.second) {
                    line_indices.emplace_back(x);
                    line_indices.emplace_back(jtem);
                }
            }
        }

        void construct_vertices_list_from_std_to_igl(MatrixDr& V
                                                     , std::vector<easy3d::vec3>& points) {
            int num = points.size();
            V.resize(num, 3);

            for(int i = 0; i < num; i++) {
                V.row(i) = construct_vertices_from_easy3d_to_igl(points[i]);
            }
        }

        void construct_vertices_list_from_igl_to_std(MatrixDr& V
                                                     , std::vector<easy3d::vec3>& points) {
            int num = V.rows();
            points.resize(num);

            for(int i = 0; i < num; i++) {
                Vector3d v = V.row(i);
                points[i] = construct_vertices_from_igl_to_easy3d(v);
            }
        }

        template<typename IGLS, typename STDS>
        void construct_list_from_std_to_igl(Eigen::Matrix<IGLS, Eigen::Dynamic, 1>& igl_list
                                            , std::vector<STDS>& std_list) {
            int num = std_list.size();
            igl_list.resize(num, 1);
            for(int i = 0; i < num; i++) {
                igl_list(i, 0) = std_list[i];
            }
        }


        template<typename IGLS, typename STDS>
        void construct_list_from_igl_to_std(Eigen::Matrix<IGLS, Eigen::Dynamic, 1>& igl_list
                , std::vector<STDS>& std_list) {
            int num = igl_list.rows();
            std_list.resize(num);
            for(int i = 0; i < num; i++) {
                std_list[i] = igl_list(i, 0);
            }
        }


        template<typename IGLS, typename STDS>
        void construct_vec_list_from_std_to_igl(Eigen::Matrix<IGLS, Eigen::Dynamic, -1, Eigen::RowMajor>& igl_vec_list
                                                , std::vector<STDS>& std_vec_list
                                                , int DIMC) {
            int DIMR = std_vec_list.size() / DIMC;
            igl_vec_list.resize(DIMR, DIMC);
            for(int i = 0; i < DIMR; i++) {
                for(int j = 0; j < DIMC; j++) {
                    igl_vec_list(i, j) = std_vec_list[DIMC * i + j];
                }
            }

        }

        template<typename IGLS, typename STDS>
        void construct_vec_list_from_igl_to_std(Eigen::Matrix<IGLS, -1, -1, Eigen::RowMajor>& igl_vec_list
                                                , std::vector<STDS>& std_vec_list) {
            int DIMR = igl_vec_list.rows();
            int DIMC = igl_vec_list.cols();
            std_vec_list.resize(DIMR * DIMC);
            int idx = 0;
            for(int i = 0; i < DIMR; i++) {
                for(int j = 0; j < DIMC; j++) {
                    std_vec_list[idx++] = igl_vec_list(i, j);
                }
            }
        }


        Vector3d construct_vertices_from_easy3d_to_igl(easy3d::vec3& v1) {
            return Vector3d(v1.x, v1.y, v1.z);
        }

        easy3d::vec3 construct_vertices_from_igl_to_easy3d(Vector3d& v1) {
            return easy3d::vec3(v1[0], v1[1], v1[2]);
        }

        void extract_curve_info(MatrixDr& m_vertives
                                ,MatrixIr& m_face
                                ,VectorI& m_face_patch
                                ,VectorI& m_patch_label
                                ,std::vector<std::vector<int>>& edge
                                ,std::vector<std::vector<int>>& edge_face
                                ,std::vector<std::vector<int>>& curve
                                ,std::vector<float>& curve_len
                                ,bool use_label) {

            std::map<int, std::map<int, std::set<int>>> mp;
            int num_face = m_face.rows();
            for(int i = 0; i < num_face; i++) {
                int fp = m_face_patch[i];
                if(m_patch_label[fp] > 0 || (!use_label)) {
                    for (int j = 0; j < 3; j++) {
                        int s = m_face(i, j);
                        int t = m_face(i, (j + 1) % 3);
                        if (s > t) {
                            std::swap(s, t);
                        }
                        mp[s][t].insert(i);
                    }
                }
            }

            std::map<int, float> unary;
            std::map<int, std::map<int,double>> binary;
            std::map<int, std::map<int,std::map<int, float>>> ternary;
            std::map<int, std::map<int,std::map<int, std::map<int, float>>>> quaternary;
            edge.clear();
            edge_face.clear();
            for(auto& item : mp) {
                int s = item.first;
                for(auto& jtem : item.second) {
                    int t = jtem.first;
                    float len = (m_vertives.row(t) - m_vertives.row(s)).norm();
                    std::vector<int> edge_indices = {s, t};
                    std::vector<int> face_indices(jtem.second.begin(), jtem.second.end());
                    edge.emplace_back(edge_indices);
                    edge_face.emplace_back(face_indices);
                    std::set<int> patch_set;
                    for(size_t i = 0; i < face_indices.size(); i++) {
                        patch_set.insert(m_face_patch[face_indices[i]]);
                    }
                    std::vector<int> patch_indices(patch_set.begin(), patch_set.end());
                    if(patch_indices.size() == 1) {
                        if(face_indices.size() == 1) {
                            unary[patch_indices[0]] += len;
                        }
                    } else if(patch_indices.size() == 2) {
                        binary[patch_indices[0]][patch_indices[1]] += len;
                    } else if(patch_indices.size() == 3) {
                        ternary[patch_indices[0]][patch_indices[1]][patch_indices[2]] += len;
                    } else if(patch_indices.size() == 4) {
                        quaternary[patch_indices[0]][patch_indices[1]][patch_indices[2]][patch_indices[3]] += len;
                    } else {
                        std::cout << "Error, there is a egde around by " << patch_indices.size() << " patches" << std::endl;
                        for(int x = 0; x < patch_indices.size(); x++){
                            std::cout << patch_indices[x] << ' ';
                        }
                        std::cout << std::endl;
                    }
                }
            }

            curve.clear();
//            std::cout << "unary curve number = " << unary.size() << std::endl;
//            std::cout << "binary curve number = " << binary.size() << std::endl;
//            std::cout << "ternary curve number = " << ternary.size() << std::endl;
//            std::cout << "quaternary curve number = " << quaternary.size() << std::endl;
            for(auto& atem : unary) {
                std::vector<int>patch_indices = {atem.first};
                curve.emplace_back(patch_indices);
                curve_len.emplace_back(atem.second);
            }

            for(auto& atem : binary) {
                int a = atem.first;
                for(auto& btem : atem.second) {
                    int b = btem.first;
                    std::vector<int>patch_indices = {a, b};
                    curve.emplace_back(patch_indices);
                    curve_len.emplace_back(btem.second);
                }
            }

            for(auto& atem : ternary) {
                int a = atem.first;
                for(auto& btem : atem.second) {
                    int b = btem.first;
                    for(auto& ctem : btem.second) {
                        int c = ctem.first;
                        std::vector<int>patch_indices = {a, b, c};
                        curve.emplace_back(patch_indices);
                        curve_len.emplace_back(ctem.second);
                    }
                }
            }

            for(auto& atem : quaternary) {
                int a = atem.first;
                for(auto& btem : atem.second) {
                    int b = btem.first;
                    for(auto& ctem : btem.second) {
                        int c = ctem.first;
                        for(auto& dtem : ctem.second) {
                            int d = dtem.first;
                            std::vector<int>patch_indices = {a, b, c, d};
                            curve.emplace_back(patch_indices);
                            curve_len.emplace_back(dtem.second);
                        }
                    }
                }
            }

        }


    }
}