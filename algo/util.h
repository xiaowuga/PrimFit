//
// Created by 小乌嘎 on 2023/1/13.
//

#ifndef QUADFIT_UTIL_H
#define QUADFIT_UTIL_H

#include <easy3d/core/types.h>
#include <fitting.h>
#include <EigenTypedef.h>

namespace PrimFit {
    namespace Util {
        void construct_SurfacePrimitive_from_std(std::vector<SurfacePrimitive*>& shapes
                                                 ,std::vector<int>& type
                                                 ,std::vector<easy3d::vec3>& pos
                                                 ,std::vector<easy3d::vec3>& dir
                                                 ,std::vector<float>& r1
                                                 ,std::vector<float>& r2
                                                 ,std::vector<easy3d::vec3>& color);

        void construct_std_from_SurfacePrimitive(std::vector<SurfacePrimitive*>& shapes
                                                 ,std::vector<int>& type
                                                 ,std::vector<easy3d::vec3>& pos
                                                 ,std::vector<easy3d::vec3>& dir
                                                 ,std::vector<float>& r1
                                                 ,std::vector<float>& r2
                                                 ,std::vector<easy3d::vec3>& color);

        void construct_arrangement_from_std(MatrixDr& m_vertives
                                            ,MatrixIr& m_faces
                                            ,VectorI& m_face_labels
                                            ,MatrixIr& m_cells
                                            ,VectorI& m_patches
                                            ,std::vector<easy3d::vec3>& arr_points
                                            ,std::vector<int>& arr_indices
                                            ,std::vector<int>& arr_labels
                                            ,std::vector<int>& arr_patches
                                            ,std::vector<int>& arr_cells);

        void construct_std_from_arrangement(MatrixDr& m_vertives
                                            ,MatrixIr& m_faces
                                            ,VectorI& m_face_labels
                                            ,MatrixIr& m_cells
                                            ,VectorI& m_patches
                                            ,std::vector<easy3d::vec3>& arr_points
                                            ,std::vector<int>& arr_indices
                                            ,std::vector<int>& arr_labels
                                            ,std::vector<int>& arr_patches
                                            ,std::vector<int>& arr_cells);

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
                                     ,std::vector<int>& patch_label);

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
                                     ,std::vector<int>& patch_label);

        void merge_proxy_from_std(MatrixDr& V,MatrixIr& F, VectorI& L
                                  ,std::vector<std::vector<easy3d::vec3>>& points
                                  ,std::vector<std::vector<int>>& indices);

        void construct_line_indices_from_triangle(std::vector<int>& line_indices
                                                  ,MatrixIr& m_faces);

        void construct_vertices_list_from_std_to_igl(MatrixDr& V, std::vector<easy3d::vec3>& points);

        void construct_vertices_list_from_igl_to_std(MatrixDr& V, std::vector<easy3d::vec3>& points);

        template<typename IGLS, typename STDS>
        void construct_list_from_std_to_igl(Eigen::Matrix<IGLS, Eigen::Dynamic, 1>& igl_list, std::vector<STDS>& std_list);


        template<typename IGLS, typename STDS>
        void construct_list_from_igl_to_std(Eigen::Matrix<IGLS, Eigen::Dynamic, 1>& igl_list, std::vector<STDS>& std_list);

        template<typename IGLS, typename STDS>
        void construct_vec_list_from_std_to_igl(Eigen::Matrix<IGLS, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& igl_vec_list
                                                , std::vector<STDS>& std_vec_list, int DIMC);

        template<typename IGLS, typename STDS>
        void construct_vec_list_from_igl_to_std(Eigen::Matrix<IGLS, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>& igl_vec_list
                                                , std::vector<STDS>& std_vec_list);

        Vector3d construct_vertices_from_easy3d_to_igl(easy3d::vec3& v1);

        easy3d::vec3 construct_vertices_from_igl_to_easy3d(Vector3d& v1);

        void extract_curve_info(MatrixDr& m_vertives
                                     ,MatrixIr& m_face
                                     ,VectorI& m_face_patch
                                     ,VectorI& m_patch_label
                                     ,std::vector<std::vector<int>>& edge
                                     ,std::vector<std::vector<int>>& edge_face
                                     ,std::vector<std::vector<int>>& curve
                                     ,std::vector<float>& curve_len
                                     ,bool use_label = false);

        void compute_face_coverage();

    }
}


#endif //QUADFIT_UTIL_H
