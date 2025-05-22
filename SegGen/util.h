//
// Created by 小乌嘎 on 2023/2/1.
//

#ifndef DATAPROCESSING_UTIL_H
#define DATAPROCESSING_UTIL_H

#include <vector>
#include <fitting.h>
#include <EigenTypedef.h>
#include <io.h>

namespace QuadFit {
    namespace Util{
        const std::vector<easy3d::vec3> color_table = {
                easy3d::vec3(0.216, 0.494, 0.722), // hard blue
                easy3d::vec3(0.302, 0.686, 0.290), // hard green
                easy3d::vec3(0.596, 0.306, 0.639), // hard purple
                easy3d::vec3(0.894, 0.100, 0.100), // hard red
                easy3d::vec3(1.000, 0.5, 0.000), // hard yellow
                easy3d::vec3(0.400, 0.761, 0.647), // medium blue
                easy3d::vec3(0.650, 0.847, 0.329), // medium green
                easy3d::vec3(0.553, 0.627, 0.796), // medium purple
                easy3d::vec3(0.906, 0.541, 0.765), // medium red
                easy3d::vec3(0.980, 0.550, 0.380), // medium yellow
                easy3d::vec3(0.500, 0.690, 0.827), // soft blue
                easy3d::vec3(0.553, 0.827, 0.780), // soft green
                easy3d::vec3(0.740, 0.720, 0.850), // soft purple
                easy3d::vec3(0.984, 0.502, 0.447), // soft red
                easy3d::vec3(1.000, 1.000, 0.700), // soft yellow
        };
        const easy3d::vec3 unknown = easy3d::vec3(0,0,0);

        void load_data(std::string& primitive_path,
                       std::string& parametric_ptah,
                       std::string& cloud_path
                       , std::vector<SurfacePrimitive*>& shapes
                       , std::vector<std::vector<int>> & segments
                       , std::vector<easy3d::vec3>& points
                       , std::vector<easy3d::vec3>& normals);

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
                ,std::vector<float>& curve_len);
    }
}

#endif //DATAPROCESSING_UTIL_H
