//
// Created by 小乌嘎 on 2023/8/26.
//

#ifndef PRIMFIT_DATA_IO_H
#define PRIMFIT_DATA_IO_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <happly.h>

namespace PrimFit {
    typedef double FLOAT;
    typedef Eigen::Matrix<FLOAT, Eigen::Dynamic, 3> MatX3f;
    typedef Eigen::Matrix<int, Eigen::Dynamic, 3> MatX3i;
    typedef Eigen::Matrix<int, Eigen::Dynamic, 2> MatX2i;
    typedef Eigen::Matrix<int, 2, Eigen::Dynamic> RowMatX2i;
    typedef Eigen::Matrix<int, Eigen::Dynamic, 1> VecXi;
    typedef Eigen::Matrix<int, 1, Eigen::Dynamic> RowVecXi;
    typedef Eigen::Matrix<FLOAT,Eigen::Dynamic, 1> VecXf;
    typedef Eigen::Matrix<FLOAT,1,Eigen::Dynamic> RowVecXf;
    class PrimFitData{
    public:
        int step;
        MatX3f point_cloud;
        VecXi point_cloud_segmentation;

        std::vector<std::vector<FLOAT>> primitive_parameters;
        MatX3f primitive_colors;

//        MatX3f primitive_mesh_vertices;
//        MatX3i primitive_mesh_faces;
//        VecXi primitive_mesh_vertices_separator;
//        VecXi primitive_mesh_faces_separator;

        MatX3f arrangemented_vertices;

        MatX3i arrangemented_faces;
        VecXi arrangemented_face_patches;
        VecXi arrangemented_face_labels;

        MatX2i arrangemented_patch_cells;
        VecXf arrangemented_patch_area;
        VecXf arrangemented_patch_valid_area;
        VecXi arrangemented_patch_point_num;
        VecXi pruning_labels;

        MatX3f result_mesh_vertices;
        MatX3i result_mesh_faces;
        MatX2i result_mesh_sharp_edges;

    };
    void save_primfit_data(const std::string& pfd_path, const PrimFitData& pfd, bool binary = true) {
        // element step
        int cur_step = pfd.step;

        // element point_cloud
        std::vector<FLOAT> point_cloud_x, point_cloud_y, point_cloud_z;
        std::vector<int> point_cloud_segmentation;

        // element primitives
        std::vector<std::vector<FLOAT>> primitive_parameters;
        std::vector<FLOAT> primitive_colors_red;
        std::vector<FLOAT> primitive_colors_green;
        std::vector<FLOAT> primitive_colors_blue;

        if(cur_step >= 1) {
            point_cloud_x.resize(pfd.point_cloud.rows());
            point_cloud_y.resize(pfd.point_cloud.rows());
            point_cloud_z.resize(pfd.point_cloud.rows());
            point_cloud_segmentation.resize(pfd.point_cloud.rows());
            for (int i = 0; i < pfd.point_cloud.rows(); i++) {
                point_cloud_x[i] = pfd.point_cloud(i, 0);
                point_cloud_y[i] = pfd.point_cloud(i, 1);
                point_cloud_z[i] = pfd.point_cloud(i, 2);
                point_cloud_segmentation[i] = pfd.point_cloud_segmentation[i];
            }

            primitive_parameters.resize(pfd.primitive_parameters.size());
            primitive_colors_red.resize(pfd.primitive_parameters.size());
            primitive_colors_green.resize(pfd.primitive_parameters.size());
            primitive_colors_blue.resize(pfd.primitive_parameters.size());
            for(int i = 0; i < pfd.primitive_parameters.size(); i++) {
                primitive_parameters[i] = pfd.primitive_parameters[i];
                const VecXf& row_vec = pfd.primitive_colors.row(i);
                primitive_colors_red[i] = row_vec[0];
                primitive_colors_green[i] = row_vec[1];
                primitive_colors_blue[i] = row_vec[2];
            }
        }

        // element arrangemented_vertives
        std::vector<FLOAT> arrangemented_vertices_x, arrangemented_vertices_y, arrangemented_vertices_z;

        // element arrangement_faces
        std::vector<std::vector<int>> arrangemented_faces;
        std::vector<int> arrangemented_face_patches, arrangemented_face_labels;

        if(cur_step >= 2) {
            arrangemented_vertices_x.resize(pfd.arrangemented_vertices.rows());
            arrangemented_vertices_y.resize(pfd.arrangemented_vertices.rows());
            arrangemented_vertices_z.resize(pfd.arrangemented_vertices.rows());
            for (int i = 0; i < pfd.arrangemented_vertices.rows(); i++) {
                arrangemented_vertices_x[i] = pfd.arrangemented_vertices(i, 0);
                arrangemented_vertices_y[i] = pfd.arrangemented_vertices(i, 1);
                arrangemented_vertices_z[i] = pfd.arrangemented_vertices(i, 2);
            }

            arrangemented_faces.resize(pfd.arrangemented_faces.rows());
            arrangemented_face_patches.resize(pfd.arrangemented_faces.rows());
            arrangemented_face_labels.resize(pfd.arrangemented_faces.rows());
            for (int i = 0; i < pfd.arrangemented_faces.rows(); i++) {
                const VecXi &row_vec = pfd.arrangemented_faces.row(i);
                arrangemented_faces[i] = std::vector<int>(row_vec.data(), row_vec.data() + row_vec.size());
                arrangemented_face_patches[i] = pfd.arrangemented_face_patches[i];
                arrangemented_face_labels[i] = pfd.arrangemented_face_labels[i];
            }
        }

        // element arrangement_patches
        std::vector<std::vector<int>> arrangemented_patch_cells;
        std::vector<FLOAT> arrangemented_patch_area, arrangemented_patch_valid_area;
        std::vector<int> arrangemented_patch_point_num, pruning_labels;
        if(cur_step >= 3) {
            arrangemented_patch_cells.resize(pfd.arrangemented_patch_cells.rows());
            arrangemented_patch_area.resize(pfd.arrangemented_patch_cells.rows());
            arrangemented_patch_valid_area.resize(pfd.arrangemented_patch_cells.rows());
            arrangemented_patch_point_num.resize(pfd.arrangemented_patch_cells.rows());
            pruning_labels.resize(pfd.arrangemented_patch_cells.rows());
            for (int i = 0; i < pfd.arrangemented_patch_cells.rows(); i++) {
                const VecXi &row_vec = pfd.arrangemented_patch_cells.row(i);
                arrangemented_patch_cells[i] = std::vector<int>(row_vec.data(), row_vec.data() + row_vec.size());
                arrangemented_patch_area[i] = pfd.arrangemented_patch_area[i];
                arrangemented_patch_valid_area[i] = pfd.arrangemented_patch_valid_area[i];
                arrangemented_patch_point_num[i] = pfd.arrangemented_patch_point_num[i];
                pruning_labels[i] = pfd.pruning_labels[i];
            }
        }

        // element result_mesh_vertives
        std::vector<FLOAT> result_mesh_vertives_x;
        std::vector<FLOAT> result_mesh_vertives_y;
        std::vector<FLOAT> result_mesh_vertives_z;

        // element result_mesh_faces
        std::vector<std::vector<int>> result_mesh_faces;

        // element result_mesh_sharp_edge
        std::vector<std::vector<int>> result_mesh_sharp_edges;
        if(cur_step >= 4) {
            result_mesh_vertives_x.resize(pfd.result_mesh_vertices.rows());
            result_mesh_vertives_y.resize(pfd.result_mesh_vertices.rows());
            result_mesh_vertives_z.resize(pfd.result_mesh_vertices.rows());
            result_mesh_faces.resize(pfd.result_mesh_faces.rows());
            result_mesh_sharp_edges.resize(pfd.result_mesh_sharp_edges.rows());
            for(int i = 0; i < pfd.result_mesh_vertices.rows(); i++) {
                result_mesh_vertives_x[i] = pfd.result_mesh_vertices(i, 0);
                result_mesh_vertives_y[i] = pfd.result_mesh_vertices(i, 1);
                result_mesh_vertives_z[i] = pfd.result_mesh_vertices(i, 2);

                const VecXi &row_vec_faces = pfd.result_mesh_faces.row(i);
                result_mesh_faces[i] = std::vector<int>(row_vec_faces.data(), row_vec_faces.data() + row_vec_faces.size());

                const VecXi &row_vec_edges = pfd.result_mesh_sharp_edges.row(i);
                result_mesh_sharp_edges[i] = std::vector<int>(row_vec_edges.data(), row_vec_edges.data() + row_vec_edges.size());
            }
        }
        // Create an empty object
        happly::PLYData plyOut;

        // Add elements
        plyOut.addElement("step", 1);
        plyOut.getElement("step").addProperty<int>("step", {cur_step});
        if(cur_step >= 1) {
            plyOut.addElement("point_cloud", pfd.point_cloud.rows());
            plyOut.getElement("point_cloud").addProperty<FLOAT>("x", point_cloud_x);
            plyOut.getElement("point_cloud").addProperty<FLOAT>("y", point_cloud_y);
            plyOut.getElement("point_cloud").addProperty<FLOAT>("z", point_cloud_z);
            plyOut.getElement("point_cloud").addProperty<int>("segmentation", point_cloud_segmentation);

            plyOut.addElement("primitive", pfd.primitive_parameters.size());
            plyOut.getElement("primitive").addListProperty<FLOAT>("parameters", primitive_parameters);
            plyOut.getElement("primitive").addProperty<FLOAT>("red", primitive_colors_red);
            plyOut.getElement("primitive").addProperty<FLOAT>("green", primitive_colors_green);
            plyOut.getElement("primitive").addProperty<FLOAT>("blue", primitive_colors_blue);
        }

        if(cur_step >= 2) {
            plyOut.addElement("arrangemented_vertives", pfd.arrangemented_vertices.rows());
            plyOut.getElement("arrangemented_vertives").addProperty<FLOAT>("x", arrangemented_vertices_x);
            plyOut.getElement("arrangemented_vertives").addProperty<FLOAT>("y", arrangemented_vertices_y);
            plyOut.getElement("arrangemented_vertives").addProperty<FLOAT>("z", arrangemented_vertices_z);

            plyOut.addElement("arrangemented_faces", pfd.arrangemented_faces.rows());
            plyOut.getElement("arrangemented_faces").addListProperty<int>("indices", arrangemented_faces);
            plyOut.getElement("arrangemented_faces").addProperty<int>("patches", arrangemented_face_patches);
            plyOut.getElement("arrangemented_faces").addProperty<int>("labels", arrangemented_face_labels);
        }

        if(cur_step >= 3) {
            plyOut.addElement("arrangemented_patches", pfd.arrangemented_patch_cells.rows());
            plyOut.getElement("arrangemented_patches").addListProperty<int>("cells", arrangemented_patch_cells);
            plyOut.getElement("arrangemented_patches").addProperty<FLOAT>("area", arrangemented_patch_area);
            plyOut.getElement("arrangemented_patches").addProperty<FLOAT>("valid_area", arrangemented_patch_valid_area);
            plyOut.getElement("arrangemented_patches").addProperty<int>("point_num", arrangemented_patch_point_num);
            plyOut.getElement("arrangemented_patches").addProperty<int>("pruning_labels", pruning_labels);
        }

        if(cur_step >= 4) {
            plyOut.addElement("result_mesh_vertives", pfd.result_mesh_vertices.rows());
            plyOut.getElement("result_mesh_vertives").addProperty<FLOAT>("x", result_mesh_vertives_x);
            plyOut.getElement("result_mesh_vertives").addProperty<FLOAT>("y", result_mesh_vertives_y);
            plyOut.getElement("result_mesh_vertives").addProperty<FLOAT>("z", result_mesh_vertives_z);

            plyOut.addElement("result_mesh_faces", pfd.result_mesh_faces.rows());
            plyOut.getElement("result_mesh_faces").addListProperty("indices", result_mesh_faces);

            plyOut.addElement("result_mesh_sharp_edges", pfd.result_mesh_sharp_edges.rows());
            plyOut.getElement("result_mesh_sharp_edges").addListProperty<int>("indices", result_mesh_sharp_edges);
        }

        plyOut.write("my_output_file.ply", binary ? happly::DataFormat::Binary : happly::DataFormat::ASCII);

    }
    void load_primfit_data(const std::string& pfd_path, PrimFitData& pfd) {
        happly::PLYData plyIn(pfd_path.c_str());

        //element step
        int cur_step = plyIn.getElement("step").getProperty<int>("step")[0];
        pfd.step = cur_step;

        if(cur_step >= 1) {
            // element point_cloud
            std::vector<FLOAT> point_cloud_x, point_cloud_y, point_cloud_z;
            std::vector<int> point_cloud_segmentation;
            point_cloud_x = plyIn.getElement("point_cloud").getProperty<FLOAT>("x");
            point_cloud_y = plyIn.getElement("point_cloud").getProperty<FLOAT>("y");
            point_cloud_z = plyIn.getElement("point_cloud").getProperty<FLOAT>("z");
            point_cloud_segmentation = plyIn.getElement("point_cloud").getProperty<int>("segmentation");
            int num = point_cloud_x.size();
            pfd.point_cloud.resize(num, 3);
            for(int i = 0; i < num; i++) {
                pfd.point_cloud.row(i) << point_cloud_x[i], point_cloud_y[i], point_cloud_z[i];
                pfd.point_cloud_segmentation[i] = point_cloud_segmentation[i];
            }

            // element primitive
            std::vector<FLOAT> primitive_colors_r, primitive_colors_g, primitive_colors_b;
            pfd.primitive_parameters = plyIn.getElement("primitive").getListProperty<FLOAT>("parameters");
            primitive_colors_r = plyIn.getElement("primitive").getProperty<FLOAT>("red");
            primitive_colors_g = plyIn.getElement("primitive").getProperty<FLOAT>("green");
            primitive_colors_b = plyIn.getElement("primitive").getProperty<FLOAT>("blue");
            num = pfd.primitive_parameters.size();
            pfd.primitive_colors.resize(num, 3);
            for(int i = 0; i < num; i++) {
                pfd.primitive_colors.row(i) << primitive_colors_r[i], primitive_colors_g[i], primitive_colors_b[i];
            }
        }

        if(cur_step >= 2) {
            // element arrangemented_vertives
            std::vector<FLOAT> arrangemented_vertices_x, arrangemented_vertices_y, arrangemented_vertices_z;
            // element arrangement_faces
            std::vector<std::vector<int>> arrangemented_faces;
            std::vector<int> arrangemented_face_patches, arrangemented_face_labels;
            arrangemented_vertices_x = plyIn.getElement("arrangemented_vertives").getProperty<FLOAT>("x");
            arrangemented_vertices_y = plyIn.getElement("arrangemented_vertives").getProperty<FLOAT>("y");
            arrangemented_vertices_z = plyIn.getElement("arrangemented_vertives").getProperty<FLOAT>("z");
            int num = arrangemented_vertices_x.size();
            pfd.arrangemented_vertices.resize(num, 3);
            for(int i = 0; i < num; i++) {
                pfd.arrangemented_vertices.row(i) << arrangemented_vertices_x[i], arrangemented_vertices_y[i], arrangemented_vertices_z[i];
            }

            arrangemented_faces = plyIn.getElement("arrangemented_vertices").getListProperty<int>("indices");
            arrangemented_face_patches = plyIn.getElement("arrangemented_vertices").getProperty<int>("patches");
            arrangemented_face_labels = plyIn.getElement("arrangemented_vertices").getProperty<int>("labels");
            num = arrangemented_face_patches.size();
            pfd.arrangemented_faces.resize(num ,3);
            for(int i = 0; i < num; i++) {
                pfd.arrangemented_faces.row(i) << arrangemented_faces[i][0], arrangemented_faces[i][1], arrangemented_faces[i][2];
                pfd.arrangemented_face_patches[i] = arrangemented_face_patches[i];
                pfd.arrangemented_face_labels[i] = arrangemented_face_labels[i];
            }
        }

        if(cur_step >= 3) {
            // element arrangement_patches
            std::vector<std::vector<int>> arrangemented_patch_cells;
            std::vector<FLOAT> arrangemented_patch_area, arrangemented_patch_valid_area;
            std::vector<int> arrangemented_patch_point_num, pruning_labels;

            arrangemented_patch_cells = plyIn.getElement("arrangement_patches").getListProperty<int>("cells");
            arrangemented_patch_area = plyIn.getElement("arrangement_patches").getProperty<FLOAT>("area");
            arrangemented_patch_valid_area = plyIn.getElement("arrangement_patches").getProperty<FLOAT>("valid_area");
            arrangemented_patch_point_num = plyIn.getElement("arrangement_patches").getProperty<int>("point_num");
            pruning_labels = plyIn.getElement("arrangement_patches").getProperty<int>("pruning_labels");
            int num = arrangemented_patch_area.size();
            pfd.arrangemented_patch_cells.resize(num, 2);
            pfd.arrangemented_patch_area.resize(num);
            pfd.arrangemented_patch_valid_area.resize(num);
            pfd.arrangemented_patch_point_num.resize(num);
            pfd.pruning_labels.resize(num);
            for(int i = 0; i < num; i++) {
                pfd.arrangemented_patch_cells.row(i) << arrangemented_patch_cells[i][0], arrangemented_patch_cells[i][1];
                pfd.arrangemented_patch_area[i] = arrangemented_patch_area[i];
                pfd.arrangemented_patch_valid_area[i] = arrangemented_patch_valid_area[i];
                pfd.arrangemented_patch_point_num[i] = arrangemented_patch_point_num[i];
                pfd.pruning_labels[i] = pruning_labels[i];
            }
        }

        if(cur_step >= 4) {
            // element result_mesh_vertives
            std::vector<FLOAT> result_mesh_vertives_x;
            std::vector<FLOAT> result_mesh_vertives_y;
            std::vector<FLOAT> result_mesh_vertives_z;

            // element result_mesh_faces
            std::vector<std::vector<int>> result_mesh_faces;

            // element result_mesh_sharp_edge
            std::vector<std::vector<int>> result_mesh_sharp_edges;

            result_mesh_vertives_x = plyIn.getElement("result_mesh_vertives").getProperty<FLOAT>("x");
            result_mesh_vertives_y = plyIn.getElement("result_mesh_vertives").getProperty<FLOAT>("y");
            result_mesh_vertives_z = plyIn.getElement("result_mesh_vertives").getProperty<FLOAT>("z");

            result_mesh_faces = plyIn.getElement("result_mesh_faces").getListProperty<int>("indices");

            result_mesh_sharp_edges = plyIn.getElement("result_mesh_sharp_edge").getListProperty<int>("indices");

            int num = result_mesh_vertives_x.size();
            pfd.result_mesh_vertices.resize(num, 3);
            for(int i = 0; i < num; i++) {
                pfd.result_mesh_vertices.row(i) << result_mesh_vertives_x[i], result_mesh_vertives_y[i], result_mesh_vertives_z[i];
            }

            num = result_mesh_faces.size();
            pfd.result_mesh_faces.resize(num, 3);
            for(int i = 0; i < num; i++) {
                pfd.result_mesh_faces.row(i) << result_mesh_faces[i][0], result_mesh_faces[i][1], result_mesh_faces[i][2];
            }

            num = result_mesh_sharp_edges.size();
            pfd.result_mesh_sharp_edges.resize(num, 2);
            for(int i = 0; i < num; i++) {
                pfd.result_mesh_sharp_edges.row(i) << result_mesh_sharp_edges[i][0], result_mesh_sharp_edges[i][1];
            }
        }

    }

}


#endif //PRIMFIT_DATA_IO_H
