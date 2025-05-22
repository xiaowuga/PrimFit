//
// Created by 小乌嘎 on 2023/1/12.
//
#include "io.h"

namespace QuadFit {
    namespace IO {
        bool construct_element_step(Element& element_step, int& step) {
            easy3d::io::IntProperty tmp;
            tmp.name = "step";
            tmp.resize(1);
            tmp[0] = step;
            element_step.int_properties.emplace_back(tmp);
            return true;
        }

        bool construct_element_vertex(Element& element_vertex
                ,std::vector<easy3d::vec3>& vertices
                ,std::vector<easy3d::vec3>& normals
                ,std::vector<std::vector<int>>& segments) {
            if(normals.size() != 0 && normals.size() == element_vertex.num_instances) {
                element_vertex.vec3_properties.emplace_back(
                        easy3d::io::GenericProperty<easy3d::vec3>("normal", normals));
            }
            if(vertices.size() != 0 && vertices.size() == element_vertex.num_instances) {
                element_vertex.vec3_properties.emplace_back(
                        easy3d::io::GenericProperty<easy3d::vec3>("point", vertices));
                easy3d::io::IntProperty seg_labels;
                seg_labels.name = "segment";
                seg_labels.resize(vertices.size(), -1);
                for(size_t i = 0; i < segments.size(); i++) {
                    for(size_t j = 0; j < segments[i].size(); j++) {
                        int idx = segments[i][j];
                        seg_labels[idx] = i;
                    }
                }
                element_vertex.int_properties.emplace_back(seg_labels);
                return true;
            } else {
                return false;
            }

        }

        bool construct_element_shape(Element& element_shapes
                ,std::vector<int>& type
                ,std::vector<easy3d::vec3>& pos
                ,std::vector<easy3d::vec3>& dir
                ,std::vector<float>& r1
                ,std::vector<float>& r2
                ,std::vector<easy3d::vec3>& color) {
            if(type.size() == element_shapes.num_instances
               && pos.size() == element_shapes.num_instances
               && dir.size() == element_shapes.num_instances
               && r1.size() == element_shapes.num_instances
               && r2.size() == element_shapes.num_instances
               && color.size() == element_shapes.num_instances) {
                element_shapes.int_properties.emplace_back(
                        easy3d::io::GenericProperty<int>("type", type));
                element_shapes.vec3_properties.emplace_back(
                        easy3d::io::GenericProperty<easy3d::vec3>("pos", pos));
                element_shapes.vec3_properties.emplace_back(
                        easy3d::io::GenericProperty<easy3d::vec3>("dir", dir));

                element_shapes.float_properties.emplace_back(
                        easy3d::io::GenericProperty<float>("r1", r1));

                element_shapes.float_properties.emplace_back(
                        easy3d::io::GenericProperty<float>("r2", r2));

                element_shapes.vec3_properties.emplace_back(easy3d::io::GenericProperty<easy3d::vec3>("color", color));

                return true;
            } else {
                return false;
            }
        }

        bool construct_element_proxy_points(Element& element_proxy_points
                ,std::vector<std::vector<easy3d::vec3>>& proxy_points) {
            easy3d::io::Vec3Property points;
            points.name = "point";
            easy3d::io::IntProperty labels;
            labels.name = "label";
            for(size_t i = 0; i < proxy_points.size(); i++) {
                for(size_t j = 0; j < proxy_points[i].size(); j++) {
                    points.emplace_back(proxy_points[i][j]);
                    labels.emplace_back(i);
                }
            }

            element_proxy_points.vec3_properties.emplace_back(points);
            element_proxy_points.int_properties.emplace_back(labels);
            return true;
        }

        bool construct_element_proxy_indices(Element& element_proxy_indices
                ,std::vector<std::vector<int>>& proxy_indices) {
            easy3d::io::IntListProperty face;
            face.name = "vertex_index";
            easy3d::io::IntProperty label;
            label.name = "label";

            for(size_t i = 0; i < proxy_indices.size(); i++) {
                for(size_t j = 0; j < proxy_indices[i].size(); j += 3) {
                    std::vector<int> indices;
                    for(size_t k = 0; k < 3; k++) {
                        indices.emplace_back(proxy_indices[i][j + k]);
                    }
                    face.emplace_back(indices);
                    label.emplace_back(i);
                }
            }

            element_proxy_indices.int_list_properties.emplace_back(face);
            element_proxy_indices.int_properties.emplace_back(label);
            return true;
        }

        bool construct_element_arr_points(Element& element_arr_points
                ,std::vector<easy3d::vec3>& arr_points) {
            element_arr_points.vec3_properties.emplace_back(
                    easy3d::io::GenericProperty<easy3d::vec3>("point", arr_points));
            return true;
        }

        bool construct_element_arr_faces(Element& element_arr_faces
                ,std::vector<int>& arr_indices
                ,std::vector<int>& arr_labels
                ,std::vector<int>& arr_patches) {
            easy3d::io::IntListProperty face;
            face.name = "vertex_index";
            easy3d::io::IntProperty labels;
            labels.name = "label";
            easy3d::io::IntProperty patches;
            patches.name = "patch";
            for(size_t i = 0; i < arr_indices.size(); i += 3) {
                std::vector<int>indices;
                for(size_t j = 0; j < 3; j++) {
                    indices.emplace_back(arr_indices[i + j]);
                }
                face.emplace_back(indices);
            }

            element_arr_faces.int_list_properties.emplace_back(face);

            labels.resize(arr_labels.size());
            for(size_t i = 0; i < labels.size(); i++) {
                labels[i] = arr_labels[i];
            }
            element_arr_faces.int_properties.emplace_back(labels);
            patches.resize(arr_patches.size());
            for(size_t i = 0; i < patches.size(); i++) {
                patches[i] = arr_patches[i];
            }
            element_arr_faces.int_properties.emplace_back(patches);
            return true;
        }

        bool construct_element_arr_cells(Element& element_arr_cells
                ,std::vector<int>& arr_cells) {
            easy3d::io::IntListProperty cells;
            cells.name = "patch_index";

            for(size_t i = 0; i < arr_cells.size(); i += 2) {
                std::vector<int>indices;
                for(size_t j = 0; j < 2; j++) {
                    indices.emplace_back(arr_cells[i + j]);
                }
                cells.emplace_back(indices);
            }
            element_arr_cells.int_list_properties.emplace_back(cells);
            return true;
        }

        bool construct_element_pblp_vertex(Element& element_vertex
                                           ,std::vector<easy3d::vec3>& vertex) {
            element_vertex.vec3_properties.emplace_back(
                    easy3d::io::GenericProperty<easy3d::vec3>("point", vertex));

            return true;
        }

        bool construct_element_pblp_face(Element& element_face
                                         ,std::vector<int>& face
                                         ,std::vector<int>& face_label
                                         ,std::vector<int>& face_patch
                                         ,std::vector<int>& face_point_num
                                         ,std::vector<float>& face_fit
                                         ,std::vector<float>& face_valid_area
                                         ,std::vector<float>& face_area
                                         ,std::vector<easy3d::vec3>& colors) {
            easy3d::io::IntListProperty face_;
            face_.name = "face";
            for(size_t i = 0; i < face.size(); i += 3) {
                std::vector<int> indices;
                for(size_t j = 0; j < 3; j++) {
                    indices.emplace_back(face[i + j]);
                }
            }
            element_face.int_properties.emplace_back(
                    easy3d::io::GenericProperty<int>("label", face_label));

            element_face.int_properties.emplace_back(
                    easy3d::io::GenericProperty<int>("patch", face_patch));

            element_face.int_properties.emplace_back(
                    easy3d::io::GenericProperty<int>("point_num", face_point_num));

            element_face.float_properties.emplace_back(
                    easy3d::io::GenericProperty<float>("fit", face_fit));

            element_face.float_properties.emplace_back(
                    easy3d::io::GenericProperty<float>("valid_area", face_valid_area));
            element_face.float_properties.emplace_back(
                    easy3d::io::GenericProperty<float>("area", face_area));

            element_face.vec3_properties.emplace_back(
                    easy3d::io::GenericProperty<easy3d::vec3>("color", colors));
            return true;
        }

        bool construct_element_pblp_patch(Element& element_patch
                                          ,std::vector<int>& patch_label) {
            element_patch.int_properties.emplace_back(
                    easy3d::io::GenericProperty<int>("label", patch_label));
            return true;
        }

    }
}
