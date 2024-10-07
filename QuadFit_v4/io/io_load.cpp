//
// Created by 小乌嘎 on 2023/1/11.
//

#include "io.h"

namespace QuadFit {
    namespace IO {

        bool load_step(Element &element_step, int &step) {
            if (element_step.name != "step" && element_step.num_instances != 1) {
                return false;
            }
            easy3d::io::IntProperty tmp;
            if (!details::extract_named_property(element_step.int_properties, tmp, "step")) {
                return false;
            }

            step = tmp[0];
            return true;
        }

        bool
        load_vertex(Element &element_vertex, std::vector<easy3d::vec3> &vertices, std::vector<easy3d::vec3> &normals,
                    std::vector<std::vector<int>> &segments) {
            if (element_vertex.name != "vertex") {
                return false;
            }

            easy3d::io::Vec3Property points;
            easy3d::io::IntProperty seg_labels;
            if (details::extract_named_property(element_vertex.vec3_properties, points, "point")
                && details::extract_named_property(element_vertex.int_properties, seg_labels, "segment")) {
                vertices.resize(points.size());
                for (size_t j = 0; j < points.size(); j++) {
                    vertices[j] = points[j];
                }
                int segment_num = 0;
                for (size_t j = 0; j < seg_labels.size(); j++) {
                    segment_num = std::max(segment_num, seg_labels[j]);
                }
                segment_num += 1;
                segments.clear();
                segments.resize(segment_num);

                for (size_t j = 0; j < seg_labels.size(); j++) {
                    int idx = seg_labels[j];
                    if (idx >= 0) {
                        segments[idx].emplace_back(j);
                    }
                }
            } else {
                return false;
            }

            easy3d::io::Vec3Property norm;
            norm.clear();
            if (details::extract_named_property(element_vertex.vec3_properties, norm, "normal")) {
                if (!norm.empty()) {
                    normals.resize(norm.size());
                    for (size_t j = 0; j < norm.size(); j++) {
                        normals[j] = norm[j];
                    }
                }
            }
            return true;
        }

        bool load_shape(Element &element_shapes, std::vector<int> &type, std::vector<easy3d::vec3> &pos,
                        std::vector<easy3d::vec3> &dir, std::vector<float> &r1, std::vector<float> &r2,
                        std::vector<easy3d::vec3> &color) {
            if (element_shapes.name != "shapes") {
                return false;
            }

            easy3d::io::Vec3Property pos_list;
            easy3d::io::Vec3Property dir_list;
            easy3d::io::Vec3Property color_list;
            easy3d::io::FloatProperty r1_list;
            easy3d::io::FloatProperty r2_list;
            easy3d::io::IntProperty type_list;

            if (details::extract_vector_property(element_shapes.float_properties, "pos_x", "pos_y", "pos_z", pos_list)
                &&
                details::extract_vector_property(element_shapes.float_properties, "dir_x", "dir_y", "dir_z", dir_list)
                && details::extract_named_property(element_shapes.vec3_properties, color_list, "color")
                && details::extract_named_property(element_shapes.float_properties, r1_list, "r1")
                && details::extract_named_property(element_shapes.float_properties, r2_list, "r2")
                && details::extract_named_property(element_shapes.int_properties, type_list, "type")) {
                pos.resize(pos_list.size());
                for (size_t i = 0; i < pos.size(); i++) {
                    pos[i] = pos_list[i];
                }
                dir.resize(dir_list.size());
                for (size_t i = 0; i < dir.size(); i++) {
                    dir[i] = dir_list[i].normalize();
                }

                color.resize(color_list.size());
                for (size_t i = 0; i < color.size(); i++) {
                    color[i] = color_list[i];
                }

                r1.resize(r1_list.size());
                for (size_t i = 0; i < r1.size(); i++) {
                    r1[i] = r1_list[i];
                }

                r2.resize(r2_list.size());
                for (size_t i = 0; i < r2.size(); i++) {
                    r2[i] = r2_list[i];
                }

                type.resize(type_list.size());
                for (size_t i = 0; i < type.size(); i++) {
                    type[i] = type_list[i];
                }

                return true;
            } else {
                return false;
            }

        }


        bool load_proxy_points(Element &element_proxy_points, std::vector<std::vector<easy3d::vec3>> &proxy_points) {
            if (element_proxy_points.name != "proxy_vertex") {
                return false;
            }
            easy3d::io::Vec3Property points;
            easy3d::io::IntProperty point_labels;
            if (details::extract_named_property(element_proxy_points.vec3_properties, points, "point")
                && details::extract_named_property(element_proxy_points.int_properties, point_labels, "label")) {
                int num = 0;
                for (size_t i = 0; i < point_labels.size(); i++) {
                    num = std::max(num, point_labels[i]);
                }
                num += 1;
                proxy_points.clear();
                proxy_points.resize(num);
                for (int i = 0; i < point_labels.size(); i++) {
                    int id = point_labels[i];
                    proxy_points[id].emplace_back(points[i]);
                }
                return true;
            } else {
                return false;
            }
        }

        bool load_proxy_indices(Element &element_proxy_indices, std::vector<std::vector<int>> &proxy_indices) {
            if (element_proxy_indices.name != "proxy_face") {
                return false;
            }
            easy3d::io::IntListProperty face;
            easy3d::io::IntProperty label;
            if (details::extract_named_property(element_proxy_indices.int_list_properties, face, "vertex_index")
                && details::extract_named_property(element_proxy_indices.int_properties, label, "label")) {
                int num = 0;
                for (size_t i = 0; i < label.size(); i++) {
                    num = std::max(num, label[i]);
                }
                num += 1;

                proxy_indices.clear();
                proxy_indices.resize(num);
                for (size_t i = 0; i < label.size(); i++) {
                    int idx = label[i];
                    for (size_t j = 0; j < face[i].size(); j++) {
                        proxy_indices[idx].emplace_back(face[i][j]);
                    }
                }
                return true;
            } else {
                return false;
            }
        }

        bool load_arr_points(Element &element_arr_points, std::vector<easy3d::vec3> &arr_points) {
            if (element_arr_points.name != "arr_vertex") {
                return false;
            }
            easy3d::io::Vec3Property points;
            if (details::extract_named_property(element_arr_points.vec3_properties, points, "point")) {
                arr_points.clear();
                arr_points.resize(points.size());
                for (size_t i = 0; i < arr_points.size(); i++) {
                    arr_points[i] = points[i];
                }
                return true;
            } else {
                return false;
            }
        }

        bool load_arr_faces(Element &element_arr_faces, std::vector<int> &arr_indices, std::vector<int> &arr_labels,
                            std::vector<int> &arr_patches) {
            if (element_arr_faces.name != "arr_face") {
                return false;
            }

            easy3d::io::IntListProperty face;
            easy3d::io::IntProperty label;
            easy3d::io::IntProperty patch;

            if (details::extract_named_property(element_arr_faces.int_list_properties, face, "vertex_index")
                && details::extract_named_property(element_arr_faces.int_properties, label, "label")
                && details::extract_named_property(element_arr_faces.int_properties, patch, "patch")) {
                arr_labels.clear();
                arr_labels.resize(label.size());
                for (size_t i = 0; i < arr_labels.size(); i++) {
                    arr_labels[i] = label[i];
                }

                arr_patches.clear();
                arr_patches.resize(patch.size());
                for (size_t i = 0; i < arr_patches.size(); i++) {
                    arr_patches[i] = patch[i];
                }

                arr_indices.clear();
                for (size_t i = 0; i < face.size(); i++) {
                    for (size_t j = 0; j < face[i].size(); j++) {
                        arr_indices.emplace_back(face[i][j]);
                    }
                }

                return true;
            } else {
                return false;
            }
        }

        bool load_arr_cells(Element &element_arr_cells, std::vector<int> &arr_cells) {
            if (element_arr_cells.name != "arr_cell") {
                return false;
            }

            easy3d::io::IntListProperty cells;
            if (details::extract_named_property(element_arr_cells.int_list_properties, cells, "patch_index")) {
                for (size_t i = 0; i < cells.size(); i++) {
                    for (size_t j = 0; j < cells[i].size(); j++) {
                        arr_cells.emplace_back(cells[i][j]);
                    }
                }
                return true;
            } else {
                return false;
            }
        }

        bool load_pblp_vertex(Element &element_vertex, std::vector<easy3d::vec3> &vertex) {
            if (element_vertex.name != "vertex") {
                return false;
            }

            easy3d::io::Vec3Property points;
            if (details::extract_named_property(element_vertex.vec3_properties, points, "point")) {
                int num = points.size();
                vertex.resize(num);
                for (int i = 0; i < num; i++) {
                    vertex[i] = points[i];
                }
                return true;
            } else {
                return false;
            }

        }

        bool load_pblp_face(Element &element_face, std::vector<int> &face, std::vector<int> &face_label,
                            std::vector<int> &face_patch, std::vector<int> &face_point_num,
                            std::vector<float> &face_fit, std::vector<float> &face_valid_area,
                            std::vector<float> &face_area, std::vector<easy3d::vec3> &colors) {
            if (element_face.name != "face") {
                return false;
            }
            easy3d::io::IntListProperty face_;
            easy3d::io::IntProperty face_label_;
            easy3d::io::IntProperty face_patch_;
            easy3d::io::IntProperty face_point_num_;
            easy3d::io::FloatProperty face_fit_;
            easy3d::io::FloatProperty face_valid_area_;
            easy3d::io::FloatProperty face_area_;
            easy3d::io::Vec3Property colors_;
            if(details::extract_named_property(element_face.int_list_properties, face_, "vertex_index")
                && details::extract_named_property(element_face.int_properties, face_label_, "label")
                && details::extract_named_property(element_face.int_properties, face_patch_, "patch")
                && details::extract_named_property(element_face.int_properties, face_point_num_, "point_num")
                && details::extract_named_property(element_face.float_properties, face_fit_, "fit")
                && details::extract_named_property(element_face.float_properties, face_valid_area_, "valid_area")
                && details::extract_named_property(element_face.float_properties, face_area_, "area")
                && details::extract_named_property(element_face.vec3_properties, colors_, "color")) {
                for (size_t i = 0; i < face_.size(); i++) {
                    for (size_t j = 0; j < face_[i].size(); j++) {
                        face.emplace_back(face_[i][j]);
                    }
                }

                face_label.resize(face_label_.size());
                for (size_t i = 0; i < face_label.size(); i++) {
                    face_label[i] = face_label_[i];
                }

                face_patch.resize(face_patch_.size());
                for (size_t i = 0; i < face_patch.size(); i++) {
                    face_patch[i] = face_patch_[i];
                }

                face_point_num.resize(face_point_num_.size());
                for (size_t i = 0; i < face_point_num.size(); i++) {
                    face_point_num[i] = face_point_num_[i];
                }

                face_fit.resize(face_fit_.size());
                for (size_t i = 0; i < face_fit.size(); i++) {
                    face_fit[i] = face_fit_[i];
                }

                face_valid_area.resize(face_valid_area_.size());
                for (size_t i = 0; i < face_valid_area.size(); i++) {
                    face_valid_area[i] = face_valid_area_[i];
                }

                face_area.resize(face_area_.size());
                for (size_t i = 0; i < face_area.size(); i++) {
                    face_area[i] = face_area_[i];
                }

                colors.resize(colors_.size());
                for (size_t i = 0; i < colors.size(); i++) {
                    colors[i] = colors_[i];
                }
                return true;
            } else {
                return false;
            }
        }


        bool load_pblp_patch(Element& element_patch
                             ,std::vector<int>& patch_label) {
            if (element_patch.name != "patch") {
                return false;
            }

            easy3d::io::IntProperty label;
            if(details::extract_named_property(element_patch.int_properties, label, "label")) {
                patch_label.resize(label.size());
                for(size_t i = 0; i < patch_label.size(); i++) {
                    patch_label[i] = label[i];
                }
                return true;
            } else {
                return false;
            }
        }

        bool load_segment(const std::string& file_name
                ,std::vector<int>& type
                ,std::vector<easy3d::vec3>& pos
                ,std::vector<easy3d::vec3>& dir
                ,std::vector<float>& r1
                ,std::vector<float>& r2
                ,std::vector<easy3d::vec3>& points
                ,std::vector<easy3d::vec3>& normals
                ,std::vector<std::vector<int>>& segments
                ,std::vector<easy3d::vec3>& colors) {
            std::vector<Element> elements;
            easy3d::io::PlyReader reader;
            if (!reader.read(file_name, elements))
                return false;
            std::map<std::string, int> check_element;
            check_element["vertex"] = -1;
            check_element["shapes"] = -1;
            if(details::extract_element(check_element, elements)) {
                int id1 = check_element["vertex"];
                if(!load_vertex(elements[id1], points, normals, segments)) {
                    return false;
                }
                int id2 = check_element["shapes"];
                if(!load_shape(elements[id2], type, pos, dir, r1, r2, colors)) {
                    return false;
                }
                return true;
            } else {
                return false;
            }

        }


        bool load_quadfit(const std::string& file_name
                ,int& cur_step
                ,std::vector<int>& type
                ,std::vector<easy3d::vec3>& pos
                ,std::vector<easy3d::vec3>& dir
                ,std::vector<float>& r1
                ,std::vector<float>& r2
                ,std::vector<easy3d::vec3>& points
                ,std::vector<easy3d::vec3>& normals
                ,std::vector<std::vector<int>>& segments
                ,std::vector<easy3d::vec3>& colors
                ,std::vector<std::vector<easy3d::vec3>>& proxy_points
                ,std::vector<std::vector<int>>& proxy_indices
                ,std::vector<easy3d::vec3>& arr_points
                ,std::vector<int>& arr_indices
                ,std::vector<int>& arr_cells
                ,std::vector<int>& arr_patches
                ,std::vector<int>& arr_labels) {
            std::vector<Element> elements;
            easy3d::io::PlyReader reader;
            if (!reader.read(file_name, elements))
                return false;

            std::map<std::string, int> check_element;
            check_element["step"] = -1;
            check_element["vertex"] = -1;
            check_element["shapes"] = -1;
            check_element["proxy_vertex"] = -1;
            check_element["proxy_face"] = -1;
            check_element["arr_vertex"] = -1;
            check_element["arr_face"] = -1;
            check_element["arr_cell"] = -1;
            if(!details::extract_element(check_element, elements)) {
                return false;
            }
            int id_step = check_element["step"];
            cur_step = -1;
            if(!load_step(elements[id_step], cur_step)) {
                return false;
            }

            if(cur_step >= 0) {
                int id1 = check_element["vertex"];
                int id2 = check_element["shapes"];
                if(!load_vertex(elements[id1] ,points, normals, segments)) {
                    return false;
                }
                if(!load_shape(elements[id2], type, pos, dir, r1,r2, colors)) {
                    return false;
                }
            }

            if(cur_step >= 1) {
                int id1 = check_element["proxy_vertex"];
                int id2 = check_element["proxy_face"];
                if(!load_proxy_points(elements[id1], proxy_points)) {
                    return false;
                }
                if(!load_proxy_indices(elements[id2], proxy_indices)) {
                    return false;
                }
            }

            if(cur_step >= 2) {
                int id1 = check_element["arr_vertex"];
                int id2 = check_element["arr_face"];
                int id3 = check_element["arr_cell"];
                if(!load_arr_points(elements[id1], arr_points)) {
                    return false;
                }
                if(!load_arr_faces(elements[id2], arr_indices, arr_labels, arr_patches)) {
                    return false;
                }
                if(!load_arr_cells(elements[id3], arr_cells)) {
                    return false;
                }
                return true;
            }
        }

        bool load_pblp(const std::string& file_name
                ,std::vector<easy3d::vec3>& vertex
                ,std::vector<int>& face
                ,std::vector<int>& face_label
                ,std::vector<int>& face_patch
                ,std::vector<int>& face_point_num
                ,std::vector<float>& face_fit
                ,std::vector<float>& face_valid_area
                ,std::vector<float>& face_area
                ,std::vector<easy3d::vec3>& colors
                ,std::vector<int>& patch_label) {
            std::vector<Element> elements;
            easy3d::io::PlyReader reader;
            if (!reader.read(file_name, elements))
                return false;

            std::map<std::string, int> check_element;
            check_element["vertex"] = -1;
            check_element["face"] = -1;
            check_element["patch"] = -1;

            if(!details::extract_element(check_element, elements)) {
                return false;
            }

            int id1 = check_element["vertex"];
            int id2 = check_element["face"];
            int id3 = check_element["patch"];
            if(!load_pblp_vertex(elements[id1], vertex)) {
                return false;
            }
            if(!load_pblp_face(elements[id2], face, face_label, face_patch
                               ,face_point_num, face_fit, face_valid_area, face_area
                               ,colors)) {
                return false;
            }

            if(!load_pblp_patch(elements[id3], patch_label)) {
                return false;
            }
            return true;
        }
    }
}