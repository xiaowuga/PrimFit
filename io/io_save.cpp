//
// Created by 小乌嘎 on 2023/1/13.
//

#include "io.h"

namespace PrimFit {
    namespace IO {

        bool save_segment(const std::string& file_name
                ,std::vector<int> type
                ,std::vector<easy3d::vec3>& pos
                ,std::vector<easy3d::vec3>& dir
                ,std::vector<float>& r1
                ,std::vector<float>& r2
                ,std::vector<easy3d::vec3>& points
                ,std::vector<easy3d::vec3>& normals
                ,std::vector<std::vector<int>>& segments
                ,std::vector<easy3d::vec3>& colors
                ,bool binary) {
            std::vector<Element> elements;

            Element element_vertex("vertex", points.size());
            if(!construct_element_vertex(element_vertex, points, normals, segments)) {
                return false;
            }

            Element element_shape("shapes", pos.size());
            if(!construct_element_shape(element_shape, type, pos, dir, r1, r2, colors)) {
                return false;
            }

            elements.emplace_back(element_vertex);
            elements.emplace_back(element_shape);

            easy3d::io::PlyWriter writer;
            return writer.write(file_name, elements, "segment", binary);
        }

        bool save_quadfit(const std::string& file_name
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
                ,std::vector<int>& arr_labels
                ,bool binary) {
            std::vector<Element>elements;
            Element element_step("step", 1);
            if(!construct_element_step(element_step, cur_step)) {
                return false;
            }
            elements.emplace_back(element_step);

//            if(cur_step >= 0) {
                Element element_vertex("vertex", points.size());
                if(!construct_element_vertex( element_vertex,points, normals, segments)) {
                    return false;
                }
                elements.emplace_back(element_vertex);

                Element element_shapes("shapes", pos.size());
                if(!construct_element_shape( element_shapes,type, pos,dir,r1,r2,colors)) {
                    return false;
                }
                elements.emplace_back(element_shapes);
//            }

            if(cur_step >= 1) {
                int num_points = 0;
                for(size_t i = 0; i < proxy_points.size(); i++) {
                    num_points += proxy_points[i].size();
                }
                Element element_proxy_points("proxy_vertex", num_points);
                if(!construct_element_proxy_points(element_proxy_points, proxy_points)) {
                    return false;
                }
                elements.emplace_back(element_proxy_points);

                int num_faces = 0;
                for(size_t i = 0; i < proxy_indices.size(); i++) {
                    num_faces += proxy_indices[i].size() / 3;
                }
                Element  element_proxy_indieces("proxy_face", num_faces);
                if(!construct_element_proxy_indices(element_proxy_indieces, proxy_indices)) {
                    return false;
                }
                elements.emplace_back(element_proxy_indieces);
            }

            if(cur_step >= 2) {
                Element element_arr_points("arr_vertex", arr_points.size());
                if(!construct_element_arr_points(element_arr_points, arr_points)) {
                    return false;
                }
                elements.emplace_back(element_arr_points);

                Element element_arr_faces("arr_face", arr_labels.size());
                if(!construct_element_arr_faces(element_arr_faces, arr_indices, arr_labels, arr_patches)) {
                    return false;
                }
                elements.emplace_back(element_arr_faces);

                Element element_arr_cells("arr_cell", arr_cells.size() / 2);
                if(!construct_element_arr_cells(element_arr_cells, arr_cells)) {
                    return false;
                }
                elements.emplace_back(element_arr_cells);
            }

            easy3d::io::PlyWriter writer;
            return writer.write(file_name, elements, "quadfit", binary);
        }

        bool save_pblp(const std::string& file_name
                ,std::vector<easy3d::vec3>& vertex
                ,std::vector<int>& face
                ,std::vector<int>& face_label
                ,std::vector<int>& face_patch
                ,std::vector<int>& face_point_num
                ,std::vector<float>& face_fit
                ,std::vector<float>& face_valid_area
                ,std::vector<float>& face_area
                ,std::vector<easy3d::vec3>& colors
                ,std::vector<int>& patch_label
                ,bool binary) {
            std::vector<Element> elements;

            Element element_vertex("vertex", vertex.size());
            if(!construct_element_pblp_vertex(element_vertex, vertex)) {
                return false;
            }
            elements.emplace_back(element_vertex);
            Element element_face("face", face_label.size());
            if(!construct_element_pblp_face(element_face, face, face_label, face_patch, face_point_num
                                            ,face_fit,face_valid_area, face_area, colors)) {
                return false;
            }
            elements.emplace_back(element_face);
            Element element_patch("patch", patch_label.size());
            if(!construct_element_pblp_patch(element_patch, patch_label)) {
                return false;
            }
            elements.emplace_back(element_patch);
            easy3d::io::PlyWriter writer;
            return writer.write(file_name, elements, "pblp", binary);
        }

    }
}