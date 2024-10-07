//
// Created by 小乌嘎 on 2023/1/6.
//

#ifndef QUADFIT_IO_H
#define QUADFIT_IO_H
#include <easy3d/core/types.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/fileio/ply_reader_writer.h>

namespace QuadFit {
    class SurfacePrimitive;
    namespace IO {
        typedef easy3d::io::Element Element;

        bool load_segment(const std::string& file_name
                          ,std::vector<int>& type
                          ,std::vector<easy3d::vec3>& pos
                          ,std::vector<easy3d::vec3>& dir
                          ,std::vector<float>& r1
                          ,std::vector<float>& r2
                          ,std::vector<easy3d::vec3>& points
                          ,std::vector<easy3d::vec3>& normals
                          ,std::vector<std::vector<int>>& segments
                          ,std::vector<easy3d::vec3>& colors);

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
                          ,bool binary = false);

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
                          ,std::vector<int>& arr_labels);

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
                ,bool binary = false);

        bool load_pblp(const std::string& file_name
                       ,std::vector<easy3d::vec3>& vertex
                       ,std::vector<int>& face
                       ,std::vector<int>& face_label
                       ,std::vector<int>& face_patch
                       ,std::vector<int>& face_point
                       ,std::vector<float>& face_fit
                       ,std::vector<float>& face_valid_area
                       ,std::vector<float>& face_area
                       ,std::vector<easy3d::vec3>& colors
                       ,std::vector<int>& patch_label);

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
                       ,bool binary = false);

        bool save_outline(const std::string& file_name
                          ,std::vector<easy3d::vec3>& edge_points
                          ,std::vector<std::vector<int>> edge_lines
                          ,bool binary = false);

        bool load_step(Element& element_step, int& step);


        bool load_vertex(Element& element_vertex
                         ,std::vector<easy3d::vec3>& vertices
                         ,std::vector<easy3d::vec3>& normals
                         ,std::vector<std::vector<int>>& segments);

        bool load_shape(Element& element_shapes
                ,std::vector<int>& type
                ,std::vector<easy3d::vec3>& pos
                ,std::vector<easy3d::vec3>& dir
                ,std::vector<float>& r1
                ,std::vector<float>& r2
                ,std::vector<easy3d::vec3>& color);

        bool load_proxy_points(Element& element_proxy_points
                ,std::vector<std::vector<easy3d::vec3>>& proxy_points);

        bool load_proxy_indices(Element& element_proxy_indices
                ,std::vector<std::vector<int>>& proxy_indices);

        bool load_arr_points(Element& element_arr_points
                ,std::vector<easy3d::vec3>& arr_points);

        bool load_arr_faces(Element& element_arr_faces
                ,std::vector<int>& arr_indices
                ,std::vector<int>& arr_labels
                ,std::vector<int>& arr_patches);

        bool load_arr_cells(Element& element_arr_cells
                            ,std::vector<int>& arr_cells);

        bool load_pblp_vertex(Element& element_vertex
                              ,std::vector<easy3d::vec3>& vertex);

        bool load_pblp_face(Element& element_face
                            ,std::vector<int>& face
                            ,std::vector<int>& face_label
                            ,std::vector<int>& face_patch
                            ,std::vector<int>& face_point_num
                            ,std::vector<float>& face_fit
                            ,std::vector<float>& face_valid_area
                            ,std::vector<float>& face_area
                            ,std::vector<easy3d::vec3>& colors);

        bool load_pblp_patch(Element& element_patch
                             ,std::vector<int>& patch_label);

        bool construct_element_step(Element& element_step, int& step);

        bool construct_element_vertex(Element& element_vertex
                                      ,std::vector<easy3d::vec3>& vertices
                                      ,std::vector<easy3d::vec3>& normals
                                      ,std::vector<std::vector<int>>& segments);

        bool construct_element_shape(Element& element_shapes
                                     ,std::vector<int>& type
                                     ,std::vector<easy3d::vec3>& pos
                                     ,std::vector<easy3d::vec3>& dir
                                     ,std::vector<float>& r1
                                     ,std::vector<float>& r2
                                     ,std::vector<easy3d::vec3>& color);

        bool construct_element_proxy_points(Element& element_proxy_points
                                            ,std::vector<std::vector<easy3d::vec3>>& proxy_points);

        bool construct_element_proxy_indices(Element& element_proxy_indices
                                             ,std::vector<std::vector<int>>& proxy_indices);



        bool construct_element_arr_points(Element& element_arr_points
                                          ,std::vector<easy3d::vec3>& arr_points);

        bool construct_element_arr_faces(Element& element_arr_faces
                                     ,std::vector<int>& arr_indices
                                     ,std::vector<int>& arr_labels
                                     ,std::vector<int>& arr_patches);

        bool construct_element_arr_cells(Element& element_arr_cells
                                         ,std::vector<int>& arr_cells);

        bool construct_element_pblp_vertex(Element& element_vertex
                                           ,std::vector<easy3d::vec3>& vertex);

        bool construct_element_pblp_face(Element& element_face
                                         ,std::vector<int>& face
                                         ,std::vector<int>& face_label
                                         ,std::vector<int>& face_patch
                                         ,std::vector<int>& face_point_num
                                         ,std::vector<float>& face_fit
                                         ,std::vector<float>& face_valid_area
                                         ,std::vector<float>& face_area
                                         ,std::vector<easy3d::vec3>& colors);

        bool construct_element_pblp_patch(Element& element_patch
                                          ,std::vector<int>& patch_label);


        namespace details {
            template<typename PropertyT>
            inline bool
            extract_named_property(std::vector<PropertyT> &properties, PropertyT &wanted, const std::string &name) {
                typename std::vector<PropertyT>::iterator it = properties.begin();
                for (; it != properties.end(); ++it) {
                    const PropertyT &prop = *it;
                    if (prop.name == name) {
                        wanted = prop;
                        properties.erase(it);
                        return true;
                    }
                }
                return false;
            }

            template<typename PropertyT>
            inline bool extract_vector_property(std::vector<PropertyT> &properties,
                                                const std::string &x_name, const std::string &y_name,
                                                const std::string &z_name,
                                                easy3d::io::Vec3Property &prop) {
                PropertyT x_coords, y_coords, z_coords;
                if (details::extract_named_property(properties, x_coords, x_name) &&
                    details::extract_named_property(properties, y_coords, y_name) &&
                    details::extract_named_property(properties, z_coords, z_name)) {
                    std::size_t num = x_coords.size();
                    prop.resize(num);
                    for (std::size_t j = 0; j < num; ++j)
                        prop[j] = easy3d::vec3(
                                static_cast<float>(x_coords[j]),
                                static_cast<float>(y_coords[j]),
                                static_cast<float>(z_coords[j])
                        );
                    return true;
                } else
                    return false;
            }

            inline bool extract_element(std::map<std::string, int>& check_element
                    ,std::vector<Element>& elements) {

                for(size_t i = 0; i < elements.size(); i++) {
                    if(check_element.find(elements[i].name) == check_element.end()) {
                        std::cout << "Found a  invalid element named :" << elements[i].name
                                  << " in file." << std::endl;
                        return false;
                    } else {
                        check_element[elements[i].name] = i;
                    }
                }
                for(auto& item : check_element) {
                    if(item.second >= 0) {
                        std::cout << "Found element " << item.first << std::endl;
                    } else {
                        std::cout << "Element " << item.first << " is not found" << std::endl;
                    }
                }

                return true;
            }
        }

    }
}


#endif //QUADFIT_IO_H
