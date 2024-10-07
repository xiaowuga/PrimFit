//
// Created by 小乌嘎 on 2023/1/24.
//

#include <partition_simplifier2.h>
#include <io.h>
#include <easy3d/util/stop_watch.h>

int main() {
    std::string input_path = "/Users/xiaowuga/Desktop/siggraph2023/appendix/PC190/Ours/PC190_arrangement.qf";
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
    QuadFit::IO::load_quadfit(input_path, cur_step
            ,type, pos, dir, r1, r2
            ,points, normals, segments, colors
            ,proxy_points, proxy_indices
            ,arr_points, arr_indices, arr_cells, arr_patches, arr_labels);

    MatrixDr m_vertives;
    MatrixIr m_faces;
    VectorI m_face_labels;
    MatrixIr m_cells;
    VectorI m_patches;
    QuadFit::Util::construct_arrangement_from_std(m_vertives, m_faces, m_face_labels
                                                  ,m_cells, m_patches
                                                  ,arr_points, arr_indices
                                                  ,arr_labels, arr_patches
                                                  ,arr_cells);
    MatrixDr m_points;
    QuadFit::Util::construct_vertices_list_from_std_to_igl(m_points, points);
    QuadFit::Partition_Simplifier2 ps;
    easy3d::StopWatch w; w.start();

    ps.init(m_points, m_vertives, m_faces
            , m_patches, m_face_labels,segments, 0.02);
    ps.simplify(0.5);
    std::cout << w.elapsed_seconds(3) <<std::endl;
    VectorI m_face_point_num = ps.m_face_point_num;
    VectorD m_face_fit = ps.m_face_fit;
    VectorD m_face_valid_area = ps.m_face_valid_area;
    VectorD m_face_area = ps.m_face_area;
    VectorI m_patch_label = ps.m_patch_label;
    int num_face = m_faces.rows();
    MatrixDr m_face_color; m_face_color.resize(num_face, 3);
    for(int i = 0; i < num_face; i++) {
        int label = m_face_labels[i];
        m_face_color.row(i) = QuadFit::Util::construct_vertices_from_easy3d_to_igl(colors[label]);
    }
    int ct = 0;
    for(int i = 0; i < m_patch_label.size(); i++) {
        if(m_patch_label[i]) ct++;
    }
    std::cout <<m_patch_label.rows() <<' ' << ct <<std::endl;
//    return 0;
    std::vector<easy3d::vec3> o_points;
    std::vector<int> o_face;
    std::vector<int> o_face_label;
    std::vector<int> o_face_patch;
    std::vector<int> o_face_point_num;
    std::vector<float> o_face_fit;
    std::vector<float> o_face_valid_area;
    std::vector<float> o_face_area;
    std::vector<easy3d::vec3> o_colors;
    std::vector<int> o_patch_label;
    QuadFit::Util::construct_std_from_pblp(m_vertives, m_faces
                                           , m_face_labels, m_patches
                                           ,m_face_point_num, m_face_fit
                                           ,m_face_valid_area, m_face_area
                                           ,m_face_color,m_patch_label
                                           ,o_points, o_face, o_face_label
                                           ,o_face_patch, o_face_point_num
                                           ,o_face_fit, o_face_valid_area
                                           ,o_face_area, o_colors, o_patch_label);

    std::string output_path = "/Users/xiaowuga/Desktop/siggraph2023/experiment/broken/broken1/PC13_broken1.pblp";
//    QuadFit::IO::save_pblp(output_path, o_points, o_face
//                           ,o_face_label, o_face_patch
//                           ,o_face_point_num, o_face_fit, o_face_valid_area
//                           ,o_face_area, o_colors, o_patch_label);
    return 0;
}