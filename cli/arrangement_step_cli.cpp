//
// Created by 小乌嘎 on 2023/1/15.
//
#include <io.h>
#include <Arrangement.h>
#include <util.h>


int main() {
    std::string input_path = "D:\\code\\PrimFit\\data\\output2\\chamfer.qf";
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

    MatrixDr V;
    MatrixIr F;
    VectorI L;
    QuadFit::Util::merge_proxy_from_std(V, F, L, proxy_points, proxy_indices);
    auto engine = PyMesh::Arrangement::create_mesh_arrangement(V, F, L);
    engine->run();
    MatrixDr m_vertices = engine->get_vertices();
    MatrixIr m_faces = engine->get_faces();
    VectorI m_out_face_labels = engine->get_out_face_labels();
    MatrixIr m_cells = engine->get_cells();
    VectorI m_patches = engine->get_patches();
    QuadFit::Util::construct_std_from_arrangement(m_vertices, m_faces
                                                  , m_out_face_labels, m_cells, m_patches
                                                  , arr_points, arr_indices, arr_labels
                                                  , arr_patches, arr_cells);
//    return 0;
    cur_step = 2;
    QuadFit::IO::save_quadfit( "D:\\code\\PrimFit\\data\\output2\\chamfer1.qf", cur_step
                              , type, pos, dir, r1, r2, points
                              , normals, segments, colors
                              , proxy_points, proxy_indices
                              , arr_points, arr_indices, arr_cells, arr_patches, arr_labels);

    return 0;
}
