//
// Created by 小乌嘎 on 2023/6/6.
//
#include <pblp.h>
#include <iostream>
#include <primitive_merger.h>
#include <io.h>
#include <util.h>
#include <Arrangement.h>
#include <mesh_generater.h>
#include <partition_simplifier3.h>
#include <linear_program_solver.h>
#include <easy3d/fileio/surface_mesh_io.h>
#include <igl/writeOBJ.h>


void merge_step(std::vector<int>& in_type, std::vector<easy3d::vec3>& in_pos
                ,std::vector<easy3d::vec3>& in_dir,  std::vector<float>& in_r1
                ,std::vector<float>& in_r2, std::vector<easy3d::vec3>& in_points
                ,std::vector<easy3d::vec3>& in_normal, std::vector<std::vector<int>>& in_segments
                ,std::vector<easy3d::vec3>& in_colors, int& out_step
                ,std::vector<int>& out_type, std::vector<easy3d::vec3>& out_pos
                ,std::vector<easy3d::vec3>& out_dir, std::vector<float>& out_r1
                ,std::vector<float>& out_r2, std::vector<easy3d::vec3>& out_points
                ,std::vector<easy3d::vec3>& out_normals, std::vector<std::vector<int>>& out_segments
                ,std::vector<easy3d::vec3>& out_colors,  std::vector<std::vector<easy3d::vec3>>& proxy_points
                ,std::vector<std::vector<int>>& proxy_indices,std::vector<easy3d::vec3>& arr_points
                ,std::vector<int>& arr_indices, std::vector<int>& arr_patches, std::vector<int>& arr_labels
                ,std::vector<int>& arr_cells) {
    std::vector<PrimFit::SurfacePrimitive*> shapes;
    PrimFit::Util::construct_SurfacePrimitive_from_std(shapes,
                                                       in_type, in_pos, in_dir, in_r1,in_r2, in_colors);
    std::vector<PrimFit::SurfacePrimitive*> shapes_merged;
    out_segments.clear();
    PrimFit::Primitive_Merger primitiveMerger(10, 0.001);
    std::vector<std::vector<int>> merge_pair;
    merge_pair.push_back({28, 32});
    merge_pair.push_back({5, 21});
    merge_pair.push_back({5, 22});
    out_segments.clear();
    primitiveMerger.merge_primitves(shapes, shapes_merged, in_points, in_normal, in_segments
            , out_segments, merge_pair);

    easy3d::Box3 bbox = easy3d::geom::bounding_box<easy3d::Box3
            , std::vector<easy3d::vec3>>(in_points);
    proxy_points.clear();
    proxy_indices.clear();
    PrimFit::Mesh_Generater meshGenerater(bbox);
    meshGenerater.generate_proxy_mesh(shapes_merged,
                                      proxy_points, proxy_indices);

    out_step = 1;
    int num = shapes_merged.size();
    out_type.clear(); out_type.resize(num);
    out_pos.clear();  out_pos.resize(num);
    out_dir.clear();  out_dir.resize(num);
    out_r1.clear();   out_r1.resize(num);
    out_r2.clear();   out_r2.resize(num);
    out_colors.clear(); out_colors.resize(num);
    out_normals = in_normal;
    out_points = in_points;
    for(int i = 0; i < num; i++) {
        out_colors[i] = shapes_merged[i]->getColor();
        PrimFit::SurfaceParameters para = shapes_merged[i]->getParameters();
        out_pos[i] = para.pos;
        out_dir[i] = para.dir;
        out_r1[i] = para.r1;
        out_r2[i] = para.r2;
        out_type[i] = PrimFit::to_int_label(shapes_merged[i]->getType());
    }

}

void arrangement_step(int& step
        ,std::vector<int>& type, std::vector<easy3d::vec3>& pos
        ,std::vector<easy3d::vec3>& dir, std::vector<float>& r1
        ,std::vector<float>& in_r2, std::vector<easy3d::vec3>& in_points
        ,std::vector<easy3d::vec3>& in_normals, std::vector<std::vector<int>>& in_segments
        ,std::vector<easy3d::vec3>& in_colors,  std::vector<std::vector<easy3d::vec3>>& proxy_points
        ,std::vector<std::vector<int>>& proxy_indices,std::vector<easy3d::vec3>& arr_points
        ,std::vector<int>& arr_indices, std::vector<int>& arr_patches, std::vector<int>& arr_labels
        ,std::vector<int>& arr_cells) {
    MatrixDr V;
    MatrixIr F;
    VectorI L;
    PrimFit::Util::merge_proxy_from_std(V, F, L, proxy_points, proxy_indices);
    auto engine = PyMesh::Arrangement::create_mesh_arrangement(V, F, L);
    engine->run();
    step = 2;
    MatrixDr m_vertices = engine->get_vertices();
    MatrixIr m_faces = engine->get_faces();
    VectorI m_out_face_labels = engine->get_out_face_labels();
    MatrixIr m_cells = engine->get_cells();
    VectorI m_patches = engine->get_patches();
    arr_points.clear();
    arr_indices.clear();
    arr_labels.clear();
    arr_patches.clear();

    PrimFit::Util::construct_std_from_arrangement(m_vertices, m_faces
            , m_out_face_labels, m_cells, m_patches
            , arr_points, arr_indices, arr_labels
            , arr_patches, arr_cells);
}

void grow_step(int& step
        ,std::vector<int>& type, std::vector<easy3d::vec3>& pos
        ,std::vector<easy3d::vec3>& dir, std::vector<float>& r1
        ,std::vector<float>& in_r2, std::vector<easy3d::vec3>& in_points
        ,std::vector<easy3d::vec3>& in_normals, std::vector<std::vector<int>>& in_segments
        ,std::vector<easy3d::vec3>& in_colors,  std::vector<std::vector<easy3d::vec3>>& proxy_points
        ,std::vector<std::vector<int>>& proxy_indices,std::vector<easy3d::vec3>& arr_points
        ,std::vector<int>& arr_indices, std::vector<int>& arr_patches, std::vector<int>& arr_labels
        ,std::vector<int>& arr_cells
        ,std::vector<easy3d::vec3>& out_face_vertices
        ,std::vector<int>& out_face_indices
        ,std::vector<int>& out_face_label
        ,std::vector<int>& out_face_patch
        ,std::vector<int>& out_face_point_num
        ,std::vector<float>& out_face_fit
        ,std::vector<float>& out_face_valid_area
        ,std::vector<float>& out_face_area
        ,std::vector<easy3d::vec3>& out_face_colors
        ,std::vector<int>& out_patch_label) {
    MatrixDr m_vertives;
    MatrixIr m_faces;
    VectorI m_face_labels;
    MatrixIr m_cells;
    VectorI m_patches;
    PrimFit::Util::construct_arrangement_from_std(m_vertives, m_faces, m_face_labels
            ,m_cells, m_patches
            ,arr_points, arr_indices
            ,arr_labels, arr_patches
            ,arr_cells);

    MatrixDr m_points;
    PrimFit::Util::construct_vertices_list_from_std_to_igl(m_points, in_points);
    PrimFit::Partition_Simplifier3 ps;
    ps.init(m_points, m_vertives, m_faces
            , m_patches, m_face_labels,in_segments, 0.02);
    ps.simplify(0.5);

    VectorI m_face_point_num = ps.m_face_point_num;
    VectorD m_face_fit = ps.m_face_fit;
    VectorD m_face_valid_area = ps.m_face_valid_area;
    VectorD m_face_area = ps.m_face_area;
    VectorI m_patch_label = ps.m_patch_label;
    int num_face = m_faces.rows();
    MatrixDr m_face_color; m_face_color.resize(num_face, 3);
    for(int i = 0; i < num_face; i++) {
        int label = m_face_labels[i];
        m_face_color.row(i) = PrimFit::Util::construct_vertices_from_easy3d_to_igl(in_colors[label]);
    }
    int ct = 0;
    for(int i = 0; i < m_patch_label.size(); i++) {
        if(m_patch_label[i]) ct++;
    }
    out_face_vertices.clear();
    out_face_indices.clear();
    out_face_label.clear();
    out_face_patch.clear();
    out_face_point_num.clear();
    out_face_fit.clear();
    out_face_valid_area.clear();
    out_face_area.clear();
    out_face_colors.clear();
    out_patch_label.clear();
    PrimFit::Util::construct_std_from_pblp(m_vertives, m_faces
            , m_face_labels, m_patches
            ,m_face_point_num, m_face_fit
            ,m_face_valid_area, m_face_area
            ,m_face_color,m_patch_label
            ,out_face_vertices, out_face_indices, out_face_label
            ,out_face_patch, out_face_point_num
            ,out_face_fit, out_face_valid_area
            ,out_face_area, out_face_colors, out_patch_label);

}

void pblp_step(std::vector<easy3d::vec3>& face_vertices
        ,std::vector<int>& face_indices
        ,std::vector<int>& face_label
        ,std::vector<int>& face_patch
        ,std::vector<int>& face_point_num
        ,std::vector<float>& face_fit
        ,std::vector<float>& face_valid_area
        ,std::vector<float>& face_area
        ,std::vector<easy3d::vec3>& face_colors
        ,std::vector<int>& patch_label
        ,MatrixDr& m_out_vertives
        ,MatrixIr& m_out_faces) {
    MatrixDr m_vertives;
    MatrixIr m_faces;
    VectorI m_face_labels;
    VectorI m_face_patches;
    VectorI m_face_point_num;
    VectorD m_face_fit;
    VectorD m_face_valid_area;
    VectorD m_face_area;
    MatrixDr m_face_color;
    VectorI m_patch_label;
    PrimFit::Util::construct_pblp_from_std(m_vertives
            ,m_faces,m_face_labels, m_face_patches
            ,m_face_point_num, m_face_fit
            ,m_face_valid_area, m_face_area
            ,m_face_color, m_patch_label
            ,face_vertices, face_indices, face_label, face_patch
            ,face_point_num, face_fit, face_valid_area
            ,face_area, face_colors, patch_label);
    PrimFit::PBLP pblp;
    double para1 = 0.9, para2 = 0.9, para3 = 0.2;
    pblp.init(m_vertives, m_faces
            ,m_face_labels, m_face_patches
            ,m_face_point_num,m_face_fit
            ,m_face_valid_area, m_face_area
            ,m_face_color,m_patch_label);
    pblp.optimize( para1, para2, para3, LinearProgramSolver::SCIP,  true);
    m_out_vertives = pblp.m_out_vertives;
    m_out_faces = pblp.m_out_faces;
    VectorI m_out_face_labels = pblp.m_out_face_labels;
}


int main() {
    std::string input_path = "D:\\code\\PrimFit\\data\\sculpt_refine.seg";
    std::string output_path = "D:\\code\\PrimFit\\data\\sculpt_output.obj";
    std::vector<int> in_type;
    std::vector<easy3d::vec3> in_pos;
    std::vector<easy3d::vec3> in_dir;
    std::vector<float> in_r1;
    std::vector<float> in_r2;
    std::vector<easy3d::vec3> in_points;
    std::vector<easy3d::vec3> in_normals;
    std::vector<std::vector<int>> in_segments;
    std::vector<easy3d::vec3> in_colors;
    PrimFit::IO::load_segment(input_path,
                              in_type, in_pos, in_dir, in_r1, in_r2,
                              in_points, in_normals, in_segments, in_colors);
    int out_step;
    std::vector<int> out_type;
    std::vector<easy3d::vec3> out_pos;
    std::vector<easy3d::vec3> out_dir;
    std::vector<float> out_r1;
    std::vector<float> out_r2;
    std::vector<easy3d::vec3> out_points;
    std::vector<easy3d::vec3> out_normals;
    std::vector<std::vector<int>> out_segments;
    std::vector<easy3d::vec3> out_colors;
    std::vector<std::vector<easy3d::vec3>> proxy_points;
    std::vector<std::vector<int>> proxy_indices;
    std::vector<easy3d::vec3> arr_points;
    std::vector<int> arr_indices;
    std::vector<int> arr_patches;
    std::vector<int> arr_labels;
    std::vector<int> arr_cells;
    merge_step(in_type, in_pos, in_dir, in_r1, in_r2, in_points, in_normals, in_segments, in_colors
               ,out_step, out_type,out_pos, out_dir, out_r1, out_r2, out_points, out_normals, out_segments, out_colors
               ,proxy_points, proxy_indices, arr_points,arr_indices, arr_patches, arr_labels, arr_cells);

    arrangement_step(out_step, out_type,out_pos, out_dir, out_r1, out_r2, out_points, out_normals, out_segments, out_colors
            ,proxy_points, proxy_indices, arr_points,arr_indices, arr_patches, arr_labels, arr_cells);

    std::vector<easy3d::vec3> out_face_vertices;
    std::vector<int> out_face_indices;
    std::vector<int> out_face_label;
    std::vector<int> out_face_patch;
    std::vector<int> out_face_point_num;
    std::vector<float> out_face_fit;
    std::vector<float> out_face_valid_area;
    std::vector<float> out_face_area;
    std::vector<easy3d::vec3> out_face_colors;
    std::vector<int> out_patch_label;
    grow_step(out_step, out_type,out_pos, out_dir, out_r1, out_r2, out_points, out_normals, out_segments, out_colors
            ,proxy_points, proxy_indices, arr_points,arr_indices, arr_patches, arr_labels, arr_cells
            ,out_face_vertices, out_face_indices, out_face_label, out_face_patch, out_face_point_num
            ,out_face_fit, out_face_valid_area, out_face_area, out_face_colors, out_patch_label);

    std::vector<easy3d::vec3> out_mesh_points;
    std::vector<int> out_mesh_indices;
    MatrixDr m_out_vertives;
    MatrixIr m_out_faces;
    pblp_step(out_face_vertices, out_face_indices, out_face_label, out_face_patch
            ,out_face_point_num, out_face_fit, out_face_valid_area, out_face_area
            ,out_face_colors, out_patch_label, m_out_vertives, m_out_faces);
    igl::writeOBJ(output_path, m_out_vertives, m_out_faces);
    
    return 0;
}
