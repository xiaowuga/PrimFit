//
// Created by 小乌嘎 on 2023/1/14.
//


#include <primitive_merger.h>
#include <io.h>
#include <util.h>
#include <mesh_generater.h>
#include <easy3d/util/stop_watch.h>

int main() {
    std::string seg_path = "D:\\code\\PrimFit\\data\\input\\chamfer.seg";
    std::vector<int> type;
    std::vector<easy3d::vec3> pos;
    std::vector<easy3d::vec3> dir;
    std::vector<float> r1;
    std::vector<float> r2;
    std::vector<easy3d::vec3> points;
    std::vector<easy3d::vec3> normals;
    std::vector<std::vector<int>> segments;
    std::vector<easy3d::vec3> colors;
    QuadFit::IO::load_segment(seg_path,
                              type, pos, dir, r1, r2,
                              points, normals, segments, colors);

    std::vector<QuadFit::SurfacePrimitive*> shapes;
    QuadFit::Util::construct_SurfacePrimitive_from_std(shapes,
                                                       type, pos, dir, r1,r2, colors);
    easy3d::StopWatch w; w.start();
    std::vector<QuadFit::SurfacePrimitive*> shapes_merged;
    std::vector<std::vector<int>> segments_merged;
    QuadFit::Primitive_Merger primitiveMerger(10, 0.001);

    std::vector<std::vector<int>> merge_pair;
    merge_pair.push_back({28, 32});
    merge_pair.push_back({5, 21});
    merge_pair.push_back({5, 22});
    if(!primitiveMerger.merge_primitves(shapes, shapes_merged, points, normals, segments
                                        , segments_merged, merge_pair)) {
        std::cout << "Primitives merging step fail!" << std::endl;
    }


    easy3d::Box3 bbox = easy3d::geom::bounding_box<easy3d::Box3
            , std::vector<easy3d::vec3>>(points);
    std::vector<std::vector<easy3d::vec3>> proxy_points;
    std::vector<std::vector<int>> proxy_indices;
    QuadFit::Mesh_Generater meshGenerater(bbox);
    meshGenerater.generate_proxy_mesh(shapes_merged,
                                      proxy_points, proxy_indices);
    std::cout << w.elapsed_seconds(3) <<std::endl;
    int cur_step = 1;
    int num = shapes_merged.size();
    std::vector<int> o_type(num);
    std::vector<easy3d::vec3> o_pos(num);
    std::vector<easy3d::vec3> o_dir(num);
    std::vector<float> o_r1(num);
    std::vector<float> o_r2(num);
    std::vector<easy3d::vec3> o_colors(num);

    std::vector<easy3d::vec3> arr_points;
    std::vector<int> arr_indices;
    std::vector<int> arr_patches;
    std::vector<int> arr_labels;
    std::vector<int> arr_cells;
    for(int i = 0; i < num; i++) {
        o_colors[i] = shapes_merged[i]->getColor();
        QuadFit::SurfaceParameters para = shapes_merged[i]->getParameters();
        o_pos[i] = para.pos;
        o_dir[i] = para.dir;
        o_r1[i] = para.r1;
        o_r2[i] = para.r2;
        o_type[i] = QuadFit::to_int_label(shapes_merged[i]->getType());
    }
//    return 0;
    std::string out_file = "D:\\code\\PrimFit\\data\\output2\\chamfer.qf";
    QuadFit::IO::save_quadfit(out_file
                              ,cur_step
                              , o_type, o_pos, o_dir, o_r1, o_r2
                              , points, normals, segments_merged, o_colors
                              ,proxy_points, proxy_indices
                              ,arr_points, arr_indices, arr_cells, arr_patches, arr_labels);
    return 0;
}