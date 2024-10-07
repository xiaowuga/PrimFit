//
// Created by 小乌嘎 on 2023/2/3.
//
#include <io.h>
#include <util.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/fileio/surface_mesh_io.h>

int main() {
    std::string input_path =  "D:\\ICCV2023\\thingi10k\\Minarets\\Ours3\\Minarets1.qf";
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

    std::string out_dir =  "D:\\ICCV2023\\thingi10k\\Minarets\\Ours3\\merge";
    for(size_t i = 0; i < segments.size(); i++) {
        easy3d::PointCloud *cloud = new easy3d::PointCloud;
        for (size_t j = 0; j < segments[i].size(); j++) {
            int idx = segments[i][j];
            easy3d::vec3 v = points[idx];
            cloud->add_vertex(v);
        }
        auto cc = cloud->add_vertex_property<easy3d::vec3>("v:color");
        auto nn = cloud->add_vertex_property<easy3d::vec3>("v:normal");
        for (size_t j = 0; j < segments[i].size(); j++) {
            auto v = easy3d::PointCloud::Vertex((int) j);
            int idx = segments[i][j];
            cc[v] = colors[i];
            nn[v] = normals[idx];
        }
        std::string type_str = QuadFit::to_str(QuadFit::to_type(type[i]));
        std::string out_path = out_dir + + "/Primitive" + std::to_string(i + 1) + "_" + type_str + ".ply";
        easy3d::PointCloudIO::save(out_path, cloud);
    }

    for(size_t i = 0; i < proxy_points.size(); i++) {
        std::vector<easy3d::SurfaceMesh::Vertex> vertex;
        easy3d::SurfaceMesh* mesh = new easy3d::SurfaceMesh;
        for(size_t j = 0; j < proxy_points[i].size(); j++) {
            auto v = mesh->add_vertex(proxy_points[i][j]);
            vertex.emplace_back(v);
        }
        for(size_t j = 0; j < proxy_indices[i].size(); j += 3) {
            int x = proxy_indices[i][j];
            int y = proxy_indices[i][j + 1];
            int z = proxy_indices[i][j + 2];
            mesh->add_triangle(vertex[x], vertex[y], vertex[z]);
        }
        auto cc = mesh->add_face_property<easy3d::vec3>("f:color");
        for(auto f : mesh->faces()) {
            cc[f] = colors[i];
        }
        std::string type_str = QuadFit::to_str(QuadFit::to_type(type[i]));
        std::string out_path = out_dir + "/Primitive" + std::to_string(i + 1) + "_" + type_str + "_proxy.ply";
        easy3d::SurfaceMeshIO::save(out_path, mesh);
    }
    return 0;
}