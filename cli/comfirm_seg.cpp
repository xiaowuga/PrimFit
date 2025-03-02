//
// Created by 小乌嘎 on 2023/2/2.
//

#include <io.h>
#include <util.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/core/random.h>
#include <time.h>
#include <stdio.h>
int main() {
    std::string seg_path = "D:\\ICCV2023\\experiment\\E1\\support_kg\\Ours\\support_kg263005.seg";
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

    std::string out_dir = "D:\\ICCV2023\\thingi10k\\bracket\\seg";
//    for(size_t i = 0; i < segments.size(); i++) {
//        easy3d::PointCloud* cloud = new easy3d::PointCloud;
//        for(size_t j = 0; j < segments[i].size(); j++) {
//            int idx = segments[i][j];
//            easy3d::vec3 v = points[idx];
//            cloud->add_vertex(v);
//        }
//        auto cc = cloud->add_vertex_property<easy3d::vec3>("v:color");
//        auto nn = cloud->add_vertex_property<easy3d::vec3>("v:normal");
//        for(size_t j = 0; j < segments[i].size(); j++) {
//            auto v = easy3d::PointCloud::Vertex((int)j);
//            int idx = segments[i][j];
//            cc[v] = colors[i];
//            nn[v] = normals[idx];
//        }
//        std::string type_str =  QuadFit::to_str(QuadFit::to_type(type[i]));
//        std::string out_path = out_dir + "/Primitive" + std::to_string(i + 1) + "_" + type_str + ".ply";
//        easy3d::PointCloudIO::save(out_path, cloud);
//        delete cloud;
//    }
//    const std::vector<easy3d::vec3> color_table = {
//            easy3d::vec3(0.9, 0.239, 0), //orange
//            easy3d::vec3(0.42, 0.85, 0.000), // yellow green
//            easy3d::vec3(0.216, 0.494, 0.722), // hard blue
//            easy3d::vec3(0.302, 0.686, 0.290), // hard green
//            easy3d::vec3(0.596, 0.306, 0.639), // hard purple
//            easy3d::vec3(0.894, 0.100, 0.100), // hard red
//            easy3d::vec3(1.000, 0.5, 0.000), // hard yellow
//            easy3d::vec3(0.400, 0.761, 0.647), // medium blue
//            easy3d::vec3(0.650, 0.847, 0.329), // medium green
//            easy3d::vec3(0.553, 0.627, 0.796), // medium purple
//            easy3d::vec3(0.906, 0.541, 0.765), // medium red
//            easy3d::vec3(0.980, 0.550, 0.380), // medium yellow
//            easy3d::vec3(0.500, 0.690, 0.827), // soft blue
//            easy3d::vec3(0.553, 0.827, 0.780), // soft green
//            easy3d::vec3(0.740, 0.720, 0.850), // soft purple
//            easy3d::vec3(0.984, 0.502, 0.447), // soft red
//            easy3d::vec3(1.000, 1.000, 0.700), // soft yellow
//            easy3d::vec3(0.811, 0.85, 0.277), // yellow green
//            easy3d::vec3(0.8, 0.4, 1.0), // yellow green
//    };
    std::srand(std::time(NULL));
    std::vector<easy3d::vec3> color_table(segments.size());   // index starts from 0
    for (auto &c : color_table)
        c = easy3d::random_color();

    std::map<int,int>mp;
    for(size_t i = 0; i < segments.size(); i++) {
        for(size_t j = 0; j < segments[i].size(); j++) {
            int idx = segments[i][j];
            mp[idx] = i;
        }
    }
    easy3d::PointCloud* P = new easy3d::PointCloud;
    std::vector<easy3d::vec3> pp;
    std::vector<easy3d::vec3> nn;
    std::vector<easy3d::vec3> cc;
    for(auto item : mp) {
        int idx = item.first;
        int cid = item.second;
        pp.emplace_back(points[idx]);
        nn.emplace_back(normals[idx]);
        cc.emplace_back(color_table[cid]);
    }
    int num = pp.size();
    auto C = P->add_vertex_property<easy3d::vec3>("v:color");
    auto N = P->add_vertex_property<easy3d::vec3>("v:normal");
    for(int i = 0; i < num; i++) {
        P->add_vertex(pp[i]);
    }
    for(int i = 0; i < num; i++) {
        auto v = easy3d::PointCloud::Vertex((int)i);
        N[v] = nn[i];
        C[v] = cc[i];
    }
    std::string out_path = "D:\\ICCV2023\\experiment\\E1\\support_kg\\Ours\\support_kg263005_segment.ply";
    easy3d::PointCloudIO::save(out_path, P);
    return 0;
}
