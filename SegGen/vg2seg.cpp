//
// Created by xiaowuga on 2023/3/2.
//


#include <iostream>
#include <util.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <igl/point_mesh_squared_distance.h>
#include <igl/readOBJ.h>
#include <fstream>

void read_ascii_group(std::istream& input
                      ,std::vector<double>& group_para
                      ,easy3d::vec3& group_color
                      ,std::vector<int>& seg) {
    group_para.clear();
    seg.clear();
    std::string dumy;
    int type;
    input >> dumy >> type;
    int num;
    std::string xx;
    input >> dumy >> num;

    if(num != 4) {
        std::cout << "error" <<std::endl;
        return;
    }

    group_para.resize(num);
    input >> dumy;
    for (int i = 0; i < num; ++i)
        input >> group_para[i];

    input >> dumy >> dumy;
    float r, g, b;
    input >> dumy >> r >> g >> b;
    group_color = easy3d::vec3(r, g, b) / 255.0;

    int num_points;
    input >> dumy >> num_points;
    for(int i = 0; i < num_points; i++) {
        int idx; input >> idx;
        seg.emplace_back(idx);
    }

}

void load_vg(const std::string& vg_path
             ,std::vector<easy3d::vec3>& vg_points
             ,std::vector<easy3d::vec3>& vg_normals
             ,std::vector<easy3d::vec3>& vg_color
             ,std::vector<std::vector<double>>& group_para
             ,std::vector<easy3d::vec3>& group_color
             ,std::vector<std::vector<int>>& segment) {
    vg_points.clear();
    vg_normals.clear();
    group_para.clear();
    group_color.clear();
    segment.clear();
    std::ifstream input(vg_path.c_str());
    std::string dumy;
    std::size_t num;

    input >> dumy >> num;
    vg_points.resize(num);
    for(int i = 0; i < num; i++) {
        double x, y, z;
        input >> x >> y >> z;
        vg_points[i] = easy3d::vec3(x, y, z);
    }
    input >> dumy >> num;
    vg_color.resize(num);
    for(int i = 0; i < num; i++) {
        double x, y, z;
        input >> x >> y >> z;
        vg_color[i] = easy3d::vec3(x, y, z);
    }

    input >> dumy >> num;
    vg_normals.resize(num);
    for(int i = 0; i < num; i++) {
        double x, y, z;
        input >> x >> y >> z;
        vg_normals[i] = easy3d::vec3(x, y, z);
    }

    std::size_t num_groups = 0;
    input >> dumy >> num_groups;
    group_para.clear();
    group_color.clear();

    for(int i = 0; i < num_groups; i++) {
        std::vector<double> para;
        easy3d::vec3 color;
        std::vector<int>seg;
        read_ascii_group(input, para, color, seg);
        group_para.emplace_back(para);
        group_color.emplace_back(color);
        segment.emplace_back(seg);
        int num_children = 0;
        input >> dumy >> num_children;

        for(int j = 0; j < num_children; j++) {
            read_ascii_group(input, para, color, seg);
        }
    }

}

int main() {
    std::string vg_path = "D:\\ICCV2023\\experiment\\E2\\peal\\points\\peal\\2023-03-04-145002\\peal.vg";
    std::vector<easy3d::vec3> vg_points;
    std::vector<easy3d::vec3> vg_normals;
    std::vector<easy3d::vec3> vg_color;
    std::vector<std::vector<double>> group_para;
    std::vector<easy3d::vec3> group_color;
    std::vector<std::vector<int>> segment;
    load_vg(vg_path, vg_points, vg_normals, vg_color, group_para, group_color, segment);

    std::vector<QuadFit::SurfacePrimitive*> shapes;
    for(int i = 0; i < group_para.size(); i++) {
        QuadFit::SurfaceParameters para;
        easy3d::vec3 dir = easy3d::vec3(group_para[i][0], group_para[i][1], group_para[i][2]);
        para.dir = dir;
        para.r1 = group_para[i][3];
        QuadFit::SurfacePrimitive* sp = QuadFit::construct_bytype( QuadFit::SurfaceType::PLANE, para);
        sp->setColor(group_color[i]);
        shapes.emplace_back(sp);
    }

    std::vector<int> type;
    std::vector<easy3d::vec3> pos;
    std::vector<easy3d::vec3> dir;
    std::vector<float> r1;
    std::vector<float> r2;
    std::vector<easy3d::vec3> colors;
    QuadFit::Util::construct_std_from_SurfacePrimitive(shapes, type, pos, dir, r1, r2, colors);
    std::string out_seg_path = "D:\\ICCV2023\\experiment\\E2\\peal\\points\\peal\\2023-03-04-145002\\peal.seg";
        QuadFit::IO::save_segment(out_seg_path, type, pos
                              , dir, r1, r2
                              , vg_points, vg_normals
                              , segment,colors);
    return 0;
}
