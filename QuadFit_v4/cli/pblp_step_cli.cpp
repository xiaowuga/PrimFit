//
// Created by 小乌嘎 on 2023/1/16.
//
#include <pblp2.h>
//#include <pblp.h>
#include <io.h>
#include <util.h>
#include <linear_program_solver.h>
#include <igl/writePLY.h>
#include <igl/writeOBJ.h>
#include <easy3d/util/stop_watch.h>
#include <ostream>
#include <easy3d/core/random.h>
#include <easy3d/fileio/surface_mesh_io.h>

int main() {
    std::string input_path =  "/Users/xiaowuga/Desktop/siggraph2023/appendix/PC190/Ours/PC190.pblp";
    std::vector<easy3d::vec3> vertex;
    std::vector<int> face;
    std::vector<int> face_label;
    std::vector<int> face_patch;
    std::vector<int> face_point;
    std::vector<float> face_fit;
    std::vector<float> face_valid_area;
    std::vector<float> face_area;
    std::vector<easy3d::vec3> colors;
    std::vector<int> patch_label;
    QuadFit::IO::load_pblp(input_path,vertex,face
            ,face_label,face_patch
            ,face_point,face_fit
            ,face_valid_area
            ,face_area,colors
            ,patch_label);
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
    QuadFit::Util::construct_pblp_from_std(m_vertives
            ,m_faces,m_face_labels, m_face_patches
            ,m_face_point_num, m_face_fit
            ,m_face_valid_area, m_face_area
            ,m_face_color, m_patch_label
            ,vertex, face, face_label, face_patch
            ,face_point, face_fit, face_valid_area
            ,face_area, colors, patch_label);
    QuadFit::PBLP2 pblp;
    easy3d::StopWatch w; w.start();
    pblp.init(m_vertives, m_faces
              ,m_face_labels, m_face_patches
              ,m_face_point_num,m_face_fit
              ,m_face_valid_area, m_face_area
              ,m_face_color,m_patch_label);
    std::cout << "init done." <<std::endl;
    double para1 = 0.9, para2 = 0.9, para3 = 0.2;
    pblp.optimize( para1, para2, para3, LinearProgramSolver::GUROBI,  false);
    std::cout << w.elapsed_seconds(3) <<std::endl;
    MatrixDr m_out_vertives = pblp.m_out_vertives;
    MatrixIr m_out_faces = pblp.m_out_faces;
    VectorI m_out_face_labels = pblp.m_out_face_labels;
    return 0;
    int max_index = m_face_labels.maxCoeff();
//    std::vector<easy3d::vec3> color_table(max_index + 1);   // index starts from 0
//    for (auto &c : color_table)
//        c = easy3d::random_color();
    std::string out_path =  "/Users/xiaowuga/Desktop/siggraph2023/experiment/E1/fit4cad_test_pc35/Ours/PC35_"
            + std::to_string(para1) + "_" + std::to_string(para2) + "_" + std::to_string(para3) + ".obj";
    igl::writeOBJ(out_path, m_out_vertives, m_out_faces);
//    return 0;
//    std::vector<easy3d::SurfaceMesh::Vertex> asd;
//    easy3d::SurfaceMesh* mesh = new easy3d::SurfaceMesh;
//    for(int i = 0; i < m_out_vertives.rows(); i++) {
//        Vector3d v = m_out_vertives.row(i);
//        auto vv = mesh->add_vertex(QuadFit::Util::construct_vertices_from_igl_to_easy3d(v));
//        asd.push_back(vv);
//    }
//    for(int i = 0; i < m_out_faces.rows(); i++) {
//        auto v1 = asd[m_out_faces(i, 0)];
//        auto v2 = asd[m_out_faces(i, 1)];
//        auto v3 = asd[m_out_faces(i, 2)];
//        mesh->add_triangle(v1, v2, v3);
//    }
//    auto cc = mesh->add_face_property<easy3d::vec3>("f:color");
//    for(int i = 0; i < m_out_faces.rows(); i++) {
//        auto f = easy3d::SurfaceMesh::Face(i);
//        cc[f] = color_table[m_out_face_labels[i]];
//    }
//    easy3d::SurfaceMeshIO::save(out_path, mesh);
//    igl::writeOBJ(out_path, m_out_vertives, m_out_faces);
//    return 0;
    std::map<int, std::map<int, std::set<int>>> mp;
    int num_face = m_out_faces.rows();
    for(int i = 0; i < num_face; i++) {
        for (int j = 0; j < 3; j++) {
            int s = m_out_faces(i, j);
            int t = m_out_faces(i, (j + 1) % 3);
            if (s > t) {
                std::swap(s, t);
            }
            mp[s][t].insert(i);
        }
    }
    std::map<int, int>mpp;
    int ctt = 1;
    std::string out_line_path =  "/Users/xiaowuga/Desktop/siggraph2023/experiment/E1/fit4cad_test_pc35/Ours/PC35_"
                            + std::to_string(para1) + "_" + std::to_string(para2) + "_" + std::to_string(para3)
                            + "_outline.obj";
    std::ofstream outine( out_line_path.c_str());
    if(outine.is_open()) {
        std::vector<std::vector<int>> edge;
        for (auto item: mp) {
            int x = item.first;
            for (auto jtem: item.second) {
                int y = jtem.first;
                std::vector<int> tmp(jtem.second.begin(), jtem.second.end());
                if (tmp.size() == 2) {
                    int f1 = tmp[0], f2 = tmp[1];
                    if (m_out_face_labels[f1] != m_out_face_labels[f2]) {
                        if (mpp.find(x) == mpp.end()) {
                            mpp[x] = ctt++;
                            outine << "v " << m_out_vertives(x, 0) << ' '
                                   << m_out_vertives(x, 1) << ' ' << m_out_vertives(x, 2) << std::endl;
                        }
                        if (mpp.find(y) == mpp.end()) {
                            mpp[y] = ctt++;
                            outine << "v " << m_out_vertives(y, 0) << ' '
                                   << m_out_vertives(y, 1) << ' ' << m_out_vertives(y, 2) << std::endl;
                        }
                        int x1 = mpp[x];
                        int x2 = mpp[y];
                        std::vector<int> tt = {x1, x2};
                        edge.push_back(tt);
                    }
                }
            }
        }

        for (int i = 0; i < edge.size(); i++) {
            outine << "l " << edge[i][0] << ' ' << edge[i][1] << std::endl;
        }
        outine.close();
    }

    return 0;
}