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
#include <easy3d/util/file_system.h>

int main() {

    std::string seg_path = "C:\\Users\\xiaowuga\\Desktop\\ICCV2023rebuttal\\subset_abc_seg\\00973146_2_10000.seg";
    std::string bn = easy3d::file_system::base_name(seg_path);
    std::string ours_dir = "C:\\Users\\xiaowuga\\Desktop\\ICCV2023rebuttal\\subset_abc_ours\\";
    std::string ours_outline_dir = "C:\\Users\\xiaowuga\\Desktop\\ICCV2023rebuttal\\subset_abc_ours_outline\\";
    std::string pblp_dir = "C:\\Users\\xiaowuga\\Desktop\\ICCV2023rebuttal\\subset_abc_pblp\\";


    std::vector<std::string> entries;
    easy3d::file_system::get_directory_entries(qf_dir, entries, false);
    double asd = 0;
    for(std::size_t j = 0; j < entries.size(); j++) {
        std::string input_path = pblp_dir + entries[j];
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
        QuadFit::IO::load_pblp(input_path, vertex, face, face_label, face_patch, face_point, face_fit, face_valid_area,
                               face_area, colors, patch_label);
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
        QuadFit::Util::construct_pblp_from_std(m_vertives, m_faces, m_face_labels, m_face_patches, m_face_point_num,
                                               m_face_fit, m_face_valid_area, m_face_area, m_face_color, m_patch_label,
                                               vertex, face, face_label, face_patch, face_point, face_fit,
                                               face_valid_area, face_area, colors, patch_label);
        QuadFit::PBLP2 pblp;
        easy3d::StopWatch w;
        w.start();
        pblp.init(m_vertives, m_faces, m_face_labels, m_face_patches, m_face_point_num, m_face_fit, m_face_valid_area,
                  m_face_area, m_face_color, m_patch_label);
//        std::cout << "init done." << std::endl;
        pblp.optimize(0.9, 0.5, 0.6, LinearProgramSolver::SCIP, true);
        asd += w.elapsed_seconds(3);
//        MatrixDr m_out_vertives = pblp.m_out_vertives;
//        MatrixIr m_out_faces = pblp.m_out_faces;
//        VectorI m_out_face_labels = pblp.m_out_face_labels;
//        int max_index = m_face_labels.maxCoeff();
//        std::vector<easy3d::vec3> color_table(max_index + 1);   // index starts from 0
//        for (auto &c: color_table)
//            c = easy3d::random_color();
//        std::string out_path = ours_dir + bn + ".obj";
//        igl::writeOBJ(out_path, m_out_vertives, m_out_faces);
//        std::map<int, std::map<int, std::set<int>>> mp;
//        int num_face = m_out_faces.rows();
//        for (int i = 0; i < num_face; i++) {
//            for (int j = 0; j < 3; j++) {
//                int s = m_out_faces(i, j);
//                int t = m_out_faces(i, (j + 1) % 3);
//                if (s > t) {
//                    std::swap(s, t);
//                }
//                mp[s][t].insert(i);
//            }
//        }
//        std::map<int, int> mpp;
//        int ctt = 1;
//        std::ofstream outine(ours_outline_dir + bn + "_outline.obj");
//        if (outine.is_open()) {
//            std::vector<std::vector<int>> edge;
//            for (auto item: mp) {
//                int x = item.first;
//                for (auto jtem: item.second) {
//                    int y = jtem.first;
//                    std::vector<int> tmp(jtem.second.begin(), jtem.second.end());
//                    if (tmp.size() == 2) {
//                        int f1 = tmp[0], f2 = tmp[1];
//                        if (m_out_face_labels[f1] != m_out_face_labels[f2]) {
//                            if (mpp.find(x) == mpp.end()) {
//                                mpp[x] = ctt++;
//                                outine << "v " << m_out_vertives(x, 0) << ' '
//                                       << m_out_vertives(x, 1) << ' ' << m_out_vertives(x, 2) << std::endl;
//                            }
//                            if (mpp.find(y) == mpp.end()) {
//                                mpp[y] = ctt++;
//                                outine << "v " << m_out_vertives(y, 0) << ' '
//                                       << m_out_vertives(y, 1) << ' ' << m_out_vertives(y, 2) << std::endl;
//                            }
//                            int x1 = mpp[x];
//                            int x2 = mpp[y];
//                            std::vector<int> tt = {x1, x2};
//                            edge.push_back(tt);
//                        }
//                    }
//                }
//            }
//
//            for (int i = 0; i < edge.size(); i++) {
//                outine << "l " << edge[i][0] << ' ' << edge[i][1] << std::endl;
//            }
//            outine.close();
//        }
    }

    std::cout << asd / 40 <<std::endl;
    return 0;
}