//
// Created by 小乌嘎 on 2023/2/4.
//

#include <io.h>
#include <util.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/core/surface_mesh.h>
#include <easy3d/fileio/surface_mesh_io.h>
#include <igl/writeOBJ.h>

int main () {
    std::string input_path = "D:/ICCV2023/QuadFit_v2/data/input/chamfer3.pblp";
    std::string out_dir = "D:/ICCV2023/result/comfirm_curve/chamfer/4";
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

    std::vector<std::vector<int>> edge;
    std::vector<std::vector<int>> edge_face;
    std::vector<std::vector<int>> curve;
    std::vector<bool> sharp_curve;
    std::vector<float> curve_len;
    QuadFit::Util::extract_curve_info(m_vertives, m_faces, m_face_patches, m_face_labels
                                      ,edge, edge_face, curve, curve_len);
    int num_points = m_vertives.rows();
    int num_face = m_faces.rows();
    int num_patch = patch_label.size();
    std::vector<std::vector<int>> patch_face;
    patch_face.resize(num_patch);
    for(int i = 0; i < num_face; i++) {
        int idx = m_face_patches[i];
        patch_face[idx].emplace_back(i);
    }
    int num_curve = curve_len.size();
    int count = 0;
    for(int i = 0; i < num_curve; i++){
        if(curve[i].size() != 4) {
            continue;
        }
        count++;
        std::map<int, int> mp;
        int num_p = 0, num_f = 0;
        MatrixDr V; V.resize(num_points, 3);
        MatrixIr F; F.resize(num_face, 3);
        for(size_t j = 0; j < curve[i].size(); j++) {
            int id1 = curve[i][j];
            for(size_t k = 0; k < patch_face[id1].size(); k++){
                int id2 = patch_face[id1][k];
                for(int x = 0; x < 3; x++) {
                    int id3 = m_faces(id2, x);
                    if(mp.find(id3) == mp.end()) {
                        V.row(num_p) = m_vertives.row(id3);
                        mp[id3] = num_p++;
                    }
                }
                int p1 = mp[m_faces(id2, 0)];
                int p2 = mp[m_faces(id2, 1)];
                int p3 = mp[m_faces(id2, 2)];
                F.row(num_f++) = Vector3I(p1, p2, p3);
            }
        }
        V.conservativeResize(num_p, 3);
        F.conservativeResize(num_f, 3);
        std::string name = std::to_string(curve[i].size()) + "_" + std::to_string(i) + ".obj";
        std::cout << out_dir + name <<std::endl;
        igl::writeOBJ(out_dir + name, V, F);
    }
    std::cout << count <<std::endl;

    return 0;
}