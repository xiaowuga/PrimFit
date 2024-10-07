//
// Created by 小乌嘎 on 2023/2/4.
//

#include <io.h>
#include <util.h>
#include <igl/writeOBJ.h>

int main() {
    std::string input_path = "D:\\ICCV2023\\experiment\\E1\\Minarets\\Ours\\Minarets.pblp";
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

    int num_patch = m_patch_label.size();
    int num_face = m_faces.rows();

    std::vector<double> patch_valid_area(num_patch, 0);
    std::vector<double> patch_area(num_patch, 0);
    for(int i = 0; i < num_face; i++) {
        int p = m_face_patches[i];
        patch_valid_area[p] += m_face_valid_area[i];
        patch_area[p] += m_face_area[i];
    }
    std::vector<bool> label(num_patch , false);
    int ct = 0;
    for(int i = 0; i < num_patch; i++) {
        float c = patch_valid_area[i] / patch_area[i];
//
        if(patch_label[i]) {
            std::cout << c << ' ' << patch_valid_area[i] << ' ' << patch_area[i] << std::endl;
            label[i] = true;
            ct++;
        }
    }
    std::cout << ct << std::endl;
    MatrixIr m_out_faces; m_out_faces.resize(num_face, 3);
    ct = 0;
    for(int i = 0; i < num_face; i++) {
        int p = m_face_patches[i];
        if(label[p]) {
            m_out_faces.row(ct++) = m_faces.row(i);
        }
    }
    m_out_faces.conservativeResize(ct, 3);
    std::string out_path = "D:\\ICCV2023\\experiment\\E1\\min1.obj";
    igl::writeOBJ(out_path, m_vertives, m_out_faces);




    return 0;
}
