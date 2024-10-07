//
// Created by 小乌嘎 on 2023/1/27.
//

#ifndef QUADFIT_PBLP_H
#define QUADFIT_PBLP_H

#include <linear_program.h>
#include <linear_program_solver.h>
#include <EigenTypedef.h>
#include <map>

namespace QuadFit {
    class PBLP {

    public:
        PBLP() : is_init(false) {}

        void init(MatrixDr& vertives, MatrixIr& faces, VectorI& face_labels, VectorI& face_patches, VectorI& face_point_num,
                  VectorD& face_fit, VectorD& face_valid_area, VectorD& face_area, MatrixDr& face_color,
                  VectorI& patch_label);

        void optimize(double para1, double para2, double para3, LinearProgramSolver::SolverName solver_name,
                      bool re_orient = true);

    public:


        MatrixDr m_in_vertives;
        MatrixIr m_in_faces;
        VectorI m_in_face_labels;
        VectorI m_in_face_patches;
//        VectorI m_in_face_point_num;
//        VectorD m_in_face_fit;
//        VectorD m_in_face_valid_area;
//        VectorD m_in_face_area;
        MatrixDr m_in_face_color;
        VectorI m_in_patch_label;
        VectorI m_in_patch_seg;

        std::vector<double> patch_fitting;
        std::vector<double> patch_valid_area;
        std::vector<double> patch_area;
        std::vector<std::vector<int>> edge;
        std::vector<std::vector<int>> edge_face;
        std::vector<std::vector<int>> curve;
        std::vector<bool> sharp_curve;
        std::vector<float> curve_len;

        MatrixDr m_out_vertives;
        MatrixIr m_out_faces;
        VectorI m_out_face_labels;
        MatrixDr m_out_face_color;

        int sum_points;
        double sum_area;
        double sum_curve;
        int num_patch;
        int num_curve;
        int num_variable;

        std::vector<int> ccm;
        std::vector<int> sccm;


        bool is_init;
    private:
        LinearProgram program_;

    };
}


#endif //QUADFIT_PBLP_H
