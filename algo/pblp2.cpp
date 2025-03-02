//
// Created by 小乌嘎 on 2023/1/27.
//

#include "pblp2.h"
#include <util.h>
#include <igl/embree/reorient_facets_raycast.h>
#include <algorithm>
namespace PrimFit {
    void PBLP2::init(MatrixDr& vertives, MatrixIr& faces
               , VectorI& face_labels, VectorI& face_patches
               , VectorI& face_point_num
               , VectorD& face_fit, VectorD& face_valid_area
               , VectorD& face_area, MatrixDr& face_color,
               VectorI& patch_label) {
        m_in_vertives = vertives;
        m_in_faces = faces;
        m_in_face_labels = face_labels;
        m_in_face_patches = face_patches;
        m_in_face_color = face_color;
        m_in_patch_label = patch_label;
        Util::extract_curve_info(m_in_vertives, m_in_faces
                                 , m_in_face_patches
                                 , m_in_patch_label
                                 , edge, edge_face
                                 , curve, curve_len);

        sum_points = face_point_num.sum();
        sum_area = face_area.sum();
        sum_curve = 0;
        num_curve = curve.size();
        num_patch = m_in_patch_label.size();
        for(size_t i = 0; i < curve_len.size(); i++) {
            sum_curve += curve_len[i];
        }
        std::cout<<"sum_points = " << sum_points<<std::endl;
        std::cout<<"sum_area = " << sum_area<<std::endl;
        std::cout<<"sum_curve = " <<sum_curve<<std::endl;
        m_in_patch_seg.resize(num_patch);
        for(int i = 0; i < num_patch; i++) {
            m_in_patch_seg[i] = -1;
        }

        patch_fitting.resize(num_patch, 0);
        patch_valid_area.resize(num_patch, 0);
        patch_area.resize(num_patch, 0);
        sharp_curve.resize(num_curve, false);
        num_variable = num_patch;

        int num_face = m_in_face_labels.size();
        for(int i = 0; i < num_face; i++) {
            int pid = m_in_face_patches[i];
            int sid = m_in_face_labels[i];
            if(m_in_patch_seg[pid] == -1) {
                m_in_patch_seg[pid] = sid;
            } else {
                if(m_in_patch_seg[pid] != sid) {
                    std::cout << "A patch has more than one seg." << std::endl;
                }
            }
            patch_fitting[pid] += face_point_num[i];
            patch_valid_area[pid] += face_valid_area[i];
            patch_area[pid] += face_area[i];
        }
//        ccm.resize(num_curve, -1);

        num_variable = 0;
        patch_state.resize(num_patch, false);
        patch_map2variable.resize(num_patch, -1);
        curve_map2variable.resize(num_curve, -1);
        sharp_curve_map2variable.resize(num_curve, -1);
        for(int i = 0; i < num_patch; i++) {
            if(m_in_patch_label[i] == 1) {
                patch_state[i] = true;
                patch_map2variable[i] = num_variable++;
            } else {
                patch_map2variable[i] = -1;
            }
        }
        int num_patch_variable = num_variable;
        std::cout << "num_patch = " << num_patch << std::endl;
        std::cout << "num_patch_variable = " << num_patch_variable <<std::endl;
        for(int i = 0; i < num_curve; i++) {
            int num = 0;
            for(int j = 0; j < curve[i].size(); j++) {
                int curve_id = curve[i][j];
                if(m_in_patch_label[curve_id]) {
                    num++;
                }
            }
            if(num > 0) {
                curve_map2variable[i] = num_variable++;
                sharp_curve_map2variable[i] = num_variable++;
            } else {
                curve_map2variable[i] = -1;
                sharp_curve_map2variable[i] = -1;
            }
        }
        std::cout << "num_curve_variable = " << (num_variable - num_patch_variable) / 2 << std::endl;
        is_init = true;
    }

    void PBLP2::optimize(double para1, double para2, double para3
                        , LinearProgramSolver::SolverName solver_name,
                        bool re_orient) {
        double coeff_data_fitting = para1;
        double coeff_coverage = sum_points * para2 / sum_area;
        double coeff_complexity = sum_points * para3 / sum_curve;

//        double coeff_data_fitting = sum_curve * para1 / sum_points;
//        double coeff_coverage = sum_curve * para2 / sum_area;
//        double coeff_complexity = para3;

        program_.clear();
        LinearObjective* objective = program_.create_objective(LinearObjective::MINIMIZE);
        int num_active_patch = 0;
        for(int i = 0; i < num_patch; i++) {
            if(patch_map2variable[i] != -1) {
                num_active_patch++;
                int vid = patch_map2variable[i];
                objective->add_coefficient(vid, -coeff_data_fitting * patch_fitting[i]);
                double uncovered_area = patch_area[i] - patch_valid_area[i];
                objective->add_coefficient(vid, uncovered_area * coeff_coverage);
            }
        }
        for(int i = 0; i < num_curve; i++) {
            int idx = sharp_curve_map2variable[i];
            if(idx != -1) {
                objective->add_coefficient(idx, -coeff_complexity * curve_len[i]);
            }
        }


        const std::vector<Variable*>& variables = program_.create_n_variables(num_variable);
        for (std::size_t i = 0; i < num_variable; ++i) {
            Variable* v = variables[i];
            v->set_variable_type(Variable::BINARY);
        }

        for(int i = 0; i < num_curve; i++) {
            LinearConstraint* c = program_.create_constraint(LinearConstraint::FIXED, 0.0, 0.0);
            int idx = curve_map2variable[i];
            if(idx != -1) {
                for(size_t j = 0; j < curve[i].size(); j++) {
                    int vid = patch_map2variable[curve[i][j]];
                    if(vid != -1) {
                        c->add_coefficient(vid, 1.0);
                    }
                }
                c->add_coefficient(idx, -2.0);
            }
        }

        double M = 1.0;
        for(int i = 0; i < num_curve; i++) {
            int idx1 = curve_map2variable[i];
            int idx2 = sharp_curve_map2variable[i];
            if(idx1 == -1 || idx2 == -1) continue;

            LinearConstraint* c = program_.create_constraint();
            c->add_coefficient(idx1, 1.0);
            c->add_coefficient(idx2, -1.0);
            c->set_bound(LinearConstraint::LOWER, 0.0);
            std::vector<int> tmp;
            for(size_t j = 0; j < curve[i].size(); j++) {
                int idx = curve[i][j];
                if(patch_map2variable[idx] != -1) {
                     tmp.push_back(curve[i][j]);
                }
            }
            for(size_t j = 0; j < tmp.size(); j++) {
                int pj = tmp[j];
                int pjs = m_in_patch_seg[pj];
                int pjv = patch_map2variable[pj];

                for(size_t k = j + 1; k < tmp.size(); k++) {
                    int pk = tmp[k];
                    int pks = m_in_patch_seg[pk];
                    int pkv = patch_map2variable[pk];
                    if(pjs != pks) {
                        c = program_.create_constraint();
                        c->add_coefficient(idx2, 1.0);
                        c->add_coefficient(pjv, -M);
                        c->add_coefficient(pkv, -M);
                        c->add_coefficient(idx1, -M);
                        c->set_bound(LinearConstraint::LOWER, 1.0 - 3.0 * M);
                    }
                }
            }
        }

//        for(int i = 0; i < num_curve; i++) {
//            if(curve[i].size() == 1) {
//                int pid = curve[i][0];
//                int pv = patch_map2variable[pid];
//                if(pv != -1) {
//                    LinearConstraint* c = program_.create_constraint(LinearConstraint::FIXED, 0.0, 0.0);
//                    c->add_coefficient(pv, 1.0);
//                }
//            }
//        }
        LinearProgramSolver solver;
        if(solver.solve(&program_, solver_name)) {
            std::cout << solver.objective_value() <<std::endl;
            std::cout << "solving the binary program done. " << std::endl;
            std::vector<int> label(num_active_patch, 1);
            const std::vector<double>& X = solver.solution();
            int num_extract = 0;
            for(int i = 0; i < num_active_patch; i++) {
                if (static_cast<int>(std::round(X[i])) == 0) {
                    label[i] = 0;
                } else {
                    num_extract++;
                }
            }
            std::cout << "extract_num = " << num_extract << std::endl;

            for(int i = 0; i < num_patch; i++) {
                if(patch_state[i]) {
                    int idx = patch_map2variable[i];
                    if(label[idx] == 0) {
                        patch_state[i] = false;
                    }
                }
            }
            int ctv = 0, ctf = 0;
            std::map<int,int> vm;
            int num_face = m_in_face_labels.size();
            for(int i = 0; i < num_face; i++) {
                int pid = m_in_face_patches[i];
                if(patch_state[pid])  {
                    ctf++;
                    for(int j = 0; j < 3; j++) {
                        int vid = m_in_faces(i, j);
                        if(vm.find(vid) == vm.end()) {
                            vm[vid] = ctv++;
                        }
                    }
                }
            }

            m_out_vertives.resize(ctv, 3);
            for(auto& v : vm) {
                int x = v.first, y = v.second;
                m_out_vertives.row(y) = m_in_vertives.row(x);
            }
            int ct = 0;
            MatrixIr tmp1; tmp1.resize(ctf, 3);
//            m_out_faces.resize(ctf, 3);
            m_out_face_labels.resize(ctf);
            m_out_face_color.resize(ctf, 3);
            for(int i = 0; i < num_face; i++) {
                int pid = m_in_face_patches[i];
                if(patch_state[pid])  {
                    int x = vm[m_in_faces(i, 0)];
                    int y = vm[m_in_faces(i, 1)];
                    int z = vm[m_in_faces(i, 2)];
//                    m_out_faces.row(ct) = Vector3I(x, y, z);
                    tmp1.row(ct) = Vector3I(x, y, z);
                    m_out_face_labels[ct] = m_in_face_labels[i];
                    m_out_face_color.row(ct) = m_in_face_color.row(i);
                    ct++;
                }
            }

            if(re_orient) {
                VectorI I;
                igl::embree::reorient_facets_raycast(m_out_vertives, tmp1, m_out_faces, I);
            } else {
                m_out_faces = tmp1;
            }

        }
    }
}