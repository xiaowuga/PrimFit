//
// Created by xiaowuga on 2024/8/6.
//

#ifndef MESH_BOOLEAN_H
#define MESH_BOOLEAN_H

#include <Eigen/Core>
#include <Eigen/Dense>


class MeshArrangement {
public:
    MeshArrangement(const Eigen::MatrixXd& vertices, const Eigen::MatrixXi& faces, const Eigen::VectorXi& face_labels)
        : m_vertices(vertices), m_faces(faces), m_in_face_labels(face_labels){}

    ~MeshArrangement() = default;

    void run();

public:
    Eigen::MatrixXd m_vertices;
    Eigen::MatrixXi m_faces;
    Eigen::VectorXi m_in_face_labels;
    Eigen::VectorXi m_out_face_labels;
    Eigen::MatrixXi m_cells;
    Eigen::VectorXi m_patches;
};



#endif //MESH_BOOLEAN_H
