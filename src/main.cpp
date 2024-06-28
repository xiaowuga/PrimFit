//
// Created by xiaowuga on 2024/6/24.
//
#include "base.h"
#include <iostream>

#include <igl/readPLY.h>

void exportToPLY(const std::string file_path,  Eigen::MatrixXd& V) {
    std::ofstream file(file_path);

    file << "ply\n";
    file << "format ascii 1.0\n";
    file << "element vertex " << V.rows() << "\n";
    file << "property float x\n";
    file << "property float y\n";
    file << "property float z\n";
    file << "end_header\n";

    for(size_t i = 0; i < V.rows(); i++) {
        file << V.row(i).x() << ' ' << V.row(i).y() << ' ' << V.row(i).z() << std::endl;
    }

    file.close();
}

using namespace PrimFit;
int main() {
    std::cout << "ASD" << std::endl;
    Mesh bbox = bbox_mesh(Eigen::Vector3d(0,0,0), Eigen::Vector3d(1,1,1));
    exportToPLY("../out.ply", bbox.V);
    return 0;
}