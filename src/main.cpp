#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <base.h>

#include <igl/writePLY.h>

using namespace Eigen;
using namespace PrimFit;




int main() {
    Vector3d p1(0, 4, 3);
    Vector3d p2(1, 2, 1);
    Vector3d p3(3, 0, 1);
    Eigen::MatrixXd V;
    V.resize(3, 3);
    V(0,0) = p1.x(); V(0,1) = p1.y(); V(0,2) = p1.z();
    V(1,0) = p2.x(); V(1,1) = p2.y(); V(1,2) = p2.z();
    V(2,0) = p3.x(); V(2,1) = p3.y(); V(2,2) = p3.z();
    int div = 5;

    Vector3d dir(0,0,1);
//    Mesh cylinder = cone_mesh(p1, dir, 1.0, EIGEN_PI / 3, 1.0);
    Mesh sphere = torus_mesh(p1, dir, 3.0, 1.0);
    igl::writePLY("../out.ply", sphere.V, sphere.F);
//    std::cout << plane.V << std::endl;
//    std::cout << plane.F << std::endl;
    return 0;
}
