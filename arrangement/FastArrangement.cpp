#include "FastArrangement.h"
#include "MatrixUtils.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/intersections.h>

#include <igl/copyleft/cgal/RemeshSelfIntersectionsParam.h>
#include <igl/copyleft/cgal/SelfIntersectMesh.h>
#include <igl/copyleft/cgal/extract_cells.h>
#include <igl/copyleft/cgal/propagate_winding_numbers.h>
#include <igl/extract_manifold_patches.h>
#include <igl/remove_unreferenced.h>
#include <igl/unique_edge_map.h>
#include <igl/write_triangle_mesh.h>

#include <solve_intersections.h>

#include <chrono>

using namespace PyMesh;
#ifdef __clang__
__attribute__((optnone))
#endif
void FastArrangement::run()
{
    auto t_begin = std::chrono::high_resolution_clock::now();

    std::vector<double> in_coords, out_coords;
    std::vector<uint> in_tris, out_tris, in_labels;
    std::vector<genericPoint*> gen_points;
    std::vector<std::bitset<NBIT>> out_labels;

    in_coords.reserve(m_vertices.size());
    std::copy(
        m_vertices.data(), m_vertices.data() + m_vertices.size(), std::back_inserter(in_coords));
    in_tris.reserve(m_faces.size());
    std::copy(m_faces.data(), m_faces.data() + m_faces.size(), std::back_inserter(in_tris));
    in_labels.reserve(m_vertices.size());
    const auto max_label = m_in_face_labels.maxCoeff();
    assert(max_label < NBIT);
    std::copy(m_in_face_labels.data(),
        m_in_face_labels.data() + m_in_face_labels.size(),
        std::back_inserter(in_labels));

    /*-------------------------------------------------------------------
     * There are 4 versions of the solveIntersections function. Please
     * refer to the solve_intersections.h file to see how to use them. */

    //igl::write_triangle_mesh("arrangement_debug.ply", m_vertices, m_faces, igl::FileEncoding::Binary);
    solveIntersections(in_coords, in_tris, in_labels, gen_points, out_tris, out_labels);

    auto t_mid = std::chrono::high_resolution_clock::now();

    // Copy source face labels over.
    assert(out_labels.size() == out_tris.size() / 3);
    m_out_face_labels.resize(out_labels.size());
    for (size_t i=0; i<m_out_face_labels.size(); i++) {
        const auto& bits = out_labels[i];
        m_out_face_labels[i] = max_label + 1;
        for (size_t j=0; j<NBIT; j++) {
            if (bits[j]) {
                m_out_face_labels[i] = static_cast<int>(j);
                break;
            }
        }
        assert(m_out_face_labels[i] <= max_label);
    }

    typedef CGAL::Epeck Kernel;
    typedef Kernel::FT ExactScalar;
    typedef Eigen::Matrix<ExactScalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixEr;
    MatrixEr resolved_vertices;
    MatrixIr resolved_faces;

    resolved_vertices.resize(gen_points.size() - 5, 3);
    const ExactScalar s = gen_points.back()->toExplicit3D().X();
    for (size_t i = 0; i < gen_points.size() - 5; i++) {
        auto p = gen_points[i];
        assert(p != nullptr);
        switch (p->getType()) {
        case EXPLICIT3D: {
            const auto& q = p->toExplicit3D();
            resolved_vertices.row(i) << q.X() / s, q.Y() / s, q.Z() / s;
        } break;
        case LPI: {
            // Line plane intersection.
            const auto& q = p->toLPI();
            Kernel::Point_3 P(q.P().X() / s, q.P().Y() / s, q.P().Z() / s);
            Kernel::Point_3 Q(q.Q().X() / s, q.Q().Y() / s, q.Q().Z() / s);
            Kernel::Point_3 R(q.R().X() / s, q.R().Y() / s, q.R().Z() / s);
            Kernel::Point_3 S(q.S().X() / s, q.S().Y() / s, q.S().Z() / s);
            Kernel::Point_3 T(q.T().X() / s, q.T().Y() / s, q.T().Z() / s);

            Kernel::Line_3 line(P, Q);
            Kernel::Plane_3 plane(R, S, T);
            auto r = CGAL::intersection(line, plane);
            if (r) {
                if (Kernel::Point_3* point = boost::get<Kernel::Point_3>(&*r)) {
                    resolved_vertices.row(i) << point->x(), point->y(), point->z();
                } else {
                    throw std::runtime_error("Line plane does not at a point!");
                }
            } else {
                throw std::runtime_error("Line plane intersection missing!");
            }
        } break;
        case TPI: {
            // 3 plane intersection.
            const auto& q = p->toTPI();
            Kernel::Point_3 V1(q.V1().X() / s, q.V1().Y() / s, q.V1().Z() / s);
            Kernel::Point_3 V2(q.V2().X() / s, q.V2().Y() / s, q.V2().Z() / s);
            Kernel::Point_3 V3(q.V3().X() / s, q.V3().Y() / s, q.V3().Z() / s);
            Kernel::Point_3 W1(q.W1().X() / s, q.W1().Y() / s, q.W1().Z() / s);
            Kernel::Point_3 W2(q.W2().X() / s, q.W2().Y() / s, q.W2().Z() / s);
            Kernel::Point_3 W3(q.W3().X() / s, q.W3().Y() / s, q.W3().Z() / s);
            Kernel::Point_3 U1(q.U1().X() / s, q.U1().Y() / s, q.U1().Z() / s);
            Kernel::Point_3 U2(q.U2().X() / s, q.U2().Y() / s, q.U2().Z() / s);
            Kernel::Point_3 U3(q.U3().X() / s, q.U3().Y() / s, q.U3().Z() / s);

            Kernel::Plane_3 v_plane(V1, V2, V3);
            Kernel::Plane_3 w_plane(W1, W2, W3);
            Kernel::Plane_3 u_plane(U1, U2, U3);

            auto r = CGAL::intersection(v_plane, w_plane, u_plane);
            if (r) {
                if (Kernel::Point_3* point = boost::get<Kernel::Point_3>(&*r)) {
                    resolved_vertices.row(i) << point->x(), point->y(), point->z();
                } else if (Kernel::Line_3* line = boost::get<Kernel::Line_3>(&*r)) {
                    throw std::runtime_error("3 planes intersect at a line!");
                } else if (Kernel::Plane_3* plane = boost::get<Kernel::Plane_3>(&*r)) {
                    throw std::runtime_error("3 planes intersect at a plane?!");
                } else {
                    throw std::runtime_error("3 planes does not intersect at a point!");
                }
            } else {
                throw std::runtime_error("3 planes does not intersect!");
            }
        } break;
        default: throw std::runtime_error("Unkonw generic point type encountered");
        }
    }

    resolved_faces.resize(out_tris.size() / 3, 3);
    std::copy(out_tris.begin(), out_tris.end(), resolved_faces.data());

    assert(resolved_faces.maxCoeff() < resolved_vertices.rows());

    // Build edge map
    Eigen::MatrixXi E, uE;
    Eigen::VectorXi EMAP;
    std::vector<std::vector<size_t>> uE2E;
    igl::unique_edge_map(resolved_faces, E, uE, EMAP, uE2E);

    // patches
    const size_t num_patches = igl::extract_manifold_patches(resolved_faces, EMAP, uE2E, m_patches);

    // cells
    const size_t num_cells = igl::copyleft::cgal::extract_cells(
        resolved_vertices, resolved_faces, m_patches, E, uE, uE2E, EMAP, m_cells);
    assert(m_cells.rows() == num_patches);
    assert(m_cells.cols() == 2);

    VectorI labels = VectorI::Zero(resolved_faces.rows());
    //// winding numbers
    // igl::copyleft::cgal::propagate_winding_numbers(
    //        resolved_vertices, resolved_faces,
    //        uE, uE2E, num_patches, m_patches, num_cells, m_cells,
    //        labels, m_winding_number);

    // Cast resolved mesh back to Float
    m_vertices = MatrixFr(resolved_vertices.rows(), resolved_vertices.cols());
    std::transform(resolved_vertices.data(),
        resolved_vertices.data() + resolved_vertices.size(),
        m_vertices.data(),
        [](const ExactScalar& val) { return CGAL::to_double(val); });
    m_faces = resolved_faces;

    auto t_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> resolve_time = t_mid - t_begin;
    std::chrono::duration<double> extract_time = t_end - t_mid;
    std::cout << "Arrangement: resolving self-intersection: " << resolve_time.count() << std::endl;
    std::cout << "Arrangement: extracting arrangement: " << extract_time.count() << std::endl;

    // Clean up
    // computeApproximateCoordinates(gen_points, out_coords);
    freePointsMemory(gen_points);
}
