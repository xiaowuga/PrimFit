/* This file is part of PyMesh. Copyright (c) 2016 by Qingnan Zhou */
#include "MeshArrangement.h"
#include "MatrixUtils.h"

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

#include <igl/copyleft/cgal/RemeshSelfIntersectionsParam.h>
#include <igl/copyleft/cgal/SelfIntersectMesh.h>
#include <igl/copyleft/cgal/extract_cells.h>
#include <igl/copyleft/cgal/propagate_winding_numbers.h>
#include <igl/extract_manifold_patches.h>
#include <igl/remove_unreferenced.h>
#include <igl/unique_edge_map.h>

#include <chrono>

using namespace PyMesh;

void MeshArrangement::run()
{
    typedef CGAL::Epeck Kernel;
    typedef Kernel::FT ExactScalar;
    typedef Eigen::Matrix<ExactScalar, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> MatrixEr;

    auto t_begin = std::chrono::high_resolution_clock::now();

    // Resolve self intersection
    igl::copyleft::cgal::RemeshSelfIntersectionsParam params;

    MatrixEr resolved_vertices;
    MatrixIr resolved_faces;
    {
        MatrixEr V;
        MatrixIr F;
        MatrixIr intersecting_faces;
        VectorI source_vertices;
        VectorI source_faces;
        igl::copyleft::cgal::SelfIntersectMesh<Kernel,
            MatrixDr,
            MatrixIr,
            MatrixEr,
            MatrixIr,
            MatrixIr,
            VectorI,
            VectorI>
            resolver(m_vertices,
                m_faces,
                params,
                V,
                F,
                intersecting_faces,
                source_faces,
                source_vertices);

        // Merge coinciding vertices into non-manifold vertices.
        std::for_each(F.data(),
            F.data() + F.size(),
            [&source_vertices](typename MatrixIr::Scalar& a) { a = source_vertices[a]; });

        // Remove unreferenced vertices.
        Eigen::VectorXi UIM;
        igl::remove_unreferenced(V, F, resolved_vertices, resolved_faces, UIM);

        // Map face labels
        m_out_face_labels.resize(resolved_faces.rows());
        for (Eigen::Index i=0; i<resolved_faces.rows(); i++) {
            m_out_face_labels[i] = m_in_face_labels[source_faces[i]];
        }
    }
    auto t_mid = std::chrono::high_resolution_clock::now();

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
    m_vertices = MatrixDr(resolved_vertices.rows(), resolved_vertices.cols());
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
}

