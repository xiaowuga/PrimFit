#pragma once

#include "Arrangement.h"

namespace PyMesh {

class MeshArrangement final : public Arrangement
{
public:
    using Base = Arrangement;

public:
    MeshArrangement(const MatrixDr& vertices, const MatrixIr& faces, const VectorI& face_labels)
        : Base(vertices, faces, face_labels)
    {}
    ~MeshArrangement() = default;
    void run() override;

private:
    using Base::m_cells;
    using Base::m_faces;
    using Base::m_patches;
    using Base::m_in_face_labels;
    using Base::m_out_face_labels;
    using Base::m_vertices;
    using Base::m_winding_number;
};

} // namespace PyMesh
