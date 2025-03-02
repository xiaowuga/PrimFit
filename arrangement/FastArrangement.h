#pragma once

#include "Arrangement.h"

namespace PyMesh {

class FastArrangement final : public Arrangement
{
public:
    using Base = Arrangement;

public:
    FastArrangement(const MatrixFr& vertices, const MatrixIr& faces, const VectorI& face_labels)
        : Base(vertices, faces, face_labels)
    {}
    ~FastArrangement() = default;

#ifdef __clang__
    __attribute__((optnone))
#endif
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
