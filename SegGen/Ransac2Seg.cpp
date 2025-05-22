#include <iostream>
#include <easy3d/core/point_cloud.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/viewer/viewer.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/util/file_system.h>


#include <util.h>

#include <RansacShapeDetector.h>
#include <PlanePrimitiveShapeConstructor.h>
#include <CylinderPrimitiveShapeConstructor.h>
#include <SpherePrimitiveShapeConstructor.h>
#include <ConePrimitiveShapeConstructor.h>
#include <TorusPrimitiveShapeConstructor.h>
#include <PlanePrimitiveShape.h>
#include <SpherePrimitiveShape.h>
#include <CylinderPrimitiveShape.h>
#include <ConePrimitiveShape.h>
#include <TorusPrimitiveShape.h>

#include <mesh_generater.h>



typedef ::PointCloud PointCloud_Ransac;

using namespace easy3d;


//enum PrimType { // values have to be exactly the same as in RANSAC
//    PLANE = 0,
//    SPHERE = 1,
//    CYLINDER = 2,
//    CONE = 3,
//    TORUS = 4,
//    UNKNOWN = -1
//};

int main() {


    PointCloud_Ransac pc;
    std::string path = "C:\\Users\\xiaowuga\\Desktop\\ICCV2023rebuttal\\failcase\\castle.ply";
    std::string bn = easy3d::file_system::base_name(path);
    std::string directorty = "C:\\Users\\xiaowuga\\Desktop\\ICCV2023rebuttal\\failcase\\";
    easy3d::PointCloud* cloud = easy3d::PointCloudIO::load(path);
    int num_points = cloud->n_vertices();
    pc.resize(cloud->n_vertices());
    auto pts = cloud->points();
    auto nms = cloud->get_vertex_property<vec3>("v:normal").vector();
    bool is_save = true ;
    easy3d::Viewer viewer("test");
    viewer.add_model(cloud);

    for(size_t i = 0; i < num_points; i++) {
        const vec3 &p = pts[i];
        const vec3 &n = nms[i];
        pc[i] = Point(
                Vec3f(p.x, p.y, p.z),
                Vec3f(n.x, n.y, n.z)
        );
        pc[i].index = i;
    }


    easy3d::Box3 box = cloud->bounding_box();
    pc.setBBox(
            Vec3f(static_cast<float>(box.min_coord(0)), static_cast<float>(box.min_coord(1)),
                  static_cast<float>(box.min_coord(2))),
            Vec3f(static_cast<float>(box.max_coord(0)), static_cast<float>(box.max_coord(1)),
                  static_cast<float>(box.max_coord(2)))
    );
    QuadFit::Mesh_Generater meshGenerater(box);


    RansacShapeDetector::Options ransacOptions;
    ransacOptions.m_minSupport = 20;
    ransacOptions.m_epsilon = 0.004 * pc.getScale();
    ransacOptions.m_bitmapEpsilon = 0.02 * pc.getScale();
    ransacOptions.m_normalThresh = 0.8;
    ransacOptions.m_probability = 0.001f;

    RansacShapeDetector detector(ransacOptions); // the detector object
    detector.Add(new PlanePrimitiveShapeConstructor());
    detector.Add(new CylinderPrimitiveShapeConstructor());
    detector.Add(new SpherePrimitiveShapeConstructor());
    detector.Add(new ConePrimitiveShapeConstructor());
//    detector.Add(new TorusPrimitiveShapeConstructor());

    MiscLib::Vector<std::pair<MiscLib::RefCountPtr<PrimitiveShape>, size_t> > shapes; // stores the detected shapes

    std::size_t remaining = detector.Detect(pc, 0, pc.size(), &shapes); // run detection

    easy3d::PointCloud* out_cloud = new easy3d::PointCloud;
    PointCloud_Ransac::reverse_iterator start = pc.rbegin();
    MiscLib::Vector<std::pair<MiscLib::RefCountPtr<PrimitiveShape>, std::size_t> >::const_iterator shape_itr = shapes.begin();

    auto primitive_types = cloud->vertex_property<int>("v:primitive_type", -1);
    auto primitive_indices = cloud->vertex_property<int>("v:primitive_index", -1);
    primitive_types.vector().assign(cloud->n_vertices(), -1);
    primitive_indices.vector().assign(cloud->n_vertices(), -1);

    std::vector<QuadFit::SurfacePrimitive*> my_shapes;
    std::vector<std::vector<int>>segments;
    std::vector<easy3d::vec3> points;
    std::vector<easy3d::vec3> normals;
    for(; shape_itr != shapes.end(); ++shape_itr) {
        const PrimitiveShape *primitive = shape_itr->first;
        std::size_t num = shape_itr->second;

        std::list<int> vts;
        PointCloud_Ransac::reverse_iterator point_itr = start;
        for (std::size_t count = 0; count < num; ++count) {
            int v = int(point_itr->index);
            vts.push_back(v);
            ++point_itr;
        }
        start = point_itr;

        if (num < ransacOptions.m_minSupport)
            continue;
        int asd = 0;
        switch (primitive->Identifier()) {
            case 0: {
                // parameters are discarded
                const Plane &pl = dynamic_cast<const PlanePrimitiveShape *>(primitive)->Internal();
                const Vec3f &p = pl.getPosition();
                const Vec3f &n = pl.getNormal();
                const Plane3 plane(vec3(p.getValue()), vec3(n.getValue()));
                easy3d::vec3 mean = easy3d::vec3(0,0,0);
                for(auto id : vts) {
                    const easy3d::PointCloud::Vertex v(id);
                    mean += cloud->position(v);
                }
                mean /= vts.size();
                easy3d::vec3 pos = mean;
                easy3d::vec3 dir = plane.normal().normalize();
                QuadFit::SurfaceParameters para;
                para.pos = pos; para.dir = dir; para.r1 = -dot(mean, dir); para.r2 = 0;
                QuadFit::SurfacePrimitive* ss = QuadFit::construct_bytype(QuadFit::SurfaceType::PLANE, para);
                int index = my_shapes.size();
                std::vector<int>tmp;
                for (auto id: vts) {
                    const easy3d::PointCloud::Vertex v(id);
                    easy3d::vec3 p = cloud->position(v);
                    int ii = points.size();
                    points.emplace_back(p);
                    normals.emplace_back(nms[v.idx()]);
                    tmp.emplace_back(ii);
                    primitive_types[v] = 0;
                    primitive_indices[v] = index;
                }
                segments.emplace_back(tmp);
                my_shapes.push_back(ss);
                std::vector<vec3>pp;
                std::vector<int>ini;
                ss->setColor(easy3d::random_color());
                meshGenerater.generate_plane_mesh(ss, pp, ini);
                std::vector<unsigned int> uini(ini.size());

                for(size_t i = 0; i < uini.size(); i++) {
                    uini[i] = ini[i];
                }
                easy3d::TrianglesDrawable* dd = new TrianglesDrawable(std::to_string(asd++));
                dd->update_vertex_buffer(pp);
                dd->update_element_buffer(uini);
                viewer.add_drawable(dd);
                break;
            }
            case 2: {
                // parameters are discarded
                const Cylinder &cylinder = dynamic_cast<const CylinderPrimitiveShape *>(primitive)->Internal();
                double radius = cylinder.Radius();
                const Vec3f &pos = cylinder.AxisPosition();
                const Vec3f &nor = cylinder.AxisDirection();
                const vec3 position(pos[0], pos[1], pos[2]);
                vec3 dir(nor[0], nor[1], nor[2]);
                dir = normalize(dir);
                QuadFit::SurfaceParameters para;
                para.pos = position; para.dir = dir; para.r1 = radius; para.r2 = 0;
                QuadFit::SurfacePrimitive* ss = QuadFit::construct_bytype(QuadFit::SurfaceType::CYLINDER, para);


                int index = my_shapes.size();
                std::vector<int>tmp;
                for (auto id: vts) {
                    const easy3d::PointCloud::Vertex v(id);
                    easy3d::vec3 p = cloud->position(v);
                    int ii = points.size();
                    points.emplace_back(p);
                    normals.emplace_back(nms[v.idx()]);
                    tmp.emplace_back(ii);
                    primitive_types[v] = 2;
                    primitive_indices[v] = index;
                }
                segments.emplace_back(tmp);
                my_shapes.push_back(ss);

                std::vector<vec3>pp;
                std::vector<int>ini;
                std::vector<QuadFit::SurfacePrimitive *> tangent_planes;
                ss->setColor(easy3d::random_color());
                meshGenerater.generate_cylinder_mesh(ss, tangent_planes, pp, ini);
                std::vector<unsigned int> uini(ini.size());

                for(size_t i = 0; i < uini.size(); i++) {
                    uini[i] = ini[i];
                }

                easy3d::TrianglesDrawable* dd = new TrianglesDrawable(std::to_string(asd++));
                dd->update_vertex_buffer(pp);
                dd->update_element_buffer(uini);
                viewer.add_drawable(dd);

                break;
            }
            case 1: {
                // parameters are discarded
                const Sphere& sphere = dynamic_cast<const SpherePrimitiveShape*>(primitive)->Internal();
                double radius = sphere.Radius();
                const Vec3f& center = sphere.Center();
                QuadFit::SurfaceParameters para;
                para.pos = easy3d::vec3(center.getValue()); para.dir = easy3d::vec3(0,0,0); para.r1 = radius; para.r2 = 0;
                QuadFit::SurfacePrimitive* ss = QuadFit::construct_bytype(QuadFit::SurfaceType::SPHERE, para);


                int index = my_shapes.size();
                std::vector<int>tmp;
                for (auto id: vts) {
                    const easy3d::PointCloud::Vertex v(id);
                    easy3d::vec3 p = cloud->position(v);
                    int ii = points.size();
                    points.emplace_back(p);
                    normals.emplace_back(nms[v.idx()]);
                    tmp.emplace_back(ii);
                    primitive_types[v] = 1;
                    primitive_indices[v] = index;
                }
                segments.emplace_back(tmp);
                my_shapes.push_back(ss);

                std::vector<vec3>pp;
                std::vector<int>ini;
                std::vector<QuadFit::SurfacePrimitive *> tangent_planes;
                ss->setColor(easy3d::random_color());
                meshGenerater.generate_sphere_mesh(ss, tangent_planes, pp, ini);
                std::vector<unsigned int> uini(ini.size());

                for(size_t i = 0; i < uini.size(); i++) {
                    uini[i] = ini[i];
                }

                easy3d::TrianglesDrawable* dd = new TrianglesDrawable(std::to_string(asd++));
                dd->update_vertex_buffer(pp);
                dd->update_element_buffer(uini);
                viewer.add_drawable(dd);

                break;
            }

            case 3: {
                // NOTE:: the center is the apex of the cone
                // parameters are discarded
                const Cone& cone = dynamic_cast<const ConePrimitiveShape*>(primitive)->Internal();
                const Vec3f& dir = cone.AxisDirection();
                const Vec3f& pos = cone.Center();
                double angle = cone.Angle();
                double radius = cone.RadiusAtLength(1.0f);

                QuadFit::SurfaceParameters para;
                para.pos = easy3d::vec3(pos.getValue()); para.dir = easy3d::vec3(dir.getValue()); para.r1 = angle; para.r2 = 0;
                QuadFit::SurfacePrimitive* ss = QuadFit::construct_bytype(QuadFit::SurfaceType::CONE, para);



                int index = my_shapes.size();
                std::vector<int>tmp;
                for (auto id: vts) {
                    const easy3d::PointCloud::Vertex v(id);
                    easy3d::vec3 p = cloud->position(v);
                    int ii = points.size();
                    points.emplace_back(p);
                    normals.emplace_back(nms[v.idx()]);
                    tmp.emplace_back(ii);
                    primitive_types[v] = 3;
                    primitive_indices[v] = index;
                }
                segments.emplace_back(tmp);
                my_shapes.push_back(ss);

                std::vector<vec3>pp;
                std::vector<int>ini;
                std::vector<QuadFit::SurfacePrimitive *> tangent_planes;
                ss->setColor(easy3d::random_color());
                meshGenerater.generate_cone_mesh(ss, tangent_planes, pp, ini);
                std::vector<unsigned int> uini(ini.size());

                for(size_t i = 0; i < uini.size(); i++) {
                    uini[i] = ini[i];
                }

                easy3d::TrianglesDrawable* dd = new TrianglesDrawable(std::to_string(asd++));
                dd->update_vertex_buffer(pp);
                dd->update_element_buffer(uini);
                viewer.add_drawable(dd);

                break;
            }
            case 4: {
                // parameters are discarded
                const Torus& torus = dynamic_cast<const TorusPrimitiveShape*>(primitive)->Internal();
                const Vec3f& center = torus.Center();
                const Vec3f& dir = torus.AxisDirection();
                double min_radius = torus.MinorRadius();
                double max_radius = torus.MajorRadius();

                QuadFit::SurfaceParameters para;
                para.pos = easy3d::vec3(center.getValue()); para.dir = easy3d::vec3(dir.getValue()); para.r1 = max_radius; para.r2 = min_radius;
                QuadFit::SurfacePrimitive* ss = QuadFit::construct_bytype(QuadFit::SurfaceType::TORUS, para);



                int index = my_shapes.size();
                std::vector<int>tmp;
                for (auto id: vts) {
                    const easy3d::PointCloud::Vertex v(id);
                    easy3d::vec3 p = cloud->position(v);
                    int ii = points.size();
                    points.emplace_back(p);
                    normals.emplace_back(nms[v.idx()]);
                    tmp.emplace_back(ii);
                    primitive_types[v] = 4;
                    primitive_indices[v] = index;
                }
                segments.emplace_back(tmp);
                my_shapes.push_back(ss);

                std::vector<vec3>pp;
                std::vector<int>ini;
                std::vector<QuadFit::SurfacePrimitive *> tangent_planes;
                ss->setColor(easy3d::random_color());
                meshGenerater.generate_torus_mesh(ss, tangent_planes, pp, ini);
                std::vector<unsigned int> uini(ini.size());

                for(size_t i = 0; i < uini.size(); i++) {
                    uini[i] = ini[i];
                }

                easy3d::TrianglesDrawable* dd = new TrianglesDrawable(std::to_string(asd++));
                dd->update_vertex_buffer(pp);
                dd->update_element_buffer(uini);
                viewer.add_drawable(dd);
                break;
            }
        }
    }


    std::vector<int> type;
    std::vector<easy3d::vec3> pos;
    std::vector<easy3d::vec3> dir;
    std::vector<float> r1;
    std::vector<float> r2;
    std::vector<easy3d::vec3> colors;
    QuadFit::Util::construct_std_from_SurfacePrimitive(my_shapes, type, pos, dir, r1, r2, colors);

    std::string out_seg_path = directorty + bn + ".seg";
    if(is_save) {
        QuadFit::IO::save_segment(out_seg_path, type, pos, dir, r1, r2, points, normals, segments, colors);
    }



    auto ssr = cloud->vertex_property<int>("v:primitive_index");
    const std::string color_name = "v:color-segments";
    auto coloring = cloud->vertex_property<vec3>(color_name, vec3(0, 0, 0));
    easy3d::Renderer::color_from_segmentation(cloud, ssr, coloring);
    auto drawable = cloud->renderer()->get_points_drawable("vertices");
    drawable->set_property_coloring(State::VERTEX, color_name);


    drawable->update();
    viewer.update();
    viewer.run();
    return 0;
}
