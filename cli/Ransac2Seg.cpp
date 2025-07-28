#include <iostream>
#include <easy3d/core/point_cloud.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/viewer/viewer.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/drawable_triangles.h>
#include <easy3d/util/file_system.h>


#include <efficient_ransac/RansacShapeDetector.h>
#include <efficient_ransac/PlanePrimitiveShapeConstructor.h>
#include <efficient_ransac/CylinderPrimitiveShapeConstructor.h>
#include <efficient_ransac/SpherePrimitiveShapeConstructor.h>
#include <efficient_ransac/ConePrimitiveShapeConstructor.h>
#include <efficient_ransac/TorusPrimitiveShapeConstructor.h>
#include <efficient_ransac/PlanePrimitiveShape.h>
#include <efficient_ransac/SpherePrimitiveShape.h>
#include <efficient_ransac/CylinderPrimitiveShape.h>
#include <efficient_ransac/ConePrimitiveShape.h>
#include <efficient_ransac/TorusPrimitiveShape.h>

#include <mesh_generater.h>
#include <util.h>
#include <io.h>
#include <CLI/CLI.hpp>
#include <nlohmann/json.hpp>

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

int main(int argc, char **argv) {

    std::string config_path;
    CLI::App app{"DEFILLET Command Line"};

    app.add_option("-c,--config", config_path, "Configure file")->required();

    CLI11_PARSE(app, argc, argv);
    using json = nlohmann::json;
    std::ifstream file(config_path);
    json j;
    file >> j;
    PointCloud_Ransac pc;
    std::string path = j["input_path"];
    std::string out_seg_path =  j["output_path"];
    std::string bn = easy3d::file_system::base_name(path);
    // std::string directorty = "C:\\Users\\xiaowuga\\Desktop\\ICCV2023rebuttal\\failcase\\";
    easy3d::PointCloud* cloud = easy3d::PointCloudIO::load(path);
    int num_points = cloud->n_vertices();
    pc.resize(cloud->n_vertices());
    auto pts = cloud->points();
    auto nms = cloud->get_vertex_property<vec3>("v:normal").vector();
    bool is_save = true ;
    // easy3d::Viewer viewer("test");
    // viewer.add_model(cloud);

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
    PrimFit::Mesh_Generater meshGenerater(box);
    int m_minSupport = j["m_minSupport"];
    float m_epsilon = j["m_epsilon"];
    float m_bitmapEpsilon = j["m_bitmapEpsilon"];
    float m_normalThresh = j["m_normalThresh"];
    float m_probability = j["m_probability"];
    RansacShapeDetector::Options ransacOptions;
    ransacOptions.m_minSupport = m_minSupport;
    ransacOptions.m_epsilon = m_epsilon * pc.getScale();
    ransacOptions.m_bitmapEpsilon = m_bitmapEpsilon * pc.getScale();
    ransacOptions.m_normalThresh = m_normalThresh;
    ransacOptions.m_probability = m_probability;

    RansacShapeDetector detector(ransacOptions); // the detector object
    bool use_plane = j["use_plane"];
    bool use_cylinder = j["use_cylinder"];
    bool use_sphere = j["use_sphere"];
    bool use_cone  = j["use_cone"];
    bool use_torus = j["use_torus"];
    if(use_plane)
        detector.Add(new PlanePrimitiveShapeConstructor());
    if(use_cylinder)
        detector.Add(new CylinderPrimitiveShapeConstructor());
    if(use_sphere)
        detector.Add(new SpherePrimitiveShapeConstructor());
    if(use_cone)
        detector.Add(new ConePrimitiveShapeConstructor());
    if(use_torus)
        detector.Add(new TorusPrimitiveShapeConstructor());
    MiscLib::Vector<std::pair<MiscLib::RefCountPtr<PrimitiveShape>, size_t> > shapes; // stores the detected shapes

    std::size_t remaining = detector.Detect(pc, 0, pc.size(), &shapes); // run detection

    easy3d::PointCloud* out_cloud = new easy3d::PointCloud;
    PointCloud_Ransac::reverse_iterator start = pc.rbegin();
    MiscLib::Vector<std::pair<MiscLib::RefCountPtr<PrimitiveShape>, std::size_t> >::const_iterator shape_itr = shapes.begin();

    auto primitive_types = cloud->vertex_property<int>("v:primitive_type", -1);
    auto primitive_indices = cloud->vertex_property<int>("v:primitive_index", -1);
    primitive_types.vector().assign(cloud->n_vertices(), -1);
    primitive_indices.vector().assign(cloud->n_vertices(), -1);

    std::vector<PrimFit::SurfacePrimitive*> my_shapes;
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
                PrimFit::SurfaceParameters para;
                para.pos = pos; para.dir = dir; para.r1 = -dot(mean, dir); para.r2 = 0;
                PrimFit::SurfacePrimitive* ss = PrimFit::construct_bytype(PrimFit::SurfaceType::PLANE, para);
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
                // easy3d::TrianglesDrawable* dd = new TrianglesDrawable(std::to_string(asd++));
                // dd->update_vertex_buffer(pp);
                // dd->update_element_buffer(uini);
                // viewer.add_drawable(dd);
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
                PrimFit::SurfaceParameters para;
                para.pos = position; para.dir = dir; para.r1 = radius; para.r2 = 0;
                PrimFit::SurfacePrimitive* ss = PrimFit::construct_bytype(PrimFit::SurfaceType::CYLINDER, para);


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
                std::vector<PrimFit::SurfacePrimitive *> tangent_planes;
                ss->setColor(easy3d::random_color());
                meshGenerater.generate_cylinder_mesh(ss, tangent_planes, pp, ini);
                std::vector<unsigned int> uini(ini.size());

                for(size_t i = 0; i < uini.size(); i++) {
                    uini[i] = ini[i];
                }

                // easy3d::TrianglesDrawable* dd = new TrianglesDrawable(std::to_string(asd++));
                // dd->update_vertex_buffer(pp);
                // dd->update_element_buffer(uini);
                // viewer.add_drawable(dd);

                break;
            }
            case 1: {
                // parameters are discarded
                const Sphere& sphere = dynamic_cast<const SpherePrimitiveShape*>(primitive)->Internal();
                double radius = sphere.Radius();
                const Vec3f& center = sphere.Center();
                PrimFit::SurfaceParameters para;
                para.pos = easy3d::vec3(center.getValue()); para.dir = easy3d::vec3(0,0,0); para.r1 = radius; para.r2 = 0;
                PrimFit::SurfacePrimitive* ss = PrimFit::construct_bytype(PrimFit::SurfaceType::SPHERE, para);


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
                std::vector<PrimFit::SurfacePrimitive *> tangent_planes;
                ss->setColor(easy3d::random_color());
                meshGenerater.generate_sphere_mesh(ss, tangent_planes, pp, ini);
                std::vector<unsigned int> uini(ini.size());

                for(size_t i = 0; i < uini.size(); i++) {
                    uini[i] = ini[i];
                }

                // easy3d::TrianglesDrawable* dd = new TrianglesDrawable(std::to_string(asd++));
                // dd->update_vertex_buffer(pp);
                // dd->update_element_buffer(uini);
                // viewer.add_drawable(dd);

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

                PrimFit::SurfaceParameters para;
                para.pos = easy3d::vec3(pos.getValue()); para.dir = easy3d::vec3(dir.getValue()); para.r1 = angle; para.r2 = 0;
                PrimFit::SurfacePrimitive* ss = PrimFit::construct_bytype(PrimFit::SurfaceType::CONE, para);



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
                std::vector<PrimFit::SurfacePrimitive *> tangent_planes;
                ss->setColor(easy3d::random_color());
                meshGenerater.generate_cone_mesh(ss, tangent_planes, pp, ini);
                std::vector<unsigned int> uini(ini.size());

                for(size_t i = 0; i < uini.size(); i++) {
                    uini[i] = ini[i];
                }

                // easy3d::TrianglesDrawable* dd = new TrianglesDrawable(std::to_string(asd++));
                // dd->update_vertex_buffer(pp);
                // dd->update_element_buffer(uini);
                // viewer.add_drawable(dd);

                break;
            }
            case 4: {
                // parameters are discarded
                const Torus& torus = dynamic_cast<const TorusPrimitiveShape*>(primitive)->Internal();
                const Vec3f& center = torus.Center();
                const Vec3f& dir = torus.AxisDirection();
                double min_radius = torus.MinorRadius();
                double max_radius = torus.MajorRadius();

                PrimFit::SurfaceParameters para;
                para.pos = easy3d::vec3(center.getValue()); para.dir = easy3d::vec3(dir.getValue()); para.r1 = max_radius; para.r2 = min_radius;
                PrimFit::SurfacePrimitive* ss = PrimFit::construct_bytype(PrimFit::SurfaceType::TORUS, para);



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
                std::vector<PrimFit::SurfacePrimitive *> tangent_planes;
                ss->setColor(easy3d::random_color());
                meshGenerater.generate_torus_mesh(ss, tangent_planes, pp, ini);
                std::vector<unsigned int> uini(ini.size());

                for(size_t i = 0; i < uini.size(); i++) {
                    uini[i] = ini[i];
                }

                // easy3d::TrianglesDrawable* dd = new TrianglesDrawable(std::to_string(asd++));
                // dd->update_vertex_buffer(pp);
                // dd->update_element_buffer(uini);
                // viewer.add_drawable(dd);
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
    PrimFit::Util::construct_std_from_SurfacePrimitive(my_shapes, type, pos, dir, r1, r2, colors);


    if(is_save) {
        PrimFit::IO::save_segment(out_seg_path, type, pos, dir, r1, r2, points, normals, segments, colors);
    }



    // auto ssr = cloud->vertex_property<int>("v:primitive_index");
    // const std::string color_name = "v:color-segments";
    // auto coloring = cloud->vertex_property<vec3>(color_name, vec3(0, 0, 0));
    // easy3d::Renderer::color_from_segmentation(cloud, ssr, coloring);
    // auto drawable = cloud->renderer()->get_points_drawable("vertices");
    // drawable->set_property_coloring(State::VERTEX, color_name);
    //
    //
    // drawable->update();
    // viewer.update();
    // viewer.run();
    return 0;
}
