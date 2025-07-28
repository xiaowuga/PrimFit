//
// Created by xiaowuga on 2025/7/28.
//
#include <easy3d/core/point_cloud.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/util/file_system.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>


bool load_xyz_with_normals(const std::string& filename, easy3d::PointCloud* cloud) {
    std::ifstream in(filename);
    if (!in.is_open()) {
        std::cerr << "Failed to open: " << filename << std::endl;
        return false;
    }

    auto points = cloud->add_vertex_property<easy3d::vec3>("v:point");
    auto normals = cloud->add_vertex_property<easy3d::vec3>("v:normal");

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) continue;
        std::istringstream iss(line);
        float x, y, z, nx, ny, nz;
        if (!(iss >> x >> y >> z >> nx >> ny >> nz)) {
            std::cerr << "Invalid line: " << line << std::endl;
            continue;
        }

        easy3d::PointCloud::Vertex v = cloud->add_vertex(easy3d::vec3(x, y, z));
        normals[v] = easy3d::vec3(nx, ny, nz);
    }

    return true;
}



int main() {
    std::string input_dir = "D:\\code\\PrimFit\\data\\subset_abc_xyz";
    std::string output_dir = "D:\\code\\PrimFit\\data\\subset_abc_ply";

    if (!easy3d::file_system::is_directory(output_dir)) {
        easy3d::file_system::create_directory(output_dir);
    }
    std::vector<std::string>files;
    easy3d::file_system::get_files(input_dir, files, false);
    std::cout << files.size() << std::endl;
    for (const auto& entry : files) {
        if (easy3d::file_system::extension(entry) == "xyz") {
            std::string base_name = easy3d::file_system::base_name(entry);
            std::string input_file = input_dir + "/" + entry;
            std::string output_file = output_dir + "/" + base_name + ".ply";

            std::cout << "Converting: " << input_file << " -> " << output_file << std::endl;

            auto* cloud = new easy3d::PointCloud();
            if (!load_xyz_with_normals(input_file, cloud)) {
                std::cerr << "Error loading " << entry << std::endl;
                delete cloud;
                continue;
            }
            easy3d::PointCloudIO::save(output_file, cloud);

            delete cloud;
        }
    }

    std::cout << "Batch conversion complete." << std::endl;
    return 0;
}
