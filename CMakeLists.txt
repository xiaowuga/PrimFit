cmake_minimum_required(VERSION 3.20)

project(PrimFit)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_subdirectory(ransac)
add_subdirectory(scip)

find_package(Eigen3 CONFIG REQUIRED)
find_package(CGAL CONFIG REQUIRED)
find_package(libigl CONFIG REQUIRED)

add_executable(main src/main.cpp
        src/mesh_boolean.h
        src/mesh_boolean.cpp)
target_include_directories(main PUBLIC src)
target_link_libraries(main PRIVATE Eigen3::Eigen)
target_link_libraries(main PRIVATE CGAL::CGAL)
target_link_libraries(main PRIVATE igl::igl_core igl_copyleft::igl_copyleft_core igl_copyleft::igl_copyleft_cgal)

add_executable(test_ransac src/test_ransac.cpp)
target_include_directories(test_ransac PUBLIC ${CMAKE_CURRENT_LIST_DIR}/ransac)
target_link_libraries(test_ransac PUBLIC ransac)

#add_executable(test_unique_edge src/test_unique_edge_map.cpp)
#target_link_libraries(test_unique_edge PRIVATE Eigen3::Eigen)
#target_link_libraries(test_unique_edge PRIVATE CGAL::CGAL)
#target_link_libraries(test_unique_edge PRIVATE igl::igl_core igl_copyleft::igl_copyleft_core igl_copyleft::igl_copyleft_cgal)


#add_executable(test_mesh_boolean src/test_mesh_boolean.cpp src/mesh_boolean.cpp src/mesh_boolean.h)
#target_link_libraries(test_mesh_boolean PRIVATE Eigen3::Eigen)
#target_link_libraries(test_mesh_boolean PRIVATE CGAL::CGAL)
#target_link_libraries(test_mesh_boolean PRIVATE igl::igl_core igl_copyleft::igl_copyleft_core igl_copyleft::igl_copyleft_cgal)