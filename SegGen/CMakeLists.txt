cmake_minimum_required(VERSION 3.17)
project(DataProcessing)

set(CMAKE_CXX_STANDARD 11)

#find_package(Easy3d)
#include(FetchContent)
#
#option(LIBIGL_WITH_CGAL "Build CGAL support in libigl" On)
#option(LIBIGL_WITH_OPENGL_GLFW "" On)
#set(LIBIGL_WITH_OPENGL_GLFW_IMGUI On)
#set(LIBIGL_WITH_OPENGL On)
#set(LIBIGL_WITH_OPENGL_GLFW On)


#include(cmake/libigl.cmake)

#add_subdirectory(io)
#add_subdirectory(quadrics)

#add_subdirectory(../efficient_ransac)

add_executable(vg2seg vg2seg.cpp util.cpp)
target_include_directories(vg2seg PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(vg2seg PUBLIC io quadrics)






add_executable(Ransac2Seg ../cli/Ransac2Seg.cpp util.cpp mesh_generater.cpp)
target_include_directories(Ransac2Seg PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ../efficient_ransac)
target_link_libraries(Ransac2Seg PUBLIC io quadrics easy3d::viewer easy3d::util efficient_ransac igl::cgal)





