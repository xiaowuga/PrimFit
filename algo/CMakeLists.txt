cmake_minimum_required(VERSION 3.12)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})


add_library(${PROJECT_NAME} STATIC
        quadrics.cpp
        mesh_generater.cpp
        primitive_merger.cpp
        partition_simplifier.cpp
        pblp.cpp  util.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME} PUBLIC easy3d::core igl::embree
        igl::core quadrics  arrangement lp 3rd_scip 3rd_glpk)
