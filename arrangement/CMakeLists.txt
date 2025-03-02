get_filename_component(module ${CMAKE_CURRENT_SOURCE_DIR} NAME)


add_library(arrangement STATIC
        ${CMAKE_CURRENT_SOURCE_DIR}/Arrangement.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/MeshArrangement.cpp)
target_include_directories(arrangement PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(arrangement PUBLIC igl::core CGAL::CGAL Eigen3::Eigen)