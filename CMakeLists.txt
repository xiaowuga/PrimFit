cmake_minimum_required(VERSION 3.10)
project(PrimFit)

# 如果设置了 VCPKG_ROOT 环境变量（推荐使用这个名字）
if(DEFINED ENV{VCPKG_DIR})
    message(STATUS "Using Vcpkg from: $ENV{VCPKG_DIR}")
    set(CMAKE_TOOLCHAIN_FILE $ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake
            CACHE STRING "Vcpkg toolchain file" FORCE)
endif()


set(CMAKE_CXX_STANDARD 11)

#SET(Easy3d_DIR /Users/xiaowuga/Desktop/algo/Easy3D/build)
find_package(Easy3d CONFIG REQUIRED)

# Eigen3
find_package(Eigen3 CONFIG REQUIRED)

# CLI11
find_package(CLI11 CONFIG REQUIRED)

find_package(nlohmann_json CONFIG REQUIRED)

# CGAL
#find_package(CGAL CONFIG REQUIRED)

#Libigl
#find_package(libigl CONFIG REQUIRED)

option(FETCHCONTENT_FULLY_DISCONNECTED   "Disables all attempts to download or update content and assumes source dirs already exist" OFF)
set(FETCHCONTENT_BASE_DIR "${CMAKE_SOURCE_DIR}/external" CACHE PATH "Directory under which to collect all populated content")


#include(FetchContent)
#set(LIBIGL_WITH_EMBREE On)
#set(LIBIGL_WITH_CGAL On)
option(LIBIGL_WITH_EMBREE "Build Embree support in libigl" On)
option(LIBIGL_WITH_CGAL "Build CGAL support in libigl" On)

include(cmake/libigl.cmake)
#include(cmake/FindGMP.cmake)


### Configuration
set(QUADFIT_ROOT         ${CMAKE_CURRENT_LIST_DIR})

set(QUADFIT_glpk_DIR                ${QUADFIT_ROOT}/3rd_glpk)
set(QUADFIT_lpsolve_DIR             ${QUADFIT_ROOT}/3rd_lpsolve)
set(QUADFIT_scip_DIR                ${QUADFIT_ROOT}/3rd_scip)
set(QUADFIT_soplex_DIR              ${QUADFIT_ROOT}/3rd_soplex)


add_subdirectory(3rd_glpk)
add_subdirectory(3rd_lpsolve)
add_subdirectory(3rd_scip)
add_subdirectory(3rd_soplex)

add_subdirectory(lp)
add_subdirectory(arrangement)
add_subdirectory(quadrics)
add_subdirectory(algo)
add_subdirectory(io)
#add_subdirectory(gui)

add_executable(merge_step cli/merge_step_cli.cpp)
target_link_libraries(merge_step PUBLIC algo io)

add_executable(arrangement_step cli/arrangement_step_cli.cpp)
target_link_libraries(arrangement_step PUBLIC algo io arrangement)


add_executable(grow_step cli/grow_step_cli.cpp)
target_link_libraries(grow_step PUBLIC algo io)


add_executable(pblp_step cli/pblp_step_cli.cpp)
target_link_libraries(pblp_step PUBLIC algo io)

add_executable(comfirm_seg cli/comfirm_seg.cpp)
target_link_libraries(comfirm_seg PUBLIC algo io)


add_executable(comfirm_merge cli/comfirm_merge.cpp)
target_link_libraries(comfirm_merge PUBLIC algo io)

add_executable(comfirm_arrangement cli/comfirm_arrangement.cpp)
target_link_libraries(comfirm_arrangement PUBLIC algo io)

add_executable(comfirm_grow cli/comfirm_grow.cpp)
target_link_libraries(comfirm_grow PUBLIC algo io)

add_executable(remian_patch_valid_area cli/remian_patch_valid_area.cpp)
target_link_libraries(remian_patch_valid_area PUBLIC algo io)

add_executable(comfirm_curve cli/comfirm_curve.cpp)
target_link_libraries(comfirm_curve PUBLIC algo io)


add_executable(batch_pblp_step_cli cli/batch_pblp_step_cli.cpp)
target_link_libraries(batch_pblp_step_cli PUBLIC algo io easy3d::util)

add_executable(primfit_cli cli/primfit_cli.cpp)
target_link_libraries(primfit_cli PUBLIC io algo easy3d::fileio CLI11::CLI11 nlohmann_json::nlohmann_json)


add_subdirectory(efficient_ransac)

add_executable(Ransac2Seg cli/Ransac2Seg.cpp)
target_include_directories(Ransac2Seg PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Ransac2Seg PUBLIC io quadrics easy3d::viewer easy3d::util  igl::cgal algo CLI11::CLI11 nlohmann_json::nlohmann_json efficient_ransac)


add_executable(vg2seg cli/vg2seg.cpp)
target_include_directories(vg2seg PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(vg2seg PUBLIC io quadrics algo CLI11::CLI11)


add_executable(xyz2ply cli/xyz2ply.cpp)
target_include_directories(xyz2ply PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(xyz2ply PUBLIC easy3d::util easy3d::fileio easy3d::core)


install(TARGETS primfit_cli
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS Ransac2Seg
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS vg2seg
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

install(TARGETS primfit_cli
        RUNTIME DESTINATION bin)

install(TARGETS Ransac2Seg
        RUNTIME DESTINATION bin)

install(TARGETS vg2seg
        RUNTIME DESTINATION bin)