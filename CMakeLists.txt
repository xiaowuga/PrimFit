cmake_minimum_required(VERSION 3.10)
project(PrimFit)

# 如果设置了 VCPKG_ROOT 环境变量（推荐使用这个名字）
if(DEFINED ENV{VCPKG_DIR})
    message(STATUS "Using Vcpkg from: $ENV{VCPKG_DIR}")
    set(CMAKE_TOOLCHAIN_FILE $ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake
            CACHE STRING "Vcpkg toolchain file" FORCE)
endif()


set(CMAKE_CXX_STANDARD 11)

find_package(Easy3d CONFIG REQUIRED)

# Eigen3
find_package(Eigen3 CONFIG REQUIRED)

# CLI11
find_package(CLI11 CONFIG REQUIRED)

find_package(nlohmann_json CONFIG REQUIRED)


option(FETCHCONTENT_FULLY_DISCONNECTED   "Disables all attempts to download or update content and assumes source dirs already exist" OFF)
set(FETCHCONTENT_BASE_DIR "${CMAKE_SOURCE_DIR}/external" CACHE PATH "Directory under which to collect all populated content")



option(LIBIGL_WITH_EMBREE "Build Embree support in libigl" On)
option(LIBIGL_WITH_CGAL "Build CGAL support in libigl" On)

include(cmake/libigl.cmake)



### Configuration
set(PRIMFIT_ROOT         ${CMAKE_CURRENT_LIST_DIR})

set(PRIMFIT_glpk_DIR                ${PRIMFIT_ROOT}/3rd_glpk)
set(PRIMFIT_lpsolve_DIR             ${PRIMFIT_ROOT}/3rd_lpsolve)
set(PRIMFIT_scip_DIR                ${PRIMFIT_ROOT}/3rd_scip)
set(PRIMFIT_soplex_DIR              ${PRIMFIT_ROOT}/3rd_soplex)


add_subdirectory(3rd_glpk)
add_subdirectory(3rd_lpsolve)
add_subdirectory(3rd_scip)
add_subdirectory(3rd_soplex)

add_subdirectory(lp)
add_subdirectory(arrangement)
add_subdirectory(quadrics)
add_subdirectory(algo)
add_subdirectory(io)



add_executable(primfit_cli cli/primfit_cli.cpp)
target_link_libraries(primfit_cli PUBLIC io algo easy3d::fileio CLI11::CLI11 nlohmann_json::nlohmann_json)
#target_compile_definitions(primfit_cli PUBLIC HAS_GUROBI)

add_subdirectory(efficient_ransac)

add_executable(Ransac2Seg cli/Ransac2Seg.cpp)
target_include_directories(Ransac2Seg PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(Ransac2Seg PUBLIC io quadrics easy3d::viewer easy3d::util  igl::cgal algo CLI11::CLI11 nlohmann_json::nlohmann_json efficient_ransac)


add_executable(vg2seg cli/vg2seg.cpp)
target_include_directories(vg2seg PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(vg2seg PUBLIC io quadrics algo CLI11::CLI11)

