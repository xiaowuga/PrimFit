cmake_minimum_required(VERSION 3.20)

project(PrimFit)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_subdirectory(scip)

find_package(libigl CONFIG REQUIRED)



add_executable(main src/main.cpp
        src/base.h)
target_include_directories(main PUBLIC src)
target_link_libraries(main PUBLIC igl::igl_core)


