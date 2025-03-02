cmake_minimum_required(VERSION 3.1)

get_filename_component(PROJECT_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${PROJECT_NAME})


#set(math_HEADERS
#        linear_program.h
#        linear_program_solver.h)

set(math_SOURCES
        linear_program.cpp
        linear_program_io.cpp
        linear_program_solver.cpp
        linear_program_solver_GLPK.cpp
        linear_program_solver_LPSOLVE.cpp
        linear_program_solver_SCIP.cpp
        linear_program_solver_GUROBI.cpp
        )


add_library(${PROJECT_NAME} STATIC ${math_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR})

target_include_directories(${PROJECT_NAME} PRIVATE
        ${QUADFIT_scip_DIR}
        ${QUADFIT_scip_DIR}/scip)

if (MSVC)
    target_compile_definitions(${PROJECT_NAME} PRIVATE
            _CRT_SECURE_NO_WARNINGS
            _CRT_SECURE_NO_DEPRECATE
            )
endif()

include(../cmake/FindGUROBI.cmake)
if (GUROBI_FOUND)
    message(STATUS "Gurobi include dir: " ${GUROBI_INCLUDE_DIRS})
    message(STATUS "Gurobi libraries: " ${GUROBI_LIBRARIES})

    target_compile_definitions(${PROJECT_NAME} PUBLIC HAS_GUROBI)

    target_include_directories(${PROJECT_NAME} PRIVATE ${GUROBI_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${GUROBI_LIBRARIES})
endif()


target_link_libraries( ${PROJECT_NAME} PRIVATE 3rd_scip 3rd_lpsolve 3rd_glpk 3rd_soplex )
#target_link_libraries( ${PROJECT_NAME} PRIVATE 3rd_scip )