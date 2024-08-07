cmake_minimum_required(VERSION 3.12)

project(efficient_ransac)
#!
# @brief Add sources from directories
# @details
# Add the sources from the specified \p directories to variable \p var
# and place them in the specified \p folder if non empty.
# @param[out] var name of the variable that receives the result list
# @param[in] folder the name of the folder
# @param[in] directories list of directories to scan
# took from geogram(inria?)
#
function(aux_source_directories var folder)
    set(sources)
    foreach(dir ${ARGN})
        file(GLOB _sources RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "${dir}/*.[ch]" "${dir}/*.[ch]pp")
        list(APPEND sources ${_sources})
    endforeach()
    
    if( NOT folder STREQUAL "")
        source_group(${folder} FILES ${sources})
    endif()

    #message("\nDEBUG: aux_source_directories: current_source_dir=${CMAKE_CURRENT_SOURCE_DIR} dirs=${ARGN}):\n${sources}\n")
    set(${var} ${${var}} ${sources} PARENT_SCOPE)
endfunction()

include_directories(.)

aux_source_directories(LIBSRCS "src" .)
aux_source_directories(LIBSRCS "src/GfxTL" GfxTL)
aux_source_directories(LIBSRCS "src/MiscLib" MiscLib)

add_library(efficient_ransac STATIC ${LIBSRCS})
