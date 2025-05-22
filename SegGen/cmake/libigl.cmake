include_guard()

include(FetchContent)
FetchContent_Declare(
        libigl
        GIT_REPOSITORY https://github.com/libigl/libigl.git
        GIT_TAG qnzhou/ui
)
FetchContent_GetProperties(libigl)
if(libigl_POPULATED)
    return()
endif()
FetchContent_Populate(libigl)

list(APPEND CMAKE_MODULE_PATH ${libigl_SOURCE_DIR}/cmake)
include(${libigl_SOURCE_DIR}/cmake/libigl.cmake ${libigl_BINARY_DIR})