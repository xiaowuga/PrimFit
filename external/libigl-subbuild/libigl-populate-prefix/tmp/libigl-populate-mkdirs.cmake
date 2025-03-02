# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/code/PrimFit/external/libigl-src"
  "D:/code/PrimFit/external/libigl-build"
  "D:/code/PrimFit/external/libigl-subbuild/libigl-populate-prefix"
  "D:/code/PrimFit/external/libigl-subbuild/libigl-populate-prefix/tmp"
  "D:/code/PrimFit/external/libigl-subbuild/libigl-populate-prefix/src/libigl-populate-stamp"
  "D:/code/PrimFit/external/libigl-subbuild/libigl-populate-prefix/src"
  "D:/code/PrimFit/external/libigl-subbuild/libigl-populate-prefix/src/libigl-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/code/PrimFit/external/libigl-subbuild/libigl-populate-prefix/src/libigl-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/code/PrimFit/external/libigl-subbuild/libigl-populate-prefix/src/libigl-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
