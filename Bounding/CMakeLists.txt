cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(Cuda-project)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake/cuda" ${CMAKE_MODULE_PATH})

find_package(CUDA QUIET REQUIRED)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories("/usr/local/cula/include")
link_directories(${PCL_LIBRARY_DIRS})
link_directories("/usr/local/cula/lib64")
add_definitions(${PCL_DEFINITIONS})

CUDA_ADD_EXECUTABLE(pcl_bounding_box_demo BoundingBoxTeste.cpp kernels.cu boundingbox.cpp ./src/alglibinternal.cpp ./src/alglibmisc.cpp ./src/ap.cpp ./src/dataanalysis.cpp ./src/diffequations.cpp ./src/fasttransforms.cpp ./src/integration.cpp ./src/interpolation.cpp ./src/linalg.cpp ./src/optimization.cpp ./src/solvers.cpp ./src/specialfunctions.cpp ./src/statistics.cpp )
target_link_libraries (pcl_bounding_box_demo ${PCL_LIBRARIES} )
