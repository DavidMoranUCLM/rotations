cmake_minimum_required(VERSION 3.0.0)
project(quaternions VERSION 0.1.0 LANGUAGES C)

include(CTest)

find_package(GSL REQUIRED)
link_libraries(GSL::gsl GSL::gslcblas)

add_subdirectory(src)
add_subdirectory(unity)
add_subdirectory(tests)
#add_subdirectory(gsl)