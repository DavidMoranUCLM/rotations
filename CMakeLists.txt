cmake_minimum_required(VERSION 3.0.0)
project(rotations VERSION 0.1.0 LANGUAGES C)

include(CTest)

add_subdirectory(gsl)
link_libraries(gsl gslcblas)

add_subdirectory(src)
add_subdirectory(unity)
add_subdirectory(tests)