cmake_minimum_required(VERSION 3.0.0)
project(quaternions VERSION 0.1.0 LANGUAGES C)

include(CTest)

#set(${GSL_ROOT_DIR} ${CMAKE_SOURCE_DIR}/gsl/build)
#find_package(GSL REQUIRED)
add_subdirectory(gsl)
link_libraries(gsl gslcblas)

add_subdirectory(src)
add_subdirectory(unity)
add_subdirectory(tests)
#add_subdirectory(gsl)