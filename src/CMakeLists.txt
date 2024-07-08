add_library(quaternions STATIC quaternions.c)
target_include_directories(quaternions PUBLIC ../include)
target_link_libraries(quaternions m)