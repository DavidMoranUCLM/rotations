add_library(quaternions STATIC quaternions.c)
target_include_directories(quaternions PUBLIC ../include)
target_link_libraries(quaternions m)

add_library(rotations STATIC rotations.c)
target_include_directories(rotations PUBLIC ../include)

add_library(rotation_matrix STATIC rotation_matrix.c)
target_include_directories(rotation_matrix PUBLIC ../include)