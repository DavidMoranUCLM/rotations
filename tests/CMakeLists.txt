
add_executable(quaternions_test quaternions_test.c)
target_include_directories(quaternions_test PUBLIC ../include)
target_link_libraries(quaternions_test quaternions unity)
target_link_libraries(quaternions_test m)
add_test(quaternions quaternions_test)

add_executable(rotations_test rotations_test.c)
target_include_directories(rotations_test PUBLIC ../include)
target_link_libraries(rotations_test rotations unity)
add_test(rotations rotations_test)

add_executable(rotation_matrix_test rotation_matrix_test.c)
target_include_directories(rotation_matrix_test PUBLIC ../include)
target_link_libraries(rotation_matrix_test rotation_matrix unity)
add_test(rotations_matrix rotation_matrix_test)

