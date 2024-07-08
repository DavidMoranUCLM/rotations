
add_executable(unity_test unity_test.c)
target_include_directories(unity_test PUBLIC ../include)
target_link_libraries(unity_test quaternions unity)
target_link_libraries(unity_test m)
add_test(test1 unity_test)

