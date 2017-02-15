project(engine_test)

add_subdirectory(googletest)

configure_file(${PROJECT_SOURCE_DIR}/config.h.in ${PROJECT_BINARY_DIR}/config.h)
include_directories(${PROJECT_BINARY_DIR})

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

add_executable(run_tests test_aux.cpp test_physics.cpp test_linalg.cpp)

target_link_libraries(run_tests gtest gtest_main)
target_link_libraries(run_tests engine)

link_directories("/usr/lib64")
target_link_libraries(run_tests GLEW)

find_package(OpenGL REQUIRED)
target_link_libraries(run_tests ${OPENGL_gl_LIBRARY})
