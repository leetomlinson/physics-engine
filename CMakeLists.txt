cmake_minimum_required(VERSION 3.6)
project(physics_engine)

set(CMAKE_CXX_STANDARD 11)

configure_file(${PROJECT_SOURCE_DIR}/config.h.in ${PROJECT_BINARY_DIR}/config.h)
include_directories(${PROJECT_BINARY_DIR} boost_1_63_0)

set(SOURCE_FILES main.cpp)
add_executable(physics_engine ${SOURCE_FILES})

include_directories(engine)
include_directories("glm")

add_subdirectory(engine)
add_subdirectory(engine_test)

target_link_libraries(physics_engine engine)

link_directories("/usr/lib64")
target_link_libraries(physics_engine GLEW)

find_package(glfw3 3.2 REQUIRED)
target_link_libraries(physics_engine glfw)

find_package(OpenGL REQUIRED)
target_link_libraries(physics_engine ${OPENGL_gl_LIBRARY})
