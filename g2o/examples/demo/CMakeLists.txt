cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)
find_package(yaml-cpp)
include_directories(${YAML_CPP_INCLUDE_DIRS})

add_executable(demo
    demo.cpp
    types_demo.cpp
)

include_directories(
    ${CHOLMOD_INCLUDE_DIR}
    ../../../../LivoxStatic/RapidJSON/include/
)

target_link_libraries(demo core solver_cholmod types_slam3d)

add_executable(demo_static
    demo_static.cpp
    types_static.cpp
)
target_link_libraries(demo_static core solver_cholmod types_slam3d ${YAML_CPP_LIBRARIES})