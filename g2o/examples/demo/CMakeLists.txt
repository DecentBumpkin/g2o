add_executable(demo
    demo.cpp
    types_demo.cpp
)

include_directories(${CHOLMOD_INCLUDE_DIR})

target_link_libraries(demo core solver_cholmod types_slam3d)

add_executable(demo_static
    demo_static.cpp
    types_static.cpp
)
target_link_libraries(demo_static core solver_cholmod types_slam3d)