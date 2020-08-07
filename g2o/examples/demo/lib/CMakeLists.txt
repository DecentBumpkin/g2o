cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 11)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
add_library (json_io SHARED json_io.cpp)

# when other libraries or executables link to Hello
target_include_directories (json_io PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/../LivoxStatic/RapidJSON/include/
)

target_link_libraries (json_io Eigen3::Eigen)

set_target_properties(json_io PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/build)
set_target_properties(json_io PROPERTIES OUTPUT_NAME json_io)
set_target_properties(json_io PROPERTIES PUBLIC_HEADER json_io.hpp)