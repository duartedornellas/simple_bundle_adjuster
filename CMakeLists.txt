# cmake -DCMAKE_BUILD_TYPE=Debug .
# cmake -DCMAKE_BUILD_TYPE=Release .

cmake_minimum_required(VERSION 2.8)

project(simple_bundle_adjuster)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})

# helloworld
add_executable(simple_bundle_adjuster simple_bundle_adjuster.cc)
target_link_libraries(simple_bundle_adjuster ${CERES_LIBRARIES})
