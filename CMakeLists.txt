cmake_minimum_required(VERSION 3.22)
project(ECDevice)


# find IGH ethercat master
set(ETHERLAB_DIR /usr/local/etherlab)
find_library( ETHERCAT_LIBRARY NAMES libethercat.so PATHS ${ETHERLAB_DIR}/lib/ )

# find third parts lib
find_package(spdlog REQUIRED)
find_package(yaml-cpp REQUIRED)




set(CMAKE_CXX_STANDARD 20)
include_directories(ECDevice)

include_directories(${ETHERLAB_DIR}/include)
link_libraries(${ETHERCAT_LIBRARY})

add_executable(ECDevice main.cpp)
target_link_libraries(ECDevice spdlog::spdlog yaml-cpp::yaml-cpp)

add_executable(dev dev.cpp)
target_link_libraries(dev spdlog::spdlog yaml-cpp::yaml-cpp)
