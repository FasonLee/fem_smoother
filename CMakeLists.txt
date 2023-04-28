cmake_minimum_required(VERSION 3.0.0)
project(femsmoother VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development NumPy)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(Protobuf REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})

FILE(GLOB_RECURSE SRC_LIST "proto/*.cc" "src/*.cc")

add_executable(${PROJECT_NAME} ${SOURCES} 
                               ${ALL_SOURCES} 
                               ${ALL_HEADERS}
                               ${SRC_LIST}
                               src/main.cpp)

target_link_libraries(${PROJECT_NAME} osqp::osqp
                                      OsqpEigen::OsqpEigen
                                      Python3::Python 
                                      Python3::NumPy
                                      ${PROTOBUF_LIBRARY_DEBUG}
                                      ${Boost_LIBRARIES})

target_include_directories(${PROJECT_NAME} PUBLIC  ${PROJECT_SOURCE_DIR}
                                                   ${Python3_INCLUDE_DIRS} 
                                                   ${Python3_NumPy_INCLUDE_DIRS})