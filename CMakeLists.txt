cmake_minimum_required(VERSION 3.0)
project(test_code_cpp)

set(CMAKE_CXX_STANDARD 11)
#set(CSV_CXX_STANDARD 11)
include_directories(
        "/home/em-hsu/test_code_cpp/include"
)
set(Eigen_DIR "/usr/include/eigen3")
#file(GLOB project_SOURCES "OpenShoe/*.cpp")

add_definitions("-Wall -g")
#add_subdirectory(src)
#message(STATUS "LOG path: ${DIR_CURR}")

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
# 查找当前目录下所有源文件名，并将其存储到DIR_CURR中
aux_source_directory(. DIR_CURR)
add_executable(${PROJECT_NAME} ${DIR_CURR}) #生成可执行文件
# ADD_LIBRARY(${PROJECT_NAME} SHARED ${DIR_CURR})#生成共享库

target_link_libraries(
        ${PROJECT_NAME}
        pthread
        #eigen3
        )
