cmake_minimum_required(VERSION 3.18)
project(still_detection_zupt VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 输出目录设置
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# 头文件目录（使用target_include_directories更现代）

# Windows 和 Linux 的编译选项处理
if(MSVC)
    # Windows MSVC 编译器选项
    add_compile_options(/W4 /Zi /utf-8)
    # 定义 _USE_MATH_DEFINES 以启用 M_PI 等数学常量
    # 使用 add_compile_definitions 替代 add_definitions（更现代的方法）
    add_compile_definitions(_USE_MATH_DEFINES)
    # 禁用一些 MSVC 安全函数警告（csv.h 中使用了标准 C 函数）
    add_compile_definitions(_CRT_SECURE_NO_WARNINGS)
else()
    # Linux/Unix 编译器选项
    add_compile_options(-Wall -g)
endif()

# ##########################################################
# # Setting path for 3rd party libraries
# ##########################################################
set(THIRD_PARTY_LIBRARY_DIR ${CMAKE_SOURCE_DIR}/../3rdParty CACHE PATH "The path of third party libraries.")  # set to the relative path to current CmakeLists.txt
if (NOT DEFINED CMAKE_BUILD_TYPE)
    message("CMAKE_BUILD_TYPE is not defined! Set it as Debug by default.")
    set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "The build type (Important for gRPC) for project.")
    # message("CMAKE_BUILD_TYPE is not defined! Set it as Release by default.")
    # set(CMAKE_BUILD_TYPE "Release" CACHE STRING "The build type (Important for gRPC) for project.")
endif()


# Eigen3
set(Eigen3_DIR ${THIRD_PARTY_LIBRARY_DIR}/Eigen/share/eigen3/cmake)
message(STATUS "Eigen3_DIR = ${Eigen3_DIR}")
find_package(Eigen3 REQUIRED PATHS ${Eigen3_DIR})
if(Eigen3_FOUND)
	message("eigen3 found!")
else()
	message(FATAL_ERROR "Can not find eigen3!")
endif()

# Matplot++
set(Matplot++_DIR ${THIRD_PARTY_LIBRARY_DIR}/Matplot++/lib/cmake/Matplot++)
message(STATUS "Matplot++_DIR = ${Matplot++_DIR}")
find_package(Matplot++ REQUIRED PATHS ${Matplot++_DIR})
if(Matplot++_FOUND)
	message("Matplot++ found!")
	add_compile_definitions(USE_MATPLOT)
else()
	message(FATAL_ERROR "Can not find Matplot++!")
endif()

# 源文件列表
set(SOURCES
    src/main.cpp
    src/iez_pdr.cpp
    src/utils.cpp
)

# 创建可执行文件
add_executable(${PROJECT_NAME} ${SOURCES})

# 设置头文件包含目录
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 链接库
if(UNIX)
    # Linux/Unix 需要链接 pthread
    target_link_libraries(${PROJECT_NAME} PRIVATE
        pthread
        Eigen3::Eigen
        Matplot++::matplot
    )
else()
    # Windows 链接 Eigen3 和 Matplot++
    target_link_libraries(${PROJECT_NAME} PRIVATE
        Eigen3::Eigen
        Matplot++::matplot
    )
endif()
