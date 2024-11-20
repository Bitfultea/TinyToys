cmake_minimum_required(VERSION 3.20)

PROJECT( TinyToys )

set(CMAKE_C_COMPILER /usr/bin/gcc-10 CACHE PATH "" FORCE)
set(CMAKE_CXX_COMPILER /usr/bin/g++-10 CACHE PATH "" FORCE)
set(CMAKE_CXX_STANDARD 17) # 指定 C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON) # 确保强制使用指定版本
set(CMAKE_CXX_EXTENSIONS OFF) # 禁用非标准扩展（如 GNU 扩展）
SET(CMAKE_BUILD_TYPE "Release")

# IF(NOT WIN32)
#     SET(CMAKE_CXX_COMPILER /usr/bin/g++-10)
# ENDIF()

#test
add_subdirectory(test)
#lib
add_subdirectory(fit_line_contour)
add_subdirectory(tensor_lib)
add_subdirectory(merge_img_cuda)
