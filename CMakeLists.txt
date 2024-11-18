cmake_minimum_required(VERSION 3.10)

PROJECT( TinyToys )

SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

#test
add_subdirectory(test)
#lib
add_subdirectory(fit_line_contour)
add_subdirectory(tensor_lib)
add_subdirectory(merge_img_cuda)
