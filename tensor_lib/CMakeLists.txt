file(GLOB SOURCES "*.cpp")

find_package(CUDA REQUIRED)
option(BUILD_CUDA_MODULE          "Build the CUDA module"                    OFF)
set(BUILD_CUDA_MODULE ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CUDA_INCLUDE_DIRS})

add_library(Tensor_lib SHARED ${SOURCES})

target_link_libraries(Tensor_lib
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
)