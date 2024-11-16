file(GLOB_RECURSE SOURCES "*.cpp" "*.cu")

# find_package(cuBLAS REQUIRED)
find_package(CUDA REQUIRED)
option(BUILD_CUDA_MODULE          "Build the CUDA module"                    OFF)
set(BUILD_CUDA_MODULE ON)
set(CMAKE_CUDA_FLAGS "${CMAKE_CUDA_FLAGS} --extended-lambda -lcusolver -lcublas")

# Enable CUDA as a language
enable_language(CUDA)

find_package(Eigen3 REQUIRED)

if(Eigen3_FOUND)
    message(STATUS "Found Eigen3: ${EIGEN3_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "Eigen3 not found!")
endif()

find_package(fmt REQUIRED)
find_package(OpenBLAS) #Fail to find OpenBLAS????
if(OpenBLAS_FOUND)
    message(STATUS "Found OpenBLAS: ${OpenBLAS_LIBRARIES}")
    message(STATUS "OpenBLAS include dirs: ${OpenBLAS_INCLUDE_DIRS}")
else()
    message(STATUS "OpenBLAS not found!")
    set(OpenBLAS_LIBRARIES /usr/lib/x86_64-linux-gnu/libopenblas.so)
endif()


include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CUDA_INCLUDE_DIRS})
include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

# manually link to OpenBLAS and cusolver
link_directories(/usr/lib/x86_64-linux-gnu)
set(CUDA_LIB_DIR /usr/local/cuda/lib64)
set(CUSOLVER_LIB ${CUDA_LIB_DIR}/libcusolver.so)


# ADD_LIBRARY(Tensor_lib SHARED ${SOURCES})
add_library(Tensor_lib SHARED ${SOURCES})

target_include_directories(Tensor_lib PUBLIC /usr/include/eigen3)
target_link_libraries(Tensor_lib
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
  fmt::fmt
  # ${MKL_LIBRARIES}
  # OpenBLAS::OpenBLAS
  ${OpenBLAS_LIBRARIES}
  ${CUBLAS_LIBRARIES}
  ${CUSOLVER_LIB}
  -llapacke
)

set_target_properties(Tensor_lib PROPERTIES
    CUDA_SEPARABLE_COMPILATION ON
    CUDA_ARCHITECTURES "86"
)
