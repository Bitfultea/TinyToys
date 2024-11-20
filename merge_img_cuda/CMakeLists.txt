set(CUDA_ARCH "86") # rtx 3090

# OpenCV
FIND_PACKAGE(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

# Compile merge.cpp into a shared library
CUDA_ADD_LIBRARY(merge SHARED 
    stack_img.cpp
    laplacianFilter.cu
    sobelEdgeDetectionFilter.cu
    tvFilter.cu
    stack_img.cu
    )

# TARGET_LINK_LIBRARIES(merge ${OpenCV_LIBS} ${CUDA_LIBRARIES} NumCpp::NumCpp)
TARGET_LINK_LIBRARIES(merge ${OpenCV_LIBS} ${CUDA_LIBRARIES} )
set_target_properties(merge PROPERTIES
  CUDA_ARCHITECTURES ${CUDA_ARCH}
  CUDA_SEPARABLE_COMPILATION ON
  )


