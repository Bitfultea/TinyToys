# OpenCV
FIND_PACKAGE(OpenCV REQUIRED)

ADD_EXECUTABLE(FitLine test_fitline.cpp)
TARGET_LINK_LIBRARIES(FitLine core ${OpenCV_LIBS})

ADD_EXECUTABLE(Tensor test_tensor.cpp)
TARGET_LINK_LIBRARIES(Tensor Tensor_lib)

ADD_EXECUTABLE(MergeImg test_mergeimg.cpp)
TARGET_LINK_LIBRARIES(MergeImg merge ${OpenCV_LIBS} ${CUDA_LIBRARIES})