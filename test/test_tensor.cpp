#include <iostream>

#include "../tensor_lib/Tensor.h"
#include "../tensor_lib/TensorCheck.h"
using namespace open3d::core;

int main(int argc, char *argv[]) {
    // create tensor with shape {3, 3}
    Tensor tensor = Tensor::Empty({3, 3}, Float32, Device("CPU:0"));
    // std::cout << tensor << std::endl;
    tensor.SetItem(TensorKey::Index(2), Tensor({1}, Float32));
    // std::cout << tensor << std::endl;

    Tensor id_tensor = Tensor::Eye(3, Float32, Device("CPU:0"));
    Tensor c = Tensor::Init<float>({0, 1, 2});
}