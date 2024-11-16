#include <iostream>

#include "../tensor_lib/EigenConverter.h"
#include "../tensor_lib/Tensor.h"
#include "../tensor_lib/TensorCheck.h"
using namespace open3d::core;

int main(int argc, char *argv[]) {
    // create tensor with shape {3, 3}
    Tensor tensor = Tensor::Empty({3, 3}, Float32, Device("CPU:0"));
    Eigen::MatrixXf eigen_matrix =
            eigen_converter::TensorToEigenMatrixXf(tensor);
    std::cout << eigen_matrix << std::endl;
    std::cout << "----------------" << std::endl;

    // std::cout << tensor << std::endl;
    tensor.SetItem(TensorKey::Index(2), Tensor::Ones({3}, Float32));
    eigen_matrix = eigen_converter::TensorToEigenMatrixXf(tensor);
    std::cout << eigen_matrix << std::endl;
    std::cout << "----------------" << std::endl;

    Tensor id_tensor = Tensor::Eye(3, Float32, Device("CUDA:0"));
    // convert to eigen is done by copying
    eigen_matrix = eigen_converter::TensorToEigenMatrixXf(id_tensor);

    std::cout << eigen_matrix << std::endl;
    std::cout << id_tensor.GetDevice().ToString() << std::endl;
    // Tensor c = Tensor::Init<float>({0, 1, 2});
}