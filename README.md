# TinyToys
Collection of various algorithm implementations from daily usage

## Required Lib
- sudo apt-get install liblapacke-dev
- sudo apt-get install libopenblas-dev

## Contain:
- Helcon fit_line_contour implementation of OpenCV
- Tensor lib for C++ from Open3D


## Detail:
### fit_line_contour
See fit_line_contour/main.cpp for usage. The implementation detail is following algorithm from [直线卡尺工具原理](https://blog.csdn.net/weixin_45416828/article/details/136378557?spm=1001.2101.3001.6650.4&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-4-136378557-blog-119606955.235%5Ev43%5Econtrol&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-4-136378557-blog-119606955.235%5Ev43%5Econtrol) and [仿Halcon卡尺工具](https://blog.csdn.net/qq_37299618/article/details/119606955). The core is to find the edge which the slope change most significantly in the ROI box. 

### merge_image
Merge a set of image onto one image based on selected method. Currently support *sum* and *average*

### Tensor
Separate the Tensor data structure from the _OPEN3D_ library and modify it to suit my daily usage.  see documentation in [Open3D Website](https://www.open3d.org/html/cpp_api/classopen3d_1_1_tensor.html).

For exmaple:
In Numpy: 
```python
t = np.empty((4, 5), dtype=np.float32)
t[2] = np.empty((5,), dtype=np.float32)
t[0:4:2] = np.empty((2, 5), dtype=np.float32)
t[2, 0:4:2] = np.empty((2, 5), dtype=np.float32)
```
In C++
```cpp
Tensor t({4, 5}, Dtype::Float32);
t.SetItem(TensorIndex(2), Tensor({5}, Dtype::Float32));
t.SetItem(TensorSlice(0, 4, 2), Tensor({2, 5}, Dtype::Float32));
t.SetItem({TensorIndex(2), TensorSlice(0, 4, 2)},
          Tensor({2, 5}, Dtype::Float32));
```
In Numpy:
```python
t = np.empty((4, 5), dtype=np.float32)
t1 = t[2]
t2 = t[0:4:2]
t3 = t[1, 0:4:2]
```
In C++:
```cpp
Tensor t({4, 5}, Dtype::Float32);
Tensor t1 = t.GetItem(TensorIndex(2));
Tensor t2 = t.GetItem(TensorSlice(0, 4, 2));
Tensor t3 = t.GetItem({TensorIndex(2), TensorSlice(0, 4, 2)});
```
There are some examples to use Tensor lib in the project
```cpp
/// Create a 0-D tensor (scalar) with given value,
core::Tensor::Init<float>(0);

/// Create a 1-D tensor with initializer list,
core::Tensor::Init<float>({0, 1, 2});

/// Create a 2-D tensor with nested initializer list,
core::Tensor::Init<float>({{0, 1, 2}, {3, 4, 5}});


/// examples for creating tensors
core::Device device = core::Device("CPU:0");
core::Device gpu = core::Device("CUDA:0")
a = core::Tensor::Zeros({3}, core::Float32, device);
b = core::Tensor::Eye(3, core::Int32, gpu);
c = core::Tensor::Ones({3}, core::Int8, device);
d = core::Tensor::Ones({3}, core::Bool, gpu);
```