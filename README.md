# TinyToys
Collection of various algorithm implementations from daily usage

## Contain:
- Helcon fit_line_contour implementation of OpenCV


## Detail:
### fit_line_contour
See fit_line_contour/main.cpp for usage. The implementation detail is following algorithm from [直线卡尺工具原理](https://blog.csdn.net/weixin_45416828/article/details/136378557?spm=1001.2101.3001.6650.4&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-4-136378557-blog-119606955.235%5Ev43%5Econtrol&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-4-136378557-blog-119606955.235%5Ev43%5Econtrol) and [仿Halcon卡尺工具](https://blog.csdn.net/qq_37299618/article/details/119606955). The core is to find the edge which the slope change most significantly in the ROI box. 

### merge_image
Merge a set of image onto one image based on selected method. Currently support *sum* and *average*

### Tensor
Separate the Tensor data structure from the _OPEN3D_ library and modify it to suit my daily usage.  see documentation in [Open3D Website](https://www.open3d.org/html/cpp_api/classopen3d_1_1_tensor.html)
