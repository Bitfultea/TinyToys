#include <cuda.h>
#include <stdio.h>

#include <iostream>
#include <opencv2/highgui.hpp>
#include <string>

#include "cuda_runtime.h"
#include "opencv2/imgproc/imgproc.hpp"

#define BLOCK_SIZE 16

__global__ void stack_maxid(unsigned char *srcImage_list,
                            unsigned char *maxImage,
                            unsigned char *max_id_Image,
                            unsigned int width,
                            unsigned int height) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    int img_size = width * height;
    // int img_num = sizeof(*srcImage_list) / img_size;
    int img_num = 3;
    int max = 0;
    int idx = 0;
    for (int i = 0; i < img_num; i++) {
        if (srcImage_list[(i * img_size + y * width + x)] > max) {
            max = srcImage_list[(i * img_size + y * width + x)];
            idx = i;
        }
        // max = i * 200 + 10;
    }

    maxImage[(y * width + x)] = max;
    max_id_Image[(y * width + x)] = idx;
}

__global__ stack_max(unsigned char *srcImage_list,
                     unsigned char *maxImage,
                     unsigned int width,
                     unsigned int height) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    int img_size = width * height;
    // int img_num = sizeof(*srcImage_list) / img_size;
    int img_num = 3;
    int max = 0;
    for (int i = 0; i < img_num; i++) {
        if (srcImage_list[(i * img_size + y * width + x)] > max) {
            max = srcImage_list[(i * img_size + y * width + x)];
        }
        // max = i * 200 + 10;
    }

    maxImage[(y * width + x)] = max;
}

__global__ stack_average(unsigned char *srcImage_list,
                         unsigned char *aveImage,
                         unsigned int width,
                         unsigned int height) {
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;

    int img_size = width * height;
    int img_num = 3;
    float average = 0;
    for (int i = 0; i < img_num; i++) {
        if (srcImage_list[(i * img_size + y * width + x)] > max) {
            average += srcImage_list[(i * img_size + y * width + x)];
        }
        // max = i * 200 + 10;
    }
    average = average / 3.0;
    aveImage[(y * width + x)] = max;
}

extern "C" void get_max_id_GPU_wrapper(const std::vector<cv::Mat> &input,
                                       cv::Mat &output) {
    cudaEvent_t start, stop;
    cv::Mat max_debug(output.size(), output.type());
    cudaEventCreate(&start);
    cudaEventCreate(&stop);

    const int inputSize = input[0].cols * input[0].rows * input.size();
    const int outputSize = output.cols * output.rows;
    unsigned char *d_input, *d_output, *d_max;
    // unsigned char *d_output, *d_max;

    // Allocate device memory
    // cv::Mat *d_mats;
    // cudaMalloc((void **)&d_mats, mats.size() * sizeof(cv::Mat));
    cudaMalloc<unsigned char>(&d_input, inputSize);
    cudaMalloc<unsigned char>(&d_output, outputSize);
    cudaMalloc<unsigned char>(&d_max, outputSize);

    // Copy data from OpenCV input image to device memory
    // cudaMemcpy(d_input, &input, inputSize, cudaMemcpyHostToDevice);
    for (int i = 0; i < input.size(); i++) {
        cudaMemcpy(d_input + i * input[0].total(), input[i].ptr(),
                   input[i].total(), cudaMemcpyHostToDevice);
    }

    // Specify block size
    const dim3 block(BLOCK_SIZE, BLOCK_SIZE);

    // Calculate grid size to cover the whole image
    const dim3 grid((output.cols + block.x - 1) / block.x,
                    (output.rows + block.y - 1) / block.y);

    // Start time
    cudaEventRecord(start);

    // Run Sobel Edge Detection Filter kernel on CUDA
    stack_maxid<<<grid, block>>>(d_input, d_max, d_output, output.cols,
                                 output.rows);

    // Stop time
    cudaEventRecord(stop);

    // Copy data from device memory to output image
    cudaMemcpy(output.ptr(), d_output, outputSize, cudaMemcpyDeviceToHost);
    cudaMemcpy(max_debug.ptr(), d_max, outputSize, cudaMemcpyDeviceToHost);

    // Free the device memory
    cudaFree(d_input);
    cudaFree(d_output);
    cudaFree(d_max);

    cudaEventSynchronize(stop);
    float milliseconds = 0;

    max_debug.convertTo(max_debug, CV_32F, 1.0 / 255, 0);
    max_debug *= 255;
    cv::imwrite("max_debug.jpg", max_debug);

    // Calculate elapsed time in milisecond
    cudaEventElapsedTime(&milliseconds, start, stop);
    std::cout << "\nMerge Processing time on GPU (ms): " << milliseconds
              << "\n";
}