#pragma once
#include <iostream>
// #include <opencv2/cudaimgproc.hpp>
#include <omp.h>

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

bool merge_app(std::vector<cv::Mat> &img_list, cv::Mat &result_img);