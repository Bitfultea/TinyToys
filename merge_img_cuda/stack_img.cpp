#include "stack_img.h"

extern "C" bool laplacianFilter_GPU_wrapper(const cv::Mat &input,
                                            cv::Mat &output);
extern "C" bool tvFilter_GPU_wrapper(const cv::Mat &input, cv::Mat &output);
extern "C" bool sobelFilter_GPU_wrapper(const cv::Mat &input, cv::Mat &output);
extern "C" void stackimg_GPU_wrapper(int stack_mode,
                                     const std::vector<cv::Mat> &input,
                                     cv::Mat &output);

bool merge_app(std::vector<cv::Mat> &img_list, cv::Mat &result_img) {
    std::vector<cv::Mat> process_img_list;
#pragma omp parallel for
    for (auto img : img_list) {
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
        process_img_list.emplace_back(img);
    }

    cv::Mat dst(process_img_list[0].rows, process_img_list[0].cols, CV_8U);

    stackimg_GPU_wrapper(1, process_img_list, dst);

    result_img = dst;
    cv::imwrite("result.png", result_img);

    return true;
}