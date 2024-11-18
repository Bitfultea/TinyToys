#include "../merge_img_cuda/stack_img.h"

int main() {
    std::vector<cv::Mat> img_list;

    cv::Mat img1 = cv::imread(
            "/home/charles/Data/downloads/江门C8极耳翻折侧面/ear_flip/1/"
            "30915_15_39_55_467_JQ22C240623194603006.jpg");
    cv::Mat img2 = cv::imread(
            "/home/charles/Data/downloads/江门C8极耳翻折侧面/ear_flip/1/"
            "30917_15_39_56_692_JQ22C240623194603006.jpg");
    cv::Mat img3 = cv::imread(
            "/home/charles/Data/downloads/江门C8极耳翻折侧面/ear_flip/1/"
            "31099_18_43_17_741_JQ22C240623194603006.jpg");
    img_list.push_back(img1);
    img_list.push_back(img2);
    img_list.push_back(img3);

    cv::Mat result_img;
    merge_app(img_list, result_img);

    return 0;
}