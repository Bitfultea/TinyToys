#include "core.h"
#include <iostream>

int main(int argc, char *argv[]) {
    std::string name = argv[1];
    std::cout << "Test " << name << std::endl;

    cv::Mat img = cv::imread(name);
    cv::Rect rect(0, 0, img.cols, img.rows);
    core core(img, rect, cv::Point2f(200, 500), cv::Point2f(3000, 1000), 10, 100, 200);
    // core.crop_img();

    cv::imwrite("result.png", img);

    // test
    core.Process();
    return 0;
}
