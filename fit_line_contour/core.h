#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <vector>
#include <iostream>
#include <math.h>

class core {
private:
    /* data */
    cv::Mat img_;
    cv::Rect rect_;
    cv::Mat crop_img_;
    std::vector<std::vector<cv::Point2f>> RoIs_;
    cv::Point2f start_;
    cv::Point2f end_;

    cv::Point2f dir_;
    cv::Point2f orthDir_;
    cv::Vec4f line_;

    float length_;

    int num_ROIs_;
    int ROI_width_;  // parallel with the line
    int ROI_height_; // orthogonal to the line

    /*privete function*/
    void crop_img();
    void gen_ROIs();
    void interpolate_ROI(std::vector<cv::Point2f> Roi, cv::Mat &Roi_interpolated_map);
    float interpolate(cv::Point2f p);
    int find_edge(std::vector<cv::Point2f> Roi);
    void draw_cross_sign(cv::Mat &img, cv::Point2f p);
    void ransac_line_filter(const std::vector<cv::Point2f> &points,
                            std::vector<cv::Point2f> &outlierPoints,
                            std::vector<cv::Point2f> &inlierPoints, double sigma);

public:
    core(cv::Mat img, cv::Rect rect, cv::Point2f start, cv::Point2f end, int num_ROIs,
         int ROI_width = 20, int ROI_height = 60);
    ~core();

    void Process();
    void write_roi_bboxes();
};
