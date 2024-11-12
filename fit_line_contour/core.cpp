#include "core.h"
core::core(cv::Mat img, cv::Rect rect, cv::Point2f start, cv::Point2f end, int num_ROIs,
           int ROI_width, int ROI_height) {
    img_ = img;
    rect_ = rect;
    crop_img_ = img_(rect_).clone();
    // convert to grayscale
    cv::cvtColor(crop_img_, crop_img_, cv::COLOR_BGR2GRAY);

    ROI_width_ = ROI_width;
    ROI_height_ = ROI_height;

    start_ = start;
    end_ = end;

    num_ROIs_ = num_ROIs; // useless
    num_ROIs_ = std::floor(length_ / (1.5 * ROI_width_));
}

core::~core() {
    img_.release();
    crop_img_.release();
}

void core::gen_ROIs() {
    cv::Point2f dir = end_ - start_;
    length_ = cv::norm(dir);
    dir_ = dir / length_;
    // set the gap between two ROIs as half of the ROI width
    int numRoIs = std::floor(length_ / (1.5 * ROI_width_));
    orthDir_ = cv::Point2f(-dir_.y, dir_.x); // anti-clockwise as positive y

    for (int i = 0; i < numRoIs; i++) {
        cv::Point2f center = start_ + (((i * 1.5) + 1) * ROI_width_) * dir_;
        cv::Point2f p1 = center + orthDir_ * ROI_height_ / 2 - dir_ * ROI_width_ / 2;
        cv::Point2f p2 = center + orthDir_ * ROI_height_ / 2 + dir_ * ROI_width_ / 2;
        cv::Point2f p3 = center - orthDir_ * ROI_height_ / 2 + dir_ * ROI_width_ / 2;
        cv::Point2f p4 = center - orthDir_ * ROI_height_ / 2 - dir_ * ROI_width_ / 2;

        std::vector<cv::Point2f> roi;
        roi.push_back(p1);
        roi.push_back(p2);
        roi.push_back(p3);
        roi.push_back(p4);
        RoIs_.push_back(roi);
    }
    std::cout << "There are " << RoIs_.size() << " ROIs" << std::endl;
}

void core::write_roi_bboxes() {
    cv::Mat color_img;
    cv::cvtColor(crop_img_, color_img, cv::COLOR_GRAY2BGR);
    int numRoIs = std::floor(length_ / (1.5 * ROI_width_));
    for (int i = 0; i < numRoIs; i++) {
        // roi box on image
        for (int j = 0; j < 4; j++) {
            cv::line(color_img, RoIs_[i][j], RoIs_[i][(j + 1) % 4], cv::Scalar{0, 255, 0},
                     2);
        }
    }
    // draw fit line
    // get line point and slope
    cv::Point point0;
    point0.x = line_[2];
    point0.y = line_[3];
    double k = line_[1] / line_[0];

    // calculate the start and end point
    cv::Point point1, point2;
    point1.x = 0;
    point1.y = k * (point1.x - point0.x) + point0.y;
    point2.x = color_img.cols - 1;
    point2.y = k * (point2.x - point0.x) + point0.y;

    cv::line(color_img, point1, point2, cv::Scalar(0, 0, 255), 2);

    cv::imwrite("result.png", color_img);
}

// Base on the method of https://blog.csdn.net/qq_37299618/article/details/119606955
float core::interpolate(cv::Point2f p) {
    // p is the point to be interpolated with corresponding four nearest pixel points

    cv::Point2i p1{std::floor(p.x), std::floor(p.y)};
    float vp1 = (float)crop_img_.at<uchar>(p1.y, p1.x) / 255.0;
    cv::Point2i p2{std::ceil(p.x), std::floor(p.y)};
    float vp2 = (float)crop_img_.at<uchar>(p2.y, p2.x) / 255.0;
    cv::Point2i p3{std::ceil(p.x), std::ceil(p.y)};
    float vp3 = (float)crop_img_.at<uchar>(p3.y, p3.x) / 255.0;
    cv::Point2i p4{std::floor(p.x), std::ceil(p.y)};
    float vp4 = (float)crop_img_.at<uchar>(p4.y, p4.x) / 255.0;

    if (p.x < 0 || p.x >= crop_img_.cols || p.y < 0 || p.y >= crop_img_.rows) {
        throw std::out_of_range("Point out of image range");
    }

    if (p1.x < 0 || p1.y < 0 || p2.x >= crop_img_.cols || p2.y < 0
        || p3.x >= crop_img_.cols || p3.y >= crop_img_.rows || p4.x < 0
        || p4.y >= crop_img_.rows) {
        throw std::out_of_range("Interpolation points out of image range");
    }

    float x1 = std::ceil(p.x) - p.x;
    float y1 = std::ceil(p.y) - p.y;

    // X direction
    float vp_x_0 = vp1 * x1 + vp2 * (1 - x1);
    float vp_x_1 = vp3 * (1 - x1) + vp4 * x1;

    // Y direction
    float vp_y_0 = vp_x_0 * y1 + vp_x_1 * (1 - y1);
    return vp_y_0;
}

/*interpolate within the Rois along the interpolation direction*/
void core::interpolate_ROI(std::vector<cv::Point2f> Roi, cv::Mat &Roi_interpolated_map) {
    // cv::Mat Roi_points;

    for (int y = 0; y < ROI_height_; y++) {
        for (int x = 0; x < ROI_width_; x++) {
            // roi points matrix's y direction is opposite
            cv::Point2f delta_x = (Roi[1] - Roi[0]) / ROI_width_;
            cv::Point2f delta_y = (Roi[3] - Roi[0]) / ROI_height_;

            cv::Point2f p = Roi[0] + (x + 0.5) * delta_x + (y + 0.5) * delta_y;
            // Roi_points.push_back(p);
            Roi_interpolated_map.at<float>(y, x) = this->interpolate(p);
        }
    }
    // std::cout << Roi_interpolated_map.size() << std::endl;
    cv::imwrite("Roi_interpolated_map.png", Roi_interpolated_map * 255);
}

// return the index of the edge on height(y-axi) direction
int core::find_edge(std::vector<cv::Point2f> Roi) {
    // interpolate the roi rigion along the sampled direction
    // create a matrix of 2D points within the bounding box
    cv::Mat Roi_interpolated_map(ROI_height_, ROI_width_, CV_32F);
    this->interpolate_ROI(Roi, Roi_interpolated_map);

    // project along width direction
    cv::Mat projected_map; //[1xHeight]
    // TODO::experiemnt on different reduce method/flags
    cv::reduce(Roi_interpolated_map, projected_map, 1, cv::REDUCE_AVG, CV_32F);

    // first order derivative
    cv::Mat derivative_map;
    cv::Sobel(projected_map, derivative_map, CV_32F, 0, 1, 3);

    // find the peak(egde)
    double peak_value;
    double max_value;
    double min_value;
    cv::Point peak_point;
    cv::Point max_point;
    cv::Point min_point;
    cv::minMaxLoc(derivative_map, &min_value, &max_value, &min_point, &max_point);
    if (std::abs(max_value) > std::abs(min_value)) {
        peak_point = max_point;
    } else {
        peak_point = min_point;
    }
    peak_value = std::max(std::abs(max_value), std::abs(min_value));
    // std::cout << peak_point << " with value " << peak_value << std::endl;

    // draw the cross sign
    this->draw_cross_sign(Roi_interpolated_map,
                          cv::Point(Roi_interpolated_map.cols / 2, peak_point.y));
    // debug
    cv::imwrite("Roi_edge.png", Roi_interpolated_map * 255);

    if (peak_value < 1e-1) { // filter out the empty ROI
        return ROI_height_ + 1;
    } else {
        return peak_point.y;
    }
}

void core::draw_cross_sign(cv::Mat &img, cv::Point2f p) {
    // Draw the first line
    cv::line(img, cv::Point(p.x - 20, p.y), cv::Point(p.x + 20, p.y),
             cv::Scalar(0, 0, 255), 2);

    // Draw the second line
    cv::line(img, cv::Point(p.x, p.y - 20), cv::Point(p.x, p.y + 20),
             cv::Scalar(0, 0, 255), 2);
}

void core::ransac_line_filter(const std::vector<cv::Point2f> &points,
                              std::vector<cv::Point2f> &outlierPoints,
                              std::vector<cv::Point2f> &inlierPoints, double sigma) {
    const int numPoints = points.size();
    if (numPoints < 2) { return; }

    cv::RNG random;
    int maxInliers = 0;
    std::vector<cv::Point2f> temp_inlierPoints;
    std::vector<cv::Point2f> temp_outlierPoints;
    int max_iterations = log(1 - 0.99) / (log(1 - (1.00 / numPoints))) * 10;

    for (int iteration = 0; iteration < max_iterations; ++iteration) {
        int i1 = random(numPoints);
        int i2 = random(numPoints);
        while (i1 == i2) { i2 = random(numPoints); }

        const cv::Point2f &p1 = points[i1];
        const cv::Point2f &p2 = points[i2];
        cv::Point2f vectorP21 = p2 - p1;
        const double vectorNorm = cv::norm(vectorP21);
        vectorP21 *= 1.0 / vectorNorm; // unity

        int inliers = 0;
        temp_outlierPoints.clear();
        temp_inlierPoints.clear();
        for (int i = 0; i < numPoints; ++i) {
            const cv::Point2f &point = points[i];
            cv::Point2f vectorPi1 = point - p1;
            const double distance =
                std::abs(vectorPi1.y * vectorP21.x - vectorPi1.x * vectorP21.y);
            if (distance < sigma) {
                ++inliers;
                // inliers += exp(-0.5*d*d/(sigma*sigma));//误差定义方式的一种
                temp_inlierPoints.emplace_back(point);
            } else {
                temp_outlierPoints.emplace_back(point);
            }
        }

        if (inliers > maxInliers) {
            maxInliers = inliers;
            outlierPoints = temp_outlierPoints;
            inlierPoints = temp_inlierPoints;
        }
    }
}

void core::Process() {
    // generate ROIs
    this->gen_ROIs();

    // find the edge in the ROIs
    std::vector<cv::Point2f> edge_points;
    for (const auto &Roi : RoIs_) {
        int edge_y = find_edge(Roi);
        if (edge_y > ROI_height_) { continue; }
        cv::Point2f delta_width = (ROI_width_ * 0.5f) * dir_;
        cv::Point2f delta_height = edge_y * -orthDir_;
        cv::Point2f roi_edge = Roi[0] + delta_height + delta_width;
        draw_cross_sign(crop_img_, roi_edge);
        edge_points.push_back(roi_edge);
    }

    // filter out the noise
    std::vector<cv::Point2f> inlier_points;
    std::vector<cv::Point2f> outlier_points;
    ransac_line_filter(edge_points, outlier_points, inlier_points, 2.0);
    std::cout << "There are " << edge_points.size() << " edge points" << std::endl;
    std::cout << "There are " << inlier_points.size() << " inlier points" << std::endl;
    std::cout << "There are " << outlier_points.size() << " outlier points" << std::endl;

    // fit the line
    cv::fitLine(inlier_points, line_, cv::DIST_L2, 0, 0.01, 0.01);
    this->write_roi_bboxes();
}
