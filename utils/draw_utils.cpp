#include "draw_utils.hpp"

#define POSE_RADIUS 3
#define POSE_COLOR cv::Scalar(0,0,255)     // RED (thanks opencv)
#define POSE_LINE_LEN POSE_RADIUS*1.5

#define LM_RADIUS 2
#define LM_COLOR cv::Scalar(255,0,0)     // BLUE

#define BEARING_LEN 50
#define BEARING_COLOR cv::Scalar(0,255,0)     // GREEN




// start with some generic auxiliaries

void draw_point(RGBImage& img, int x, int y, int radius, const cv::Scalar& color) {
    // check bounds (x is the column index, y is the row)
    if (x<0 || x>=img.cols || y<0 || y>=img.rows) {
        return;
    }
    cv::circle(img, cv::Point(x,y), radius, color);
}

void draw_line_ray(RGBImage& img, int x, int y, int length, float orientation, const cv::Scalar& color) {
    cv::Point center(x, y);
                                                     // y axis is flipped in the image
    cv::Point endpoint(x + length*cos(orientation), y - length*sin(orientation));
    cv::line(img, center, endpoint, color);
}

void draw_square_center(RGBImage& img, int x, int y, int radius, const cv::Scalar& color) {
    cv::Point p1(x-radius, y-radius);
    cv::Point p2(x+radius, y+radius);
    cv:rectangle(img, p1, p2, color);
}



// on to the useful ones

void draw_pose(RGBImage& img, float x, float y, float theta) {
    draw_point(img, x, y, POSE_RADIUS, POSE_COLOR);
    draw_line_ray(img, x, y, POSE_LINE_LEN, theta, POSE_COLOR);
}

void draw_lm(RGBImage& img, float x, float y) {
    draw_square_center(img, x, y, LM_RADIUS, LM_COLOR);
}

void draw_bearing(RGBImage& img, float x, float y, float theta, float alpha) {
    draw_line_ray(img, x, y, BEARING_LEN, theta+alpha, BEARING_COLOR);
}
