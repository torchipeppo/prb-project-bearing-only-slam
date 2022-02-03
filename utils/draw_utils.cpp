#include "draw_utils.hpp"

#define POSE_RADIUS 3
#define POSE_COLOR cv::Scalar(0,0,255)     // RED (thanks opencv)
#define POSE_LINE_LEN POSE_RADIUS*1.8

#define LM_RADIUS 2
#define LM_COLOR cv::Scalar(255,0,0)     // BLUE

#define BEARING_LEN 50
#define BEARING_COLOR cv::Scalar(0,255,0)     // GREEN



namespace proj02 {

// start with some generic auxiliaries

void draw_point(RGBImage& img, const int& x, const int& y, const int& radius, const cv::Scalar& color) {
    // check bounds (x is the column index, y is the row)
    if (x<0 || x>=img.cols || y<0 || y>=img.rows) {
        return;
    }
    cv::circle(img, cv::Point(x,y), radius, color);
}

void draw_line_ray(RGBImage& img, const int& x, const int& y, const int& length, const float& orientation, const cv::Scalar& color) {
    cv::Point center(x, y);
                                                     // y axis is flipped in the image
    cv::Point endpoint(x + length*cos(orientation), y - length*sin(orientation));
    cv::line(img, center, endpoint, color);
}

void draw_square_center(RGBImage& img, const int& x, const int& y, const int& radius, const cv::Scalar& color) {
    cv::Point p1(x-radius, y-radius);
    cv::Point p2(x+radius, y+radius);
    cv:rectangle(img, p1, p2, color);
}



// on to the useful ones

void draw_pose(RGBImage& img, const float& x, const float& y, const float& theta) {
    draw_point(img, x, y, POSE_RADIUS, POSE_COLOR);
    draw_line_ray(img, x, y, POSE_LINE_LEN, theta, POSE_COLOR);
}
void draw_pose(RGBImage& img, const EPose& ep) {
    draw_pose(img, ep.x(), ep.y(), ep.z());
}
void draw_pose(RGBImage& img, const NEPose& nep) {
    draw_pose(img, t2v(nep));
}

void draw_lm(RGBImage& img, const float& x, const float& y) {
    draw_square_center(img, x, y, LM_RADIUS, LM_COLOR);
}
void draw_lm(RGBImage& img, const LMPos& lm) {
    draw_lm(img, lm.x(), lm.y());
}

void draw_bearing(RGBImage& img, const float& x, const float& y, const float& theta, const float& alpha) {
    draw_line_ray(img, x, y, BEARING_LEN, theta+alpha, BEARING_COLOR);
}
void draw_bearing(RGBImage& img, const EPose& src, const float& alpha) {
    draw_bearing(img, src.x(), src.y(), src.z(), alpha);
}
void draw_bearing(RGBImage& img, const NEPose& src, const float& alpha) {
    draw_bearing(img, t2v(src), alpha);
}



// finally, some collective ones

void draw_poses(RGBImage& img, const NEPoseVector& poses) {
    for (const NEPose& pose : poses) {
        draw_pose(img, pose);
    }
}

void draw_landmarks(RGBImage& img, const LMPosVector lms) {
    for (const LMPos& lm : lms) {
        draw_lm(img, lm);
    }
}

}  // namespace proj02
