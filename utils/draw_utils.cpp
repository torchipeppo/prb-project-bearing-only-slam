#include "draw_utils.hpp"

#define POSE_RADIUS 4
#define POSE_COLOR cv::Scalar(0,0,255)     // RED (thanks opencv)
#define POSE_LINE_LEN POSE_RADIUS*2

#define LM_RADIUS 2
#define LM_COLOR cv::Scalar(255,0,0)     // BLUE

#define BEARING_LEN 50
#define BEARING_COLOR cv::Scalar(0,255,0)     // GREEN

#define ODOMETRY_COLOR cv::Scalar(255, 0, 250)    // PURPLE
#define ODOMETRY_LEN 4



namespace proj02 {

// start with some generic auxiliaries

void draw_point(RGBImage& img, const int& x, const int& y, const int& radius, const cv::Scalar& color) {
    // check bounds (x is the column index, y is the row)
    if (x<0 || x>=img.cols || y<0 || y>=img.rows) {
        return;
    }
    cv::circle(img, cv::Point(x,y), radius, color);
}

void draw_segment(RGBImage& img, const int& x1, const int& y1, const int& x2, const int& y2, const cv::Scalar& color) {
    cv::line(img, cv::Point(x1, y1), cv::Point(x2, y2), color);
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

float map_interval(const float& x, const float& from_min, const float& from_max, const float& to_min, const float& to_max) {
    float ratio = (x - from_min) / (from_max - from_min);
    return to_min + ratio * (to_max - to_min);
}



// on to the useful ones
// General note: If "bound" is given, we assume that x,y are in Cartesian coordinates in [-bound, bound]
// and we map them to the image's [0,rows) and [0,cols) ranges.
// Also we flip the y axis so that we accurately represent the Cartesian plane (y-up) in the image (y-down by default).
// Otherwise, if "bound" is not given (and left to the default of -1, see .h),
// then we assume that the coordinates are directly in image space and leave them fully unchanged.

void draw_pose(RGBImage& img, const float& x, const float& y, const float& theta, const float& bound) {
    float xx;
    float yy ;
    if (bound > 0) {
        xx = map_interval(x, -bound, bound, 0, img.cols-1);    // remember x goes on the column index
        yy = map_interval(y, -bound, bound, 0, img.rows-1);    // and y on the rows
        // in the image the y axis is flipped
        yy = img.rows-1 - yy;
    }
    else {
        xx = x;
        yy = y;
    }
    draw_point(img, xx, yy, POSE_RADIUS, POSE_COLOR);
    draw_line_ray(img, xx, yy, POSE_LINE_LEN, theta, POSE_COLOR);
}
void draw_pose(RGBImage& img, const EPose& ep, const float& bound) {
    draw_pose(img, ep.x(), ep.y(), ep.z(), bound);
}
void draw_pose(RGBImage& img, const NEPose& nep, const float& bound) {
    draw_pose(img, t2v(nep), bound);
}

void draw_lm(RGBImage& img, const float& x, const float& y, const float& bound) {
    float xx;
    float yy ;
    if (bound > 0) {
        xx = map_interval(x, -bound, bound, 0, img.cols-1);    // remember x goes on the column index
        yy = map_interval(y, -bound, bound, 0, img.rows-1);    // and y on the rows
        // in the image the y axis is flipped
        yy = img.rows-1 - yy;
    }
    else {
        xx = x;
        yy = y;
    }
    draw_square_center(img, xx, yy, LM_RADIUS, LM_COLOR);
}
void draw_lm(RGBImage& img, const LMPos& lm, const float& bound) {
    draw_lm(img, lm.x(), lm.y(), bound);
}

void draw_bearing(RGBImage& img, const float& x, const float& y, const float& theta, const float& alpha, const float& bound) {
    float xx;
    float yy ;
    if (bound > 0) {
        xx = map_interval(x, -bound, bound, 0, img.cols-1);    // remember x goes on the column index
        yy = map_interval(y, -bound, bound, 0, img.rows-1);    // and y on the rows
        // in the image the y axis is flipped
        yy = img.rows-1 - yy;
    }
    else {
        xx = x;
        yy = y;
    }
    draw_line_ray(img, xx, yy, BEARING_LEN, theta+alpha, BEARING_COLOR);
}
void draw_bearing(RGBImage& img, const EPose& src, const float& alpha, const float& bound) {
    draw_bearing(img, src.x(), src.y(), src.z(), alpha, bound);
}
void draw_bearing(RGBImage& img, const NEPose& src, const float& alpha, const float& bound) {
    draw_bearing(img, t2v(src), alpha, bound);
}
void draw_bearing(RGBImage& img, const BearingObservation& obs, const State& state, const float& bound) {
    NEPose src = state.get_pose_by_id(obs.get_pose_id());
    float alpha = obs.get_bearing().angle();
    draw_bearing(img, src, alpha, bound);
}

void draw_odometry(RGBImage& img, const NEPose& src, const EPose& trasf, const float& bound) {
    EPose src_eucl = t2v(src);
    float src_x = src_eucl.x();
    float src_y = src_eucl.y();
    float src_theta = src_eucl.z();
    // memento: this transformation is NOT boxplus
    Eigen::Vector2f t = trasf.head<2>();
    t = src.rotation() * t;
    float dest_x = src_x + t.x();
    float dest_y = src_y + t.y();
    float dest_theta = src_theta + trasf.z();
    if (bound > 0) {
        src_x = map_interval(src_x, -bound, bound, 0, img.cols-1);
        src_y = map_interval(src_y, -bound, bound, 0, img.cols-1);
        src_y = img.rows-1 - src_y;
        dest_x = map_interval(dest_x, -bound, bound, 0, img.cols-1);
        dest_y = map_interval(dest_y, -bound, bound, 0, img.cols-1);
        dest_y = img.rows-1 - dest_y;
    }   // else they are fine as initialized
    draw_segment(img, src_x, src_y, dest_x, dest_y, ODOMETRY_COLOR);
    draw_line_ray(img, dest_x, dest_y, ODOMETRY_LEN, dest_theta, ODOMETRY_COLOR);
}
void draw_odometry(RGBImage& img, const OdometryObservation& obs, const State& state, const float& bound) {
    NEPose src = state.get_pose_by_id(obs.get_source_id());
    EPose trasf = obs.get_transformation();
    draw_odometry(img, src, trasf, bound);
}



// finally, some collective ones

void draw_poses(RGBImage& img, const NEPoseVector& poses, const float& bound) {
    for (const NEPose& pose : poses) {
        draw_pose(img, pose, bound);
    }
}

void draw_landmarks(RGBImage& img, const LMPosVector& lms, const float& bound) {
    for (const LMPos& lm : lms) {
        draw_lm(img, lm, bound);
    }
}

void draw_bearings(RGBImage& img, const BearingObservationVector& observations, const State& state, const float& bound) {
    for (const BearingObservation& obs : observations) {
        #if DRAW_ONLY_POSE>=0
        if (obs.get_pose_id()!=DRAW_ONLY_POSE) continue;
        #endif
        #if DRAW_ONLY_LM>=0
        if (obs.get_lm_id()!=DRAW_ONLY_LM) continue;
        #endif
        draw_bearing(img, obs, state, bound);
    }
}

void draw_odometries(RGBImage& img, const OdometryObservationVector& observations, const State& state, const float& bound) {
    for (const OdometryObservation& obs : observations) {
        #if DRAW_ONLY_POSE>=0
        if (obs.get_source_id()!=DRAW_ONLY_POSE) continue;
        // std::cout << t2v(state.get_pose_by_id(obs.get_source_id())).transpose() << std::endl;
        // std::cout << obs.get_transformation().transpose() << std::endl;
        // std::cout << t2v(boxplus(state.get_pose_by_id(obs.get_source_id()), obs.get_transformation())).transpose() << std::endl;
        // std::cout << t2v(state.get_pose_by_id(obs.get_dest_id())).transpose() << std::endl;
        #endif
        draw_odometry(img, obs, state, bound);
    }
}

}  // namespace proj02
