#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../framework/definitions.hpp"

namespace proj02 {

void draw_pose(RGBImage& img, const float& x, const float& y, const float& theta);
void draw_pose(RGBImage& img, const EPose& ep);
void draw_pose(RGBImage& img, const NEPose& nep);

void draw_lm(RGBImage& img, const float& x, const float& y);
void draw_lm(RGBImage& img, const LMPos& lm);

void draw_bearing(RGBImage& img, const float& x, const float& y, const float& theta, const float& alpha);
void draw_bearing(RGBImage& img, const EPose& src, const float& alpha);
void draw_bearing(RGBImage& img, const NEPose& src, const float& alpha);

void draw_poses(RGBImage& img, const NEPoseVector& poses);

void draw_landmarks(RGBImage& img, const LMPosVector lms);

}  // namespace proj02
