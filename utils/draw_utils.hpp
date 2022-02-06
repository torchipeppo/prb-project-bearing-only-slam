#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../framework/definitions.hpp"
#include "../framework/state.hpp"
#include "../framework/observation.hpp"

namespace proj02 {

void draw_pose(RGBImage& img, const float& x, const float& y, const float& theta, const float& bound = -1);
void draw_pose(RGBImage& img, const EPose& ep, const float& bound = -1);
void draw_pose(RGBImage& img, const NEPose& nep, const float& bound = -1);

void draw_lm(RGBImage& img, const float& x, const float& y, const float& bound = -1);
void draw_lm(RGBImage& img, const LMPos& lm, const float& bound = -1);

void draw_bearing(RGBImage& img, const float& x, const float& y, const float& theta, const float& alpha, const float& bound = -1);
void draw_bearing(RGBImage& img, const EPose& src, const float& alpha, const float& bound = -1);
void draw_bearing(RGBImage& img, const NEPose& src, const float& alpha, const float& bound = -1);
void draw_bearing(RGBImage& img, const BearingObservation& obs, const State& state, const float& bound = -1);

void draw_poses(RGBImage& img, const NEPoseVector& poses, const float& bound = -1);

void draw_landmarks(RGBImage& img, const LMPosVector& lms, const float& bound = -1);

void draw_bearings(RGBImage& img, const BearingObservationVector& obs, const State& state, const float& bound = -1);

}  // namespace proj02
