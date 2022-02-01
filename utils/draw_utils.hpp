#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

typedef cv::Mat_< cv::Vec3b > RGBImage;

void draw_pose(RGBImage& img, float x, float y, float theta);

void draw_lm(RGBImage& img, float x, float y);

void draw_bearing(RGBImage& img, float x, float y, float theta, float alpha);
