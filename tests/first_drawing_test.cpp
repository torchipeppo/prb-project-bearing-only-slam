#include "../utils/draw_utils.hpp"

using namespace proj02;

int main() {
    RGBImage img;
    img.create(500, 500);
    img=cv::Vec3b(240,240,240);

    int x=100;
    int y=300;
    float theta=CV_PI/4;
    draw_pose(img, x, y, theta);

    int xl=200;
    int yl=200;
    draw_lm(img, xl, yl);

    draw_bearing(img, x, y, theta, CV_PI/3);
    draw_bearing(img, x, y, theta, -CV_PI/6);

    cv::imshow("pippo", img);

    char key=cv::waitKey(0);

    return 0;
}
