#include "../utils/draw_utils.hpp"
#include "../framework/state.hpp"

using namespace proj02;

int main() {
    RGBImage img;
    img.create(500, 500);
    img=cv::Vec3b(240,240,240);

    State state(5, 5);

    state.add_pose(100, 300, CV_PI/4, 897);
    state.add_pose(250, 378, CV_PI, 357);
    state.add_pose(400, 128, -CV_PI/3, 205);

    state.add_landmark(200, 200, 35);
    state.add_landmark(400, 400, 35);

    state.draw(img);

    cv::imshow("pippo", img);

    char key=cv::waitKey(0);

    return 0;
}
