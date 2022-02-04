#include "../utils/draw_utils.hpp"
#include "../framework/state.hpp"
#include "../framework/observation.hpp"

using namespace proj02;

void add_coherent_observation(BearingObservationVector& observations, float statex, float statey, float statetheta, int stateid, float lmx, float lmy, int lmid) {
    // y axis is flipped in the image
    float absolute_angle = atan2(statey-lmy, lmx-statex);
    float relative_angle = absolute_angle - statetheta;
    observations.emplace_back(stateid, lmid, relative_angle);
}

int main() {
    RGBImage img;
    img.create(500, 500);
    img=cv::Vec3b(240,240,240);

    State state(5, 5);

    state.add_pose(100, 300, CV_PI/4, 897);
    state.add_pose(250, 378, CV_PI, 357);
    state.add_pose(400, 128, -CV_PI/3, 205);

    state.add_landmark(200, 200, 35);
    state.add_landmark(400, 400, 65);

    BearingObservationVector observations;

    add_coherent_observation(observations, 100, 300, CV_PI/4, 897, 200, 200, 35);
    add_coherent_observation(observations, 250, 378, CV_PI, 357, 200, 200, 35);
    add_coherent_observation(observations, 250, 378, CV_PI, 357, 400, 400, 65);
    add_coherent_observation(observations, 400, 128, -CV_PI/3, 205, 400, 400, 65);

    state.draw(img);
    draw_bearings(img, observations, state);

    cv::imshow("beholder", img);

    char key=cv::waitKey(0);

    return 0;
}