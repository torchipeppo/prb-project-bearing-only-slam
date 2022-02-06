#include "../utils/draw_utils.hpp"
#include "../utils/g2o_utils.hpp"

using namespace proj02;

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "fname please" << std::endl;
        return 1;
    }

    RGBImage img;
    img.create(800, 800);
    img=cv::Vec3b(240,240,240);

    State state(300, 200);
    BearingObservationVector observations;
    observations.reserve(1800);   // by rough observation of the dataset, 300 poses * ~6 measurements each
    int fixed_pose_id;
    float bound = 0;

    parse_g2o(argv[1], state, observations, fixed_pose_id, bound);
    std::cout << "And the fixed pose id is " << fixed_pose_id << std::endl;

    draw_bearings(img, observations, state, bound);
    state.draw(img, bound);

    cv::imshow("odyssey", img);

    char key=cv::waitKey(0);

    return 0;
}