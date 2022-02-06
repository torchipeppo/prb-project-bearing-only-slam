#include "../utils/draw_utils.hpp"
#include "../utils/g2o_utils.hpp"
#include "../slam/triangulation.hpp"

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
    observations.reserve(1800);
    int fixed_pose_id;
    float bound = 0;

    parse_g2o(argv[1], state, observations, fixed_pose_id, bound);

    triangulate_landmarks(state, observations);

    draw_bearings(img, observations, state, bound);
    state.draw(img, bound);

    cv::imshow("two lines connect to triangles", img);

    char key=cv::waitKey(0);

    return 0;
}
