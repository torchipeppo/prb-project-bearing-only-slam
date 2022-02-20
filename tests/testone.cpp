#include "../slam/solver.hpp"
#include "../slam/triangulation.hpp"
#include "../utils/g2o_utils.hpp"
#include "../utils/draw_utils.hpp"

#define KEY_ESC 27
#define KEY_PGDN 65366
#define KEY_SHIFT 65506
#define KEY_TAB 9

#define CLEAR_COLOR cv::Vec3b(240,240,240)

using namespace proj02;

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "give initial_guess ground_truth please" << std::endl;
        return 1;
    }

    RGBImage img;
    img.create(800, 800);
    img=CLEAR_COLOR;

    // get the initial guess of the state from the initial guess file...
    State state(300, 200);
    BearingObservationVector _;
    _.reserve(1800);
    int fixed_pose_id;
    float bound1 = 0;
    parse_g2o(argv[1], state, _, fixed_pose_id, bound1);

    // ...but get the *noiseless* observations from the groundtruth file.
    // let's try easy mode first.
    State __(300, 200);
    BearingObservationVector observations;
    observations.reserve(1800);
    int ___;
    float bound2 = 0;
    parse_g2o(argv[2], __, observations, ___, bound2);

    // keep the larger bound
    float bound = (bound1 > bound2) ? bound1 : bound2;

    // default the fixed pose id if it wasn't provided
    if (fixed_pose_id < 0) {
        fixed_pose_id = state.default_pose_id();
    }

    // initial guess for the landmarks
    triangulate_landmarks(state, observations);

    // initialize the solver
    Solver solver(state, observations, fixed_pose_id);

    // show initial situation
    draw_bearings(img, observations, solver.state, bound);
    solver.state.draw(img, bound);
    cv::imshow("we're all trapped in a maze of landmarks", img);

    std::cout << "Any key other than the specified ones: advance one iteration" << std::endl;
    std::cout << "Tab/PgDn/Shift: advance many iterations" << std::endl;
    std::cout << "Esc: close" << std::endl;

    int counter = 0;
    while (true) {
        int key = cv::waitKey(0);
        if (key == KEY_ESC) {
            break;
        }
        else if (key == KEY_TAB ||key == KEY_PGDN || key == KEY_SHIFT) {
            std::cout << "Occhio che ci metto un po'" << std::endl;
            for (int i=0; i<50; i++) {
                solver.step();
                ++counter;
            }
        }
        else {
            solver.step();
            ++counter;
        }

        img=CLEAR_COLOR;
        // just a simple visual indicator that iterations are passing, it has NO meaning as a progressbar or anything
        cv::rectangle(img, cv::Point(img.rows-11, 0), cv::Point(img.rows-1, (counter*10)%img.cols), cv::Scalar(255, 0, 200), -1);
        draw_bearings(img, observations, solver.state, bound);
        solver.state.draw(img, bound);
        cv::imshow("we're all trapped in a maze of landmarks", img);
    }

    return 0;
}
