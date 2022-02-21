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
    if (argc < 2) {
        std::cout << "initial_guess_fname please" << std::endl;
        return 1;
    }

    RGBImage img;
    img.create(800, 800);
    img=CLEAR_COLOR;

    // parse dataset
    State state(300, 200);
    BearingObservationVector bearing_observations;
    bearing_observations.reserve(1800);
    OdometryObservationVector odometry_observations;
    odometry_observations.reserve(300);
    int fixed_pose_id;
    float bound = 0;
    parse_g2o(argv[1], state, bearing_observations, odometry_observations, fixed_pose_id, bound);

    // default the fixed pose id if it wasn't provided
    if (fixed_pose_id < 0) {
        fixed_pose_id = state.default_pose_id();
    }

    // initial guess for the landmarks
    triangulate_landmarks(state, bearing_observations);

    // initialize the solver
    Solver solver(state, bearing_observations, odometry_observations, fixed_pose_id);

    // show initial situation
    draw_bearings(img, bearing_observations, solver.state, bound);
    draw_odometries(img, odometry_observations, solver.state, bound);
    solver.state.draw(img, bound);
    cv::imshow("pursuing my true slam", img);

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
            std::cout << "Fatto!" << std::endl;
        }
        else {
            solver.step();
            ++counter;
        }

        img=CLEAR_COLOR;
        // just a simple visual indicator that iterations are passing, it has NO meaning as a progressbar or anything
        cv::rectangle(img, cv::Point(img.rows-11, 0), cv::Point(img.rows-1, (counter*10)%img.cols), cv::Scalar(255, 0, 200), -1);
        draw_bearings(img, bearing_observations, solver.state, bound);
        draw_odometries(img, odometry_observations, solver.state, bound);
        solver.state.draw(img, bound);
        cv::imshow("pursuing my true slam", img);
    }

    return 0;
}
