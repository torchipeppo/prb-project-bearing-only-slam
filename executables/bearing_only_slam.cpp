#include "../slam/solver.hpp"
#include "../slam/triangulation.hpp"
#include "../utils/g2o_utils.hpp"
#include "../utils/draw_utils.hpp"

#define KEY_ESC 27
#define KEY_PGDN 65366
#define KEY_SHIFT 65506
#define KEY_TAB 9
#define KEY_B 98
#define KEY_O 111

#define CLEAR_COLOR cv::Vec3b(240,240,240)

using namespace proj02;

void draw(
    RGBImage& img,
    Solver& solver,
    const BearingObservationVector& bearing_observations,
    const OdometryObservationVector odometry_observations,
    const int& bound,
    const int& counter,
    const bool& should_draw_bearings,
    const bool& should_draw_odometries
) {
    img=CLEAR_COLOR;
    // just a simple visual indicator that iterations are passing, it has NO meaning as a progressbar or anything
    cv::rectangle(img, cv::Point(img.rows-11, 0), cv::Point(img.rows-1, (counter*10)%img.cols), cv::Scalar(255, 0, 200), -1);
    if (should_draw_bearings) {
        draw_bearings(img, bearing_observations, solver.state, bound);
    }
    if (should_draw_odometries) {
        draw_odometries(img, odometry_observations, solver.state, bound);
    }
    solver.state.draw(img, bound);
    cv::imshow("Bearing-Only SLAM", img);
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "usage: bearing_only_slam <dataset_fname>" << std::endl;
        return 1;
    }

    RGBImage img;
    img.create(800, 800);
    img=CLEAR_COLOR;
    bool should_draw_bearings = true;
    bool should_draw_odometries = true;

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
    draw(img, solver, bearing_observations, odometry_observations, bound, 0, should_draw_bearings, should_draw_odometries);

    std::cout << "\n\n" <<
        " ______  _______ _______  ______ _____ __   _  ______      _____  __   _        __   __      _______        _______ _______ \n" <<
        " |_____] |______ |_____| |_____/   |   | \\  | |  ____ ___ |     | | \\  | |        \\_/        |______ |      |_____| |  |  | \n" <<
        " |_____] |______ |     | |    \\_ __|__ |  \\_| |_____|     |_____| |  \\_| |_____    |         ______| |_____ |     | |  |  | \n\n" << std::endl;

    std::cout << "Any key other than the specified ones: advance one iteration" << std::endl;
    std::cout << "Tab/PgDn/Shift: advance many iterations" << std::endl;
    std::cout << "B: toggle bearing observation display" << std::endl;
    std::cout << "O: toggle odometry observation display" << std::endl;
    std::cout << "Esc: close" << std::endl;

    int counter = 0;
    while (true) {
        int key = cv::waitKey(0);
        if (key == KEY_ESC) {
            break;
        }
        else if (key == KEY_TAB || key == KEY_PGDN || key == KEY_SHIFT) {
            std::cout << "Occhio che ci metto un po'" << std::endl;
            for (int i=0; i<50; i++) {
                solver.step();
                ++counter;
            }
            std::cout << "Fatto!" << std::endl;
        }
        else if (key == KEY_B) {
            should_draw_bearings = !should_draw_bearings;
        }
        else if (key == KEY_O) {
            should_draw_odometries = !should_draw_odometries;
        }
        else {
            solver.step();
            ++counter;
        }

        draw(img, solver, bearing_observations, odometry_observations, bound, counter, should_draw_bearings, should_draw_odometries);
    }

    return 0;
}
