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


// arguments are actually irrelevant, as long as *something* is given to the solver.
void predict_trust_check(const State& state, const BearingObservationVector& observations, const int& fixed_pose_id) {
    std::cout << "Predict trust check." << std::endl;
    const float PI = 3.14159265358979323846;

    Solver s(state, observations, fixed_pose_id);

    std::cout << "This should be zero: ";
    std::cout << s.predict_bearing(v2t(EPose(0,0,0)), LMPos(1,0)) << std::endl;
    std::cout << "This should be pi/2: ";
    std::cout << s.predict_bearing(v2t(EPose(0,0,0)), LMPos(0,1)) << std::endl;
    std::cout << "This should be (+/-)pi: ";
    std::cout << s.predict_bearing(v2t(EPose(0,0,0)), LMPos(-1,0)) << std::endl;
    std::cout << "This should be -pi/2: ";
    std::cout << s.predict_bearing(v2t(EPose(0,0,0)), LMPos(0,-1)) << std::endl;
    std::cout << "This should be pi/4: ";
    std::cout << s.predict_bearing(v2t(EPose(0,0,0)), LMPos(1,1)) << std::endl;
    std::cout << "This should be -pi/4: ";
    std::cout << s.predict_bearing(v2t(EPose(0,0,PI/2)), LMPos(1,1)) << std::endl;
    std::cout << "This should be (+/-)pi: ";
    std::cout << s.predict_bearing(v2t(EPose(0,0,PI)), LMPos(1,0)) << std::endl;
}


void jacobian_correctness_test(const State& state, const BearingObservationVector& observations, const int& fixed_pose_id) {
    std::cout << "Jacobian correctness test (comparison b/w analytical and numerical)." << std::endl;
    const int N = 3*state.number_of_poses() + 2*state.number_of_landmarks();
    float _;
    SparseMatrixXf jacobian_ant;
    SparseMatrixXf jacobian_num;
    jacobian_ant.resize(1,N);
    jacobian_num.resize(1,N);

    Solver s(state, observations, fixed_pose_id);

    float highest_sum = 0;
    float highest_max = 0;
    float total_sum = 0;
    float total_max = 0;

    for (const BearingObservation& obs : observations) {
        s.error_and_jacobian(state, obs, _, jacobian_ant);
        s.error_and_numerical_jacobian(state, obs, _, jacobian_num);

        SparseMatrixXf jacobian_absdiff = (jacobian_ant - jacobian_num).cwiseAbs();

        float sum = jacobian_absdiff.sum();

        jacobian_absdiff.makeCompressed();
        float max = jacobian_absdiff.coeffs().maxCoeff();

        highest_sum = std::max(highest_sum, sum);
        highest_max = std::max(highest_max, max);

        total_sum += sum;
        total_max += max;
    }

    std::cout << "highest_sum: " << highest_sum << std::endl;
    std::cout << "highest_max: " << highest_max << std::endl;
    std::cout << "average_sum: " << total_sum / observations.size() << std::endl;
    std::cout << "average_max: " << total_max / observations.size() << std::endl;

    /**
     * highest_sum: 0.0135395
     * highest_max: 0.0131645
     * average_sum: 0.000372358
     * average_max: 0.000166852
     * Which isn't outrageous, so the analytical jacobian should be okay
     */
}


int main(int argc, char** argv) {
    if (argc < 3) {
        std::cout << "give initial_guess ground_truth please" << std::endl;
        return 1;
    }

    RGBImage img;
    img.create(800, 800);
    img=CLEAR_COLOR;

    // parse both files 

    State state_ig(300, 200);
    BearingObservationVector observations_ig;
    observations_ig.reserve(1800);
    int fixed_pose_id;
    float bound1 = 0;
    parse_g2o(argv[1], state_ig, observations_ig, fixed_pose_id, bound1);

    State state_gt(300, 200);
    BearingObservationVector observations_gt;
    observations_gt.reserve(1800);
    int _;
    float bound2 = 0;
    parse_g2o(argv[2], state_gt, observations_gt, _, bound2);

    // keep the larger bound
    float bound = (bound1 > bound2) ? bound1 : bound2;

    // default the fixed pose id if it wasn't provided
    if (fixed_pose_id < 0) {
        fixed_pose_id = 0;
    }

    // initial guess for the landmarks
    triangulate_landmarks(state_ig, observations_ig);

    // Preparations done. Select a test here.
    // predict_trust_check(state_gt, observations_gt, fixed_pose_id);
    jacobian_correctness_test(state_gt, observations_gt, fixed_pose_id);

    return 0;
}
