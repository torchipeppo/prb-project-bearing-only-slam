#include "solver.hpp"

namespace proj02 {

Solver::Solver(const State& state, const BearingObservationVector& bear_obs, const OdometryObservationVector& odom_obs, const int& fixed_pose_id) :
    state(state),
    bearing_observations(bear_obs),
    odometry_observations(odom_obs),
    fixed_pose_id(fixed_pose_id)
{
    N = 3*state.number_of_poses() + 2*state.number_of_landmarks();
    H.resize(N, N);
    b.resize(N);
    analyzed_H = false;
    construct_the_permutation();
    kernel_threshold = 1.0f;
    damping_factor = 0.01f;
};

void Solver::set_kernel_threshold(float kt) {
    kernel_threshold = kt;
}
void Solver::set_damping_factor(float df) {
    damping_factor = df;
}

void Solver::step() {
    H.setZero();
    b.setZero();

    for (const BearingObservation& obs : bearing_observations) {
        float error;
        SparseMatrixXf jacobian;
        jacobian.resize(1, N);
        error_and_jacobian(state, obs, error, jacobian);

        // simplistic threshold robust kernel
        float error_omeganorm = error * obs.get_omega() * error;
        if (error_omeganorm > kernel_threshold) {
            error *= std::sqrt(kernel_threshold/error_omeganorm);
        }

        // omega is assumed to be on chart
        H += jacobian.transpose() * obs.get_omega() * jacobian;
        b += jacobian.transpose() * obs.get_omega() * error;
    }

    for (OdometryObservation& obs : odometry_observations) {
        EPose error;
        SparseMatrixXf jacobian;
        jacobian.resize(3, N);
        error_and_jacobian(state, obs, error, jacobian);

        // simplistic threshold robust kernel
        float error_omeganorm = error.transpose() * obs.get_omega() * error;
        if (error_omeganorm > kernel_threshold) {
            error *= std::sqrt(kernel_threshold/error_omeganorm);
        }

        H += jacobian.transpose() * obs.get_omega_sparse() * jacobian;
        b += jacobian.transpose() * obs.get_omega_sparse() * error;
    }

    // add a small diagonal matrix to the H as damping, in case it's not SPD
    SparseMatrixXf diag;
    diag.resize(H.rows(), H.cols());
    diag.setIdentity();
    diag *= damping_factor;
    H += diag;

    // remove the fixed pose from H and b
    H_nofixed = (permutation_fixed_pose_to_last * H * permutation_fixed_pose_to_last.transpose()).topLeftCorner(N-3, N-3);
    b_nofixed = (permutation_fixed_pose_to_last * b).head(N-3);

    H_nofixed.makeCompressed();

    if (!analyzed_H) {
        sparse_system_solver.analyzePattern(H_nofixed);
        analyzed_H = true;
    }
    sparse_system_solver.factorize(H_nofixed);
    if(sparse_system_solver.info()!=Eigen::Success && sparse_system_solver.info()==Eigen::NumericalIssue) {
        std::cout << "The matrix appears not to be SPD this time. And after all the care I put in the damping..." << std::endl;
    }
    Eigen::VectorXf delta_x_nofixed = sparse_system_solver.solve(-b_nofixed);

    // we still have to reintroduce the fixed pose (as zero delta).
    Eigen::VectorXf delta_x;
    // define delta_x as an extended version of delta_x_nofixed...
    delta_x.resize(N);
    delta_x.head(N-3) = delta_x_nofixed;
    delta_x.tail<3>() = Eigen::Vector3f::Zero();
    // ...then apply the inverse permutation to put the zeros to the place of the fixed_pose
    delta_x = permutation_fixed_pose_to_last.inverse() * delta_x;

    state.apply_boxplus(delta_x);
}

void Solver::construct_the_permutation() {
    // this permutation will move the rows/cols of H and elems of b all the way to the end,
    // shifting the next elements to fill the void
    // (it can obviously be inverted to restore the removed variable back to its place)

    // first off, we have the id of the pose in the state, but we want to work with indices in the big Delta_x vector (dxi = Delta_x index)
    int fixed_pose_dxi = 3*state.pose_stix(fixed_pose_id);

    // now let's define that permutation
    Eigen::VectorXi indices;   // maps each `i` to `indices[i]`, as per Eigen doc
    indices.resize(N);
    // all indices before our fateful fixed_pose_dxi remain where they are
    for (int i=0; i<fixed_pose_dxi; i++) {
        indices[i] = i;
    }
    // the three components of the fixed pose are mapped last
    indices[fixed_pose_dxi] = N-3;
    indices[fixed_pose_dxi+1] = N-2;
    indices[fixed_pose_dxi+2] = N-1;
    // the remaining ones are shifted by 3 positions to the left
    for (int i=fixed_pose_dxi+3; i<N; i++) {
        indices[i] = i-3;
    }

    // finally we build the Eigen object
    permutation_fixed_pose_to_last = Eigen::PermutationMatrix<Eigen::Dynamic>(indices);
}

}   // namespace proj02
