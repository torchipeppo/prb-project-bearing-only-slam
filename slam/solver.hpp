/**
 * Defines an object to perform least squares on the SLAM problem.
 * 
 * Note: this header's implementation is split into two files because it's huge
 */

#pragma once

#include "../framework/state.hpp"
#include "../framework/observation.hpp"
#include <Eigen/Sparse>

// tested several solvers, here I can switch between them
#define SOLVER_LDLT 0
#define SOLVER_LU 1
#define SOLVER_QR 2
#define WHICH_SOLVER SOLVER_LDLT

namespace proj02 {

class Solver {

    public:

    // much of the reason the solver is an object is so that it can hold the state
    State state;
    BearingObservationVector bearing_observations;
    OdometryObservationVector odometry_observations;

    Solver(const State& state, const BearingObservationVector& bear_obs, const OdometryObservationVector& odom_obs, const int& fixed_pose_id);

    // configure some optional parameters, if the default is not good
    void set_kernel_threshold(float kt);
    void set_damping_factor(float df);

    void step();

    void error_and_jacobian(const State& state, const BearingObservation& obs, float& error, SparseMatrixXf& jacobian);

    void error_and_jacobian(const State& state, const OdometryObservation& obs, EPose& error, SparseMatrixXf& jacobian);

    void error_and_numerical_jacobian(const State& state, const BearingObservation& obs, float& error, SparseMatrixXf& jacobian);

    void error_and_numerical_jacobian(const State& state, const OdometryObservation& obs, EPose& error, SparseMatrixXf& jacobian);

    float predict_bearing(const NEPose& pose, const LMPos& lm);

    EPose predict_odometry(const NEPose& src, const NEPose& dst);

    float normalized_angle(float angle);

    private:

    void construct_the_permutation();

    // another advantage of holding a solver object is to allocate memory for H and b only once:
    // a mere function to call would have to allocate and deallocate every time
    SparseMatrixXf H;
    Eigen::VectorXf b;
    SparseMatrixXf H_nofixed;
    Eigen::VectorXf b_nofixed;

    // optional parameters, default in constructor
    float kernel_threshold;
    float damping_factor;

    // it appears an Eigen sparse system solver can save the sparsity pattern of H
    // in order to solve many problems with the same pattern more efficiently.
    // that's exactly our case, so that's yet another benefit
    // of having a solver object of ours where to store all this.
    #if WHICH_SOLVER == SOLVER_LDLT
    Eigen::SimplicialLDLT<SparseMatrixXf> sparse_system_solver;   
    #elif WHICH_SOLVER == SOLVER_LU
    Eigen::SparseLU<SparseMatrixXf, Eigen::COLAMDOrdering<int>> sparse_system_solver;
    #elif WHICH_SOLVER == SOLVER_QR
    Eigen::SparseQR<SparseMatrixXf, Eigen::COLAMDOrdering<int>> sparse_system_solver;
    #endif
    // (the simple name "solver" is already taken by the class, it'd be ambiguous)

    // Has sparse_system_solver.analyzePattern already been called?
    // We only need to call it once if the sparsity pattern of H doesn't change.
    bool analyzed_H;

    // dimension of state chart, noticed I want this in more than one function
    int N;

    // The id of the pose to keep fixed in order to remove the intrinsic degrees of freedom of this problem
    int fixed_pose_id;
    // A permutation that brings the indices corresponding to the fixed_pose all the way to the end. Can be inverted to do the reverse.
    Eigen::PermutationMatrix<Eigen::Dynamic> permutation_fixed_pose_to_last;

};

}   // namespace proj02
