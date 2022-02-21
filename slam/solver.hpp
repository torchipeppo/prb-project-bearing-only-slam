/**
 * Defines an object to perform least squares on the SLAM problem.
 * 
 * TODO get this thing a .cpp file
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

    Solver(const State& state, const BearingObservationVector& bear_obs, const OdometryObservationVector& odom_obs, const int& fixed_pose_id) :
        state(state),
        bearing_observations(bear_obs),
        odometry_observations(odom_obs),
        fixed_pose_id(fixed_pose_id)
    {
        N = 3*state.number_of_poses() + 2*state.number_of_landmarks();
        H.resize(N, N);
        b.resize(N);
        // TODO might be able to reserve the H right away
        analyzed_H = false;
        construct_the_permutation();
        kernel_threshold = 1.0f;
        damping_factor = 0.01f;
    };

    // configure some optional parameters, if the default is not good
    void set_kernel_threshold(float kt) {
        kernel_threshold = kt;
    }
    void set_damping_factor(float df) {
        damping_factor = df;
    }

    void step() {
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

    void error_and_jacobian(const State& state, const BearingObservation& obs, float& error, SparseMatrixXf& jacobian) {
        int pose_id = obs.get_pose_id();
        int lm_id = obs.get_lm_id();
        const NEPose& pose = state.get_pose_by_id(pose_id);
        const LMPos& lm = state.get_landmark_by_id(lm_id);

        float prediction = predict_bearing(pose, lm);     // h(X)

        // chosen boxminus is: normalized difference of angles (in R)
        error = normalized_angle(prediction - obs.get_bearing().smallestAngle());

        // Now for the Jacobian
        // Error function is linear, so we can derivate h(X) rather than the error
        // (I'm dropping the i and j indices for brevity, but in theory all X_p are actually X_p_i and all X_l are X_l_j)
        // h(X) = atan2(g(X))
        // g(X) = X_p^-1 * X_l
        // g(X [+] Dx) = X_p^-1 * v2t(Dx_p)^-1 * (X_l + Dx_l) =
        //             = X_p^-1 * DR^T * (X_l + Dx_l - Dt)
        //             = R^T * (DR^T * (X_l + Dx_l - Dt) - t)
        // (note that the t of the original pose X_p is always an additive constant, so it won't be relevant for the derivatives, only the R will be)
        // ([+] is boxplus, D is Delta, ^T is transpose, ^-1 is inverse)

        // first, we do the jac. of atan2(g) wrt g (computed in Dx=0)
        Eigen::Vector2f g_of_X_in_Dx_eq_0 = pose.inverse() * lm;
        const float& gx = g_of_X_in_Dx_eq_0(0);
        const float& gy = g_of_X_in_Dx_eq_0(1);
        const float& atan2_factor = 1 / (gx*gx + gy*gy);
        // Matrix1_2f jac_of_atan2_wrt_g;
        // jac_of_atan2_wrt_g(0) = atan2_factor * (-gy);
        // jac_of_atan2_wrt_g(1) = atan2_factor * gx;
        // ...this is what I WOULD LIKE to do, but since at the very end
        // we'll be doing a matrix product between this and a sparse matrix,
        // eigen just works better if both matrices to be multiplicated are sparse.
        // this matrix has just two elements, so I believe I won't be wasting too much memory.
        SparseMatrixXf jac_of_atan2_wrt_g;
        jac_of_atan2_wrt_g.resize(1,2);
        std::vector<Triplet_f> triplets;
        triplets.reserve(10);     // right now we only need 2, but we'll need 10 later
        triplets.emplace_back(0, 0, atan2_factor * (-gy));
        triplets.emplace_back(0, 1, atan2_factor * gx);
        jac_of_atan2_wrt_g.setFromTriplets(triplets.begin(), triplets.end());

        Eigen::Matrix2f R_T = pose.rotation().transpose();
        Eigen::Matrix2f DR_prime_T;   // (computed in Dx = 0)
        DR_prime_T(0,0) = 0;
        DR_prime_T(1,0) = -1;
        DR_prime_T(0,1) = 1;
        DR_prime_T(1,1) = 0;
        // then, the blocks of the jacobian of g(X [+] Dx) wrt Dx
        // they are: jac. wrt Dx_p (further subdivideable in: wrt Dt and wrt Dtheta) and jac. wrt Dx_l
        Eigen::Matrix2f jac_of_g_wrt_Dt = -R_T;        // long answer: R^T * DR^T * (-I), but when Dx = 0 then DR is also I.
        Eigen::Vector2f jac_of_g_wrt_Dtheta = R_T * DR_prime_T * lm;    // long answer: that, but final term is (X_l+Dx_l-Dt), which becomes just X_l when Dx=0
        Matrix2_3f jac_of_g_wrt_Dxp;
        jac_of_g_wrt_Dxp.block<2,2>(0,0) = jac_of_g_wrt_Dt;
        jac_of_g_wrt_Dxp.block<2,1>(0,2) = jac_of_g_wrt_Dtheta;
        Eigen::Matrix2f jac_of_g_wrt_Dxl = R_T;        // long answer: R^T * DR_T * I, but... (see above)

        // build the *sparse* jacobian of g wrt Dx from the blocks
        SparseMatrixXf jac_of_g_wrt_Dx;
        jac_of_g_wrt_Dx.resize(2, N);
        // fiddling with the indices of the jacobian (jacox = JAcobian COlumn indeX)
        const int pose_start_jacox = 3*state.pose_stix(pose_id);
        const int lm_start_jacox = 3*state.number_of_poses() + 2*state.landmark_stix(lm_id);
        // back to the blocks
        // jac_of_g_wrt_Dx.block<2,3>(0, pose_start_jacox) = jac_of_g_wrt_Dxp;
        // jac_of_g_wrt_Dx.block<2,2>(0, lm_start_jacox) = jac_of_g_wrt_Dxl;
        // this is the idea, but initializing a sparse matrix is a little more complicated
        triplets.clear();
        // triplets.reserve(10);  // total of 10 nonzero elements for this jac.   // moved above
        for (int i=0; i<2; i++) {
            // for each row of the jacobian, fill its elements
            // first those from the "pose" block
            for (int j=0; j<3; j++) {
                triplets.push_back(Triplet_f(i, pose_start_jacox+j, jac_of_g_wrt_Dxp(i,j)));
            }
            // then those from the "landmark" block
            for (int j=0; j<2; j++) {
                triplets.push_back(Triplet_f(i, lm_start_jacox+j, jac_of_g_wrt_Dxl(i,j)));
            }
        }
        jac_of_g_wrt_Dx.setFromTriplets(triplets.begin(), triplets.end());

        // FINALLY the *real* jacobian: that of h(X [+] Dx) wrt Dx
        jacobian = jac_of_atan2_wrt_g * jac_of_g_wrt_Dx;

        // done at last. error and jacobian are "returned" by reference.
    }

    void error_and_jacobian(const State& state, const OdometryObservation& obs, EPose& error, SparseMatrixXf& jacobian) {
        int src_id = obs.get_source_id();
        int dst_id = obs.get_dest_id();
        const NEPose& src_pose = state.get_pose_by_id(src_id);
        const NEPose& dst_pose = state.get_pose_by_id(dst_id);

        EPose prediction = predict_odometry(src_pose, dst_pose);     // h(X)

        // chosen boxminus is: Euclidean minus with angle normalization
        error = prediction - obs.get_transformation();
        error.z() = normalized_angle(error.z());

        // Now for the Jacobian
        // Error function is linear, so we can derivate h(X) rather than the error.
        // (R_s and t_s are the components of src_pose. Similarly dor R_d, t_d and dst_pose.)
        // (R_s and R_t are rotation matrices, theta_s and theta_d the corresponding angles. Angle normalizations are omitted.)
        // h(X) = [ R_s^T * (t_d - t_s) ]
        //        [  theta_d - theta_s  ]                 (that's how I'll represent a multiple-row/block expression)
        // Remembering that
        // X [+] Dx = [ DR*R , DR*t + Dt ]
        //            [   0  ,     1     ]                (comma separates columns)
        // Then
        // h(X [+] Dx) = [ (DR_s*R_s)^T * (DR_d*t_d + Dt_d - DR_s*t_s - Dt_s) ]
        //               [       theta_d + Dtheta_d - theta_s - Dtheta_s      ]
        //
        // (note the top block is == (DR_s*R_s)^T * (DR_d*t_d + Dt_d - Dt_s) - R_s^T * t_s, this will be relevant (only) when deriving w.r.t. Dtheta_s)
        // (fortunately there's none of the nested function madness that was in the bearing jacobian)

        // caching stuff, we'll need it
        Eigen::Matrix2f R_s = src_pose.rotation();
        Eigen::Vector2f t_d = dst_pose.translation();
        Eigen::Matrix2f DR_prime;   // (computed in Dx = 0)
        DR_prime(0,0) = 0;
        DR_prime(1,0) = 1;
        DR_prime(0,1) = -1;
        DR_prime(1,1) = 0;

        // build all the blocks
        Matrix3_2f jac_wrt_Dt_s;
        jac_wrt_Dt_s.setZero();    // this takes care of the bottom row
        jac_wrt_Dt_s.block<2,2>(0,0) = -R_s.transpose();     // there'd also be DR_s, but it becomes I when Dx==0
        Matrix3_1f jac_wrt_Dtheta_s;
        jac_wrt_Dtheta_s.head<2>() = (DR_prime * R_s).transpose() * t_d;   // see alternative top block, derivate that, then set Dx=0
        jac_wrt_Dtheta_s(2) = -1;
        Matrix3_2f jac_wrt_Dt_d;
        jac_wrt_Dt_d.setZero();    // this takes care of the bottom row
        jac_wrt_Dt_d.block<2,2>(0,0) = R_s.transpose();     // there'd also be DR_s, but it becomes I when Dx==0
        Matrix3_1f jac_wrt_Dtheta_d;
        jac_wrt_Dtheta_d.head<2>() = R_s.transpose() * DR_prime * t_d;
        jac_wrt_Dtheta_d(2) = 1;

        // prepare the triplets for the sparse jacobian
        std::vector<Triplet_f> triplets;
        triplets.reserve(18);
        // fiddling with the indices of the jacobian (jacox = JAcobian COlumn indeX)
        const int src_start_jacox = 3*state.pose_stix(src_id);
        const int dst_start_jacox = 3*state.pose_stix(dst_id);
        // back to the blocks
        for (int i=0; i<3; i++) {
            triplets.emplace_back(i, src_start_jacox, jac_wrt_Dt_s(i,0));
            triplets.emplace_back(i, src_start_jacox+1, jac_wrt_Dt_s(i,1));
            triplets.emplace_back(i, src_start_jacox+2, jac_wrt_Dtheta_s(i));
            triplets.emplace_back(i, dst_start_jacox, jac_wrt_Dt_d(i,0));
            triplets.emplace_back(i, dst_start_jacox+1, jac_wrt_Dt_d(i,1));
            triplets.emplace_back(i, dst_start_jacox+2, jac_wrt_Dtheta_d(i));
        }

        // finally set the jacobian
        jacobian.setFromTriplets(triplets.begin(), triplets.end());

        // done at last. error and jacobian are "returned" by reference.
    }

    void error_and_numerical_jacobian(const State& state, const BearingObservation& obs, float& error, SparseMatrixXf& jacobian) {
        int pose_id = obs.get_pose_id();
        int lm_id = obs.get_lm_id();
        const NEPose& pose = state.get_pose_by_id(pose_id);
        const LMPos& lm = state.get_landmark_by_id(lm_id);

        float prediction = predict_bearing(pose, lm);     // h(X)

        // chosen boxminus is: normalized difference of angles (in R)
        error = normalized_angle(prediction - obs.get_bearing().smallestAngle());

        // Now for the Jacobian. some preparations first...
        // Sparse matrices are initialized from triplets
        std::vector<Triplet_f> triplets;
        triplets.reserve(5);

        float epsilon = 0.001;

        auto error_of_X_boxplus_Dx = [this, &pose, &lm, &obs](EPose delta_pose, LMPos delta_lm) -> float {
            float prediction = predict_bearing(boxplus(pose, delta_pose), lm + delta_lm);     // h(X [+] Dx)
            return normalized_angle(prediction - obs.get_bearing().smallestAngle());
        };

        // pass arguments that have exactly one "1" among them and the rest is all zero (so as to select the specific var. to derivate wrt.)
        auto numerical_derivative = [epsilon, error_of_X_boxplus_Dx](EPose pose_selector, LMPos lm_selector) -> float {
            EPose delta_pose = epsilon*pose_selector;
            LMPos delta_lm = epsilon*lm_selector;
            float err_plus = error_of_X_boxplus_Dx(delta_pose, delta_lm);
            float err_minus = error_of_X_boxplus_Dx(-delta_pose, -delta_lm);
            return (err_plus-err_minus) / (2*epsilon);
        };

        // fiddling with the indices of the jacobian (jacox = JAcobian COlumn indeX)
        const int pose_start_jacox = 3*state.pose_stix(pose_id);
        const int lm_start_jacox = 3*state.number_of_poses() + 2*state.landmark_stix(lm_id);

        // ready for the numerical jacobian
        // first "column" (i.e. element, b/c the error is scalar so the jac. is a row)
        triplets.push_back(Triplet_f(0, pose_start_jacox, numerical_derivative(EPose(1, 0, 0), LMPos(0, 0))));
        // second
        triplets.push_back(Triplet_f(0, pose_start_jacox+1, numerical_derivative(EPose(0, 1, 0), LMPos(0, 0))));
        // third
        triplets.push_back(Triplet_f(0, pose_start_jacox+2, numerical_derivative(EPose(0, 0, 1), LMPos(0, 0))));
        // fourth (now for the lm)
        triplets.push_back(Triplet_f(0, lm_start_jacox, numerical_derivative(EPose(0, 0, 0), LMPos(1, 0))));
        // fifth
        triplets.push_back(Triplet_f(0, lm_start_jacox+1, numerical_derivative(EPose(0, 0, 0), LMPos(0, 1))));

        // construct jac. from those triplets
        jacobian.setFromTriplets(triplets.begin(), triplets.end());

        // done at last. error and jacobian are "returned" by reference.
    }

    void error_and_numerical_jacobian(const State& state, const OdometryObservation& obs, EPose& error, SparseMatrixXf& jacobian) {
        int src_id = obs.get_source_id();
        int dst_id = obs.get_dest_id();
        const NEPose& src_pose = state.get_pose_by_id(src_id);
        const NEPose& dst_pose = state.get_pose_by_id(dst_id);

        EPose prediction = predict_odometry(src_pose, dst_pose);     // h(X)

        // chosen boxminus is: Euclidean minus with angle normalization
        error = prediction - obs.get_transformation();
        error.z() = normalized_angle(error.z());

        // Now for the Jacobian. some preparations first...
        // Sparse matrices are initialized from triplets
        std::vector<Triplet_f> triplets;
        triplets.reserve(18);

        float epsilon = 0.001;

        auto error_of_X_boxplus_Dx = [this, &src_pose, &dst_pose, &obs](EPose delta_src_pose, EPose delta_dst_pose) -> EPose {
            EPose prediction = predict_odometry(boxplus(src_pose, delta_src_pose), boxplus(dst_pose, delta_dst_pose));     // h(X [+] Dx)
            EPose err_bp = prediction - obs.get_transformation();
            err_bp.z() = normalized_angle(err_bp.z());
            return err_bp;
        };

        // pass arguments that have exactly one "1" among them and the rest is all zero (so as to select the specific var. to derivate wrt.)
        auto numerical_derivative = [epsilon, error_of_X_boxplus_Dx](EPose src_selector, EPose dst_selector) -> EPose {
            EPose delta_src = epsilon*src_selector;
            EPose delta_dst = epsilon*dst_selector;
            EPose err_plus = error_of_X_boxplus_Dx(delta_src, delta_dst);
            EPose err_minus = error_of_X_boxplus_Dx(-delta_src, -delta_dst);
            return (err_plus-err_minus) / (2*epsilon);
        };

        // fiddling with the indices of the jacobian (jacox = JAcobian COlumn indeX)
        const int src_start_jacox = 3*state.pose_stix(src_id);
        const int dst_start_jacox = 3*state.pose_stix(dst_id);

        // ready for the numerical jacobian
        // first column
        EPose nd = numerical_derivative(EPose(1, 0, 0), EPose(0, 0, 0));
        triplets.push_back(Triplet_f(0, src_start_jacox, nd(0)));
        triplets.push_back(Triplet_f(1, src_start_jacox, nd(1)));
        triplets.push_back(Triplet_f(2, src_start_jacox, nd(2)));
        // second
        nd = numerical_derivative(EPose(0, 1, 0), EPose(0, 0, 0));
        triplets.push_back(Triplet_f(0, src_start_jacox+1, nd(0)));
        triplets.push_back(Triplet_f(1, src_start_jacox+1, nd(1)));
        triplets.push_back(Triplet_f(2, src_start_jacox+1, nd(2)));
        // third
        nd = numerical_derivative(EPose(0, 0, 1), EPose(0, 0, 0));
        triplets.push_back(Triplet_f(0, src_start_jacox+2, nd(0)));
        triplets.push_back(Triplet_f(1, src_start_jacox+2, nd(1)));
        triplets.push_back(Triplet_f(2, src_start_jacox+2, nd(2)));
        // fourth (now for the dst)
        nd = numerical_derivative(EPose(0, 0, 0), EPose(1, 0, 0));
        triplets.push_back(Triplet_f(0, dst_start_jacox, nd(0)));
        triplets.push_back(Triplet_f(1, dst_start_jacox, nd(1)));
        triplets.push_back(Triplet_f(2, dst_start_jacox, nd(2)));
        // fifth
        nd = numerical_derivative(EPose(0, 0, 0), EPose(0, 1, 0));
        triplets.push_back(Triplet_f(0, dst_start_jacox+1, nd(0)));
        triplets.push_back(Triplet_f(1, dst_start_jacox+1, nd(1)));
        triplets.push_back(Triplet_f(2, dst_start_jacox+1, nd(2)));
        // sixth
        nd = numerical_derivative(EPose(0, 0, 0), EPose(0, 0, 1));
        triplets.push_back(Triplet_f(0, dst_start_jacox+2, nd(0)));
        triplets.push_back(Triplet_f(1, dst_start_jacox+2, nd(1)));
        triplets.push_back(Triplet_f(2, dst_start_jacox+2, nd(2)));

        // construct jac. from those triplets
        jacobian.setFromTriplets(triplets.begin(), triplets.end());

        // done at last. error and jacobian are "returned" by reference.
    }

    float predict_bearing(const NEPose& pose, const LMPos& lm) {
        Eigen::Vector2f robot_to_lm_wrt_robot = pose.inverse() * lm;
        float bearing_angle = atan2f(robot_to_lm_wrt_robot.y(), robot_to_lm_wrt_robot.x());
        return bearing_angle;
    }

    EPose predict_odometry(const NEPose& src, const NEPose& dst) {
        // want to get an odometry <t, theta> such that
        //   dst.t = src.t + rot(src.theta) * odom.t
        //   dst.theta = src.theta + odom.theta
        //   (i.e. odom.t is in the frame of src)
        // this is NOT an homogeneous transformation handled by v2t/t2v
        // this is what matches with the test plot_g2o_v2 and the G2O doc
        EPose esrc = t2v(src);
        EPose edst = t2v(dst);
        EPose pred;
        // the translation part is the vector from dst to src, in the frame of src
        Eigen::Vector2f t = edst.head<2>() - esrc.head<2>();
        pred.head<2>() = src.rotation().transpose() * t;
        // the rotation part is just the angle difference
        pred.z() = normalized_angle(edst.z() - esrc.z());
        return pred;
    }

    float normalized_angle(float angle) {
        while (angle < -CV_PI) {
            angle += CV_2PI;
        }
        while (angle >= CV_PI) {
            angle -= CV_2PI;
        }
        return angle;
    }

    private:

    void construct_the_permutation() {
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
