#include "observation.hpp"

namespace proj02 {

int BearingObservation::get_pose_id() const {
    return pose_id;
}

int BearingObservation::get_lm_id() const {
    return lm_id;
}

Rotation2f BearingObservation::get_bearing() const {
    return bearing;
}

float BearingObservation::get_omega() const {
    return omega;
}


int OdometryObservation::get_source_id() const {
    return source_id;
}
int OdometryObservation::get_dest_id() const {
    return dest_id;
}
EPose OdometryObservation::get_transformation() const {
    return transformation;
}
Eigen::Matrix3f OdometryObservation::get_omega() const {
    return omega;
}
SparseMatrixXf OdometryObservation::get_omega_sparse() {
    if (!omega_sparse_computed) {
        std::vector<Triplet_f> triplets;
        triplets.reserve(9);
        triplets.emplace_back(0, 0, omega(0,0));
        triplets.emplace_back(0, 1, omega(0,1));
        triplets.emplace_back(0, 2, omega(0,2));
        triplets.emplace_back(1, 0, omega(1,0));
        triplets.emplace_back(1, 1, omega(1,1));
        triplets.emplace_back(1, 2, omega(1,2));
        triplets.emplace_back(2, 0, omega(2,0));
        triplets.emplace_back(2, 1, omega(2,1));
        triplets.emplace_back(2, 2, omega(2,2));
        omega_sparse.resize(3,3);
        omega_sparse.setFromTriplets(triplets.begin(), triplets.end());
    }
    return omega_sparse;
}

}   // namespace proj02
