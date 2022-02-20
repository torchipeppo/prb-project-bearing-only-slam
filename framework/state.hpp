/**
 * Defines a data structure to hold the state.
 */

#pragma once

#include "definitions.hpp"

namespace proj02 {

inline NEPose boxplus(const NEPose& X, const EPose& delta_x) {
    return v2t(delta_x) * X;
}

class State {

    public:

    State(int expected_states=300, int expected_landmarks=200);

    void add_pose(const NEPose& pose, const int& id);
    void add_pose(const float& x, const float& y, const float& theta, const int& id);

    void add_landmark(const LMPos& lm, const int& id);
    void add_landmark(const float& x, const float& y, const int& id);

    NEPose get_pose_by_id(const int& id) const;
    LMPos get_landmark_by_id(const int& id) const;

    int number_of_poses() const;
    int number_of_landmarks() const;

    int pose_stix(const int& id) const;
    int landmark_stix(const int& id) const;

    // return the id of any pose in the state. may be useful to initialize the fixed_pose_id with an existing pose id, if not provided.
    int default_pose_id();

    // Side Effect: apply a perturbation to each pose and landmark in the state
    void apply_boxplus(const Eigen::VectorXf& delta_x);

    void print_full_vector();
    void draw(RGBImage& img, const float& bound = -1);

    private:

    NEPoseVector poses;
    LMPosVector landmarks;

    AssociationMap pose_id_to_stix;     // where stix = "STate IndeX". Rolls off the tongue better than "sidx", and is less ambiguous.
    AssociationVec pose_stix_to_id;
    AssociationMap lm_id_to_stix;
    AssociationVec lm_stix_to_id;
};

}   // namespace proj02
