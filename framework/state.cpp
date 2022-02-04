#include "state.hpp"

#include "../utils/draw_utils.hpp"

namespace proj02 {

State::State(int expected_states, int expected_landmarks) {
    poses = NEPoseVector();
    poses.reserve(expected_states);
    landmarks = LMPosVector();
    landmarks.reserve(expected_landmarks);
    pose_id_to_stix = AssociationMap();
    lm_id_to_stix = AssociationMap();
    pose_stix_to_id = AssociationVec();
    pose_stix_to_id.reserve(expected_states);
    lm_stix_to_id = AssociationVec();
    lm_stix_to_id.reserve(expected_landmarks);
}

void State::add_pose(const NEPose& pose, const int& id) {
    poses.push_back(pose);
    int stix = poses.size()-1;    // where stix stands for "STate IndeX".
    pose_id_to_stix[id] = stix;
    pose_stix_to_id.push_back(id);
}

void State::add_pose(const float& x, const float& y, const float& theta, const int& id) {
    NEPose X = v2t(EPose(x,y,theta));
    add_pose(X, id);
}

void State::add_landmark(const LMPos& lm, const int& id) {
    landmarks.push_back(lm);
    int stix = landmarks.size()-1;
    lm_id_to_stix[id] = stix;
    lm_stix_to_id.push_back(id);
}

void State::add_landmark(const float& x, const float& y, const int& id) {
    add_landmark(LMPos(x,y), id);
}

NEPose State::get_pose_by_id(const int& id) const {
    return poses[pose_id_to_stix.at(id)];
}

LMPos State::get_landmark_by_id(const int& id) const {
    return landmarks[lm_id_to_stix.at(id)];
}

void State::draw(RGBImage& img) {
    draw_poses(img, poses);
    draw_landmarks(img, landmarks);
}

}   // namespace proj02
