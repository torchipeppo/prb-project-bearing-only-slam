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

int State::number_of_poses() const {
    return poses.size();
}
int State::number_of_landmarks() const {
    return landmarks.size();
}

int State::pose_stix(const int& id) const {
    return pose_id_to_stix.at(id);
}
int State::landmark_stix(const int& id) const {
    return lm_id_to_stix.at(id);
}

int State::default_pose_id() {
    return pose_stix_to_id[0];
}

void State::apply_boxplus(const Eigen::VectorXf& delta_x) {
    const int NP = poses.size();
    const int NL = landmarks.size();
    for (int i=0; i<NP; i++) {
        EPose Dx_i = delta_x.segment<3>(3*i);
        poses[i] = boxplus(poses[i], Dx_i);
    }
    for (int j=0; j<NL; j++) {
        LMPos Dx_j = delta_x.segment<2>(3*NP + 2*j);
        landmarks[j] += Dx_j;    // landmarks are Euclidean
    }
}

// been useful for debugging a bit (on the mini data)
void State::print_full_vector() {
    Eigen::VectorXf v;
    v.resize(3*poses.size() + 2*landmarks.size());
    for (int i=0; i<poses.size(); i++) {
        v.segment<3>(3*i) = t2v(poses[i]);
    }
    for (int j=0; j<landmarks.size(); j++) {
        v.segment<2>(3*poses.size() + 2*j) = landmarks[j];
    }
    std::cout << "State: " << v.transpose() << std::endl;
}

void State::draw(RGBImage& img, const float& bound) {
    #if DRAW_ONLY_POSE>=0
    draw_pose(img, poses[pose_id_to_stix[DRAW_ONLY_POSE]], bound);
    #else
    draw_poses(img, poses, bound);
    #endif

    #if DRAW_ONLY_LM>=0
    draw_lm(img, landmarks[lm_id_to_stix[DRAW_ONLY_LM]], bound);
    #else
    draw_landmarks(img, landmarks, bound);
    #endif
}

}   // namespace proj02
