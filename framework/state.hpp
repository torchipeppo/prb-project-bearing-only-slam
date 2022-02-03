/**
 * Defines a data structure to hold the state.
 */

#pragma once

#include "definitions.hpp"

namespace proj02 {

class State {

    public:

    State(int expected_states=300, int expected_landmarks=200);

    void add_pose(const NEPose& pose, const int& id);
    void add_pose(const float& x, const float& y, const float& theta, const int& id);

    void add_landmark(const LMPos& lm, const int& id);
    void add_landmark(const float& x, const float& y, const int& id);

    void draw(RGBImage& img);

    private:

    NEPoseVector poses;
    LMPosVector landmarks;

    AssociationMap pose_id_to_stix;     // where stix = "STate IndeX". Rolls off the tongue better than "sidx", and is less ambiguous.
    AssociationVec pose_stix_to_id;
    AssociationMap lm_id_to_stix;
    AssociationVec lm_stix_to_id;
};

}   // namespace proj02
