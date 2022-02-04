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

}   // namespace proj02
