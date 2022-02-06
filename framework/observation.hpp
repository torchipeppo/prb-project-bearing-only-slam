/**
 * Structure(s?) to represent single observations, and types for vectors of those.
 */

#pragma once

#include "definitions.hpp"

namespace proj02 {

// class for bearing-only Pose-Landmark observations. (The naming convention is open for an OdometryObservation too, if I find it's relevant later on.)
class BearingObservation {

    public:

    BearingObservation(const int& pose_id, const int& lm_id, const Rotation2f& bearing, const float& omega=1) :
        pose_id(pose_id),
        lm_id(lm_id),
        bearing(bearing),
        omega(omega) {}
    
    BearingObservation(const int& pose_id, const int& lm_id, const float& bearing, const float& omega=1) :
        pose_id(pose_id),
        lm_id(lm_id),
        bearing(bearing),
        omega(omega) {}
    
    int get_pose_id() const;
    int get_lm_id() const;
    Rotation2f get_bearing() const;
    float get_omega() const;

    private:

    int pose_id;
    int lm_id;
    Rotation2f bearing;
    float omega;

};

typedef std::vector<BearingObservation> BearingObservationVector;

typedef std::map<int, BearingObservationVector> BearingObservationsByLandmarkId;

}   // namespace proj02
