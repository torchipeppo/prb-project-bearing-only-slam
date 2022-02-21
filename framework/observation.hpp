/**
 * Structure(s?) to represent single observations, and types for vectors of those.
 */

#pragma once

#include "definitions.hpp"

namespace proj02 {

// class for bearing-only Pose-Landmark observations.
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


// Assume the odometry is given on the chart of the source pose. So it's R^3, not SO(2).
// Also, it's a little different. TODO elaborate.
// Seems to coincide with my understanding of the G2O doc: https://github.com/RainerKuemmerle/g2o/wiki/File-Format-SLAM-2D#odometry-or-loop-closures
class OdometryObservation {

    public:

    OdometryObservation(const int& source_id, const int& dest_id, EPose transformation, Eigen::Matrix3f omega) :
        source_id(source_id),
        dest_id(dest_id),
        transformation(transformation),
        omega(omega) {}
    
    OdometryObservation(const int& source_id, const int& dest_id, float x, float y, float theta, Eigen::Matrix3f omega) :
        source_id(source_id),
        dest_id(dest_id),
        transformation(x,y,theta),
        omega(omega) {}
    
    int get_source_id() const;
    int get_dest_id() const;
    EPose get_transformation() const;
    Eigen::Matrix3f get_omega() const;
    SparseMatrixXf get_omega_sparse();

    private:

    int source_id;
    int dest_id;
    EPose transformation;
    Eigen::Matrix3f omega;
    // omega sparse is compted lazily on demand, only once
    // need it for compatibility of the product with the sparse jacobian
    bool omega_sparse_computed = false;
    SparseMatrixXf omega_sparse;

};


typedef std::vector<BearingObservation> BearingObservationVector;
typedef std::vector<OdometryObservation> OdometryObservationVector;

typedef std::map<int, BearingObservationVector> BearingObservationsByLandmarkId;

}   // namespace proj02
