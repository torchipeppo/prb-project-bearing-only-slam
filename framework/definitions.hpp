#pragma once

#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Sparse>

// use to draw selectively for debug purposes. very aggressive. active if >=0.
#define DRAW_ONLY_POSE -1
#define DRAW_ONLY_LM -1

namespace proj02 {

typedef cv::Mat_<cv::Vec3b> RGBImage;

typedef Eigen::Isometry2f NEPose;    // Non-Euclidean Pose, as a 2D homogeneous matrix
typedef Eigen::Vector3f EPose;       // Euclidean Pose

typedef Eigen::Vector2f LMPos;       // Landmark Position

typedef std::vector<NEPose, Eigen::aligned_allocator<NEPose>> NEPoseVector;
typedef std::vector<LMPos, Eigen::aligned_allocator<LMPos>> LMPosVector;

typedef Eigen::Rotation2D<float> Rotation2f;

typedef std::map<int, int> AssociationMap;   // id to index is not ideal as a vector since there's no guarantee to start from zero or be continuous.
typedef std::vector<int> AssociationVec;     // index to id is fine as a vector instead, so why not reap the benefits

typedef Eigen::Matrix<float, 1, 2> Matrix1_2f;
typedef Eigen::Matrix<float, 2, 3> Matrix2_3f;

typedef Eigen::SparseMatrix<float> SparseMatrixXf;

typedef Eigen::Triplet<float> Triplet_f;

inline EPose t2v(const NEPose& nep) {
    Eigen::Vector2f t = nep.translation();
    Rotation2f r = Rotation2f(nep.rotation());
    return EPose(t.x(), t.y(), r.smallestAngle());
}

inline NEPose v2t(const EPose& ep) {
    NEPose X;
    X.setIdentity();
    Eigen::Vector2f t = Eigen::Vector2f(ep.x(), ep.y());
    X.translation() = t;
    Rotation2f r = Rotation2f(ep.z());
    X.linear() = r.matrix();    // need to use linear since X.rotation() returns a const
    return NEPose(X);
}

}  // namespace proj02
