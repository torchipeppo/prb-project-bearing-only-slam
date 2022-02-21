#pragma once

#include <fstream>
#include <string>
#include <sstream>
#include <iostream>

#include "../framework/state.hpp"
#include "../framework/observation.hpp"

/**
 * THIS ASSUMES THE FOLLOWING FORMATS FOR THE LINES OF THE G2O FILE,
 * ACCORDING TO THE README AND THE REPO WIKI (https://github.com/RainerKuemmerle/g2o/wiki/File-Format-SLAM-2D):
 * 
 * VERTEX_SE2 id x y theta
 * VERTEX_XY id x y
 * FIX id
 * EDGE_SE2 i j x y theta <upper-triangular block of omega, row-major (6 numbers)>
 * EDGE_BEARING_SE2_XY id_pose id_landmark bearing <stuff to ignore, as instructed>
 * 
 * Stuff might break if the test data have any other tokens!
 */

namespace proj02 {

// OUTDATED VERSION, it only stays to make past tests work
void parse_g2o(std::string fname, State& state, BearingObservationVector& bearings, int& fixed_pose_id, float& bound);

void parse_g2o(std::string fname, State& state, BearingObservationVector& bearings, OdometryObservationVector& odometries, int& fixed_pose_id, float& bound);

}   // namespace proj02
