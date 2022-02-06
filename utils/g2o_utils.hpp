#pragma once

#include <fstream>
#include <string>
#include <sstream>
#include <iostream>

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

void parse_g2o(std::string fname, State& state, BearingObservationVector& bearings, int& fixed_pose_id, float& bound) {
    bound = 0;
    fixed_pose_id = -1;

    std::ifstream f;
    f.open(fname);

    // read file by lines
    std::string line;
    while (std::getline(f, line)) {
        // line stream to tokenize by whitespace
        std::istringstream line_stream(line);
        // first check the type of the line
        std::string line_type;
        line_stream >> line_type;

        std::string token;
        // then handle each line type

        // pose
        if (line_type.compare("VERTEX_SE2") == 0) {
            line_stream >> token;
            int id = std::stoi(token);
            line_stream >> token;
            float x = std::stof(token);
            line_stream >> token;
            float y = std::stof(token);
            line_stream >> token;
            float theta = std::stof(token);

            if (std::abs(x) > bound) {
                bound = std::abs(x);
            }
            if (std::abs(y) > bound) {
                bound = std::abs(y);
            }

            state.add_pose(x, y, theta, id);
        }

        // landmark (ground truth only)
        else if (line_type.compare("VERTEX_XY") == 0) {
            line_stream >> token;
            int id = std::stoi(token);
            line_stream >> token;
            float x = std::stof(token);
            line_stream >> token;
            float y = std::stof(token);

            if (std::abs(x) > bound) {
                bound = std::abs(x);
            }
            if (std::abs(y) > bound) {
                bound = std::abs(y);
            }

            state.add_landmark(x, y, id);
        }

        // the id of the pose to fix
        else if (line_type.compare("FIX") == 0) {
            line_stream >> token;
            int id = std::stoi(token);

            // uncertainty handling needs only one pose, so we'll assume that no more than one is specified.
            fixed_pose_id = id;
        }

        // odometry edge
        else if (line_type.compare("EDGE_SE2") == 0) {
            // unsupported unless I find it to be useful later
            ;   // do nothing, so this doesn't count as unrecognized later
        }

        // bearing observation edge
        else if (line_type.compare("EDGE_BEARING_SE2_XY") == 0) {
            line_stream >> token;
            int id_pose = std::stoi(token);
            line_stream >> token;
            int id_lm = std::stoi(token);
            line_stream >> token;
            float bearing = std::stof(token);

            bearings.emplace_back(id_pose, id_lm, bearing);
        }

        else {
            std::cout << "Unrecognized " << line_type << std::endl;
        }
    }

    // some extra margin for the bound
    bound += 3;

    f.close();
}

}   // namespace proj02
