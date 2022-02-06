#include "g2o_utils.hpp"

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