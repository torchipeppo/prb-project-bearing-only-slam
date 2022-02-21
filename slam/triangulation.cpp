#include "triangulation.hpp"

namespace proj02 {

BearingObservationsByLandmarkId subdivide_bearings_by_landmark_id(const BearingObservationVector& all_observations) {
    BearingObservationsByLandmarkId the_resulting_map;
    // just loop over all observations and put each in the correct bucket
    for (BearingObservation obs : all_observations) {
        int id = obs.get_lm_id();
        // create the bucket if it doesn't exist
        if (the_resulting_map.count(id) == 0) {
            the_resulting_map[id] = BearingObservationVector();
            the_resulting_map[id].reserve(all_observations.size() / 200);   // very rough approximation of expected entries
        }
        // add this observation
        the_resulting_map[id].push_back(obs);
    }
    return the_resulting_map;
}

LMPos triangulate_one_landmark(const State& state, const BearingObservationVector& observations) {
    // let the landmark be [x,y], a generic pose be [px, py, theta], and the corresponding bearing observation [alpha].
    // then each observation provides one equation in the form "[x,y] belongs to the line defined by point [px,py] and angle [theta+alpha]":
    // sin(theta+alpha) * (x - px) - cos(theta+alpha) * (y - py) = 0
    // sin(..) * x - cos(..) * y - sin(..)*px + cos(..)*py = 0
    // sin(..) * x - cos(..) * y = sin(..)*px - cos(..)*py
    // we can then get a (likely overdetermined) linear system by stacking one such equation for each pose:
    // A*[x,y] = b
    // and then we can have Eigen compute the best overdetermined solution.
    // (all vectors above are columns)

    int M = observations.size();
    Eigen::Matrix<float, Eigen::Dynamic, 2> A;
    Eigen::VectorXf b;
    A.resize(M, Eigen::NoChange);
    b.resize(M);

    if (M==1) {
        std::cout << "Landmark no. " << observations[0].get_lm_id() << " only has one observation.\n";
        std::cout << "  Bearing-only SLAM won't be able to locate it properly." << std::endl;
        // (as is the case with numbers 69, 112, 114 in the data I was given)
    }

    for (int i=0; i<M; i++) {
        int id = observations[i].get_pose_id();
        EPose pose = t2v(state.get_pose_by_id(id));    // t2v for convenience
        float theta = pose.z();
        float bearing = observations[i].get_bearing().smallestAngle();

        float s = sin(theta+bearing);
        float c = cos(theta+bearing);

        A.row(i) = Eigen::Vector2f(s, -c);
        b(i) = s*pose.x() - c*pose.y();
    }

    // chose the best compromise between speed, accuracy and numerical stability.
    // I expect to only compute this triangulation once and as an initial estimate, after all.
    LMPos lm = A.colPivHouseholderQr().solve(b);

    return lm;
}

// adds the triangulated lms directly to the state
void triangulate_landmarks(State& state, const BearingObservationVector& observations) {
    BearingObservationsByLandmarkId observations_by_lm = subdivide_bearings_by_landmark_id(observations);

    for (auto it = observations_by_lm.begin(); it != observations_by_lm.end(); it++) {
        int id = it->first;
        BearingObservationVector& obs_of_that_lm = it->second;
        LMPos lm = triangulate_one_landmark(state, obs_of_that_lm);
        state.add_landmark(lm, id);
    }
}

}   // namespace proj02
