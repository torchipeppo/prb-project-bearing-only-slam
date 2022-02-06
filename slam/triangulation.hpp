#pragma once

#include "../framework/state.hpp"
#include "../framework/observation.hpp"

namespace proj02 {

void triangulate_landmarks(State& state, const BearingObservationVector& observations);

}   // namespace proj02
