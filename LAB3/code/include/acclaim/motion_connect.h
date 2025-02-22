#pragma once

#include "Eigen/Core"
#include <vector>

#include "acclaim/posture.h"
#include "acclaim/skeleton.h"
#include "acclaim/motion.h"

namespace acclaim {
// methodMask
#define DISTANCE_CHECK 0x01
#define MATCH_FORWARD 0x02
#define MATCH_BACKWARD 0x04
#define BLENDING 0x10

const float DIST_THRESHOLD = 15.0;
// short version of connect()
bool connect(Skeleton& actor, Motion& m1, Motion& m2, int numInterFrames, int methodMask);

// generic version of connect() -- blending()
bool blending(Skeleton& actor, Motion& m1, Motion& m2, const std::vector<float>& blendWeight,
              const std::vector<float>& frameWeight, const std::vector<float>& jntWeight, int numSrhFrames,
              int methodMask);
}  // namespace acclaim
