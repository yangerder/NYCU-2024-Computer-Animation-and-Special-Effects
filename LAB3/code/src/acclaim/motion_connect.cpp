#include "acclaim/motion_connect.h"
#include "acclaim/maths.h"
#include <iostream>
using namespace acclaim;

void weightNormalization(std::vector<float>& weight) {
    float sum = 0.0;
    for (int i = 0; i < weight.size(); i++) sum += weight[i];

    for (int i = 0; i < weight.size(); i++) weight[i] /= sum;
}

// connect two motions, m1 = m1 + m2
// Two connection modes are supported,
//		1. insert frames by interpolation
//		2. blend overlapped frames by ease in/out
//
// numInterFrames:	number of frames inserted between two motions, only used in interpolation mode
// methodMask:		mask used to specify the matching method, operation mode, and connecting approach.
//		DISTANCE_CHECK		generate distance warning message if the bit is set
//		MATCH_FORWARD		keep m1 intact, trim m2
//		MATCH_BACKWARD		keep m2 intact, trim m1
//		INTERPOLATION		connecting by interpolation
//		BLENDING			connecting by ease in/out blending
//
// Note:
//		1. the length of blendWeight[] and frameWeight[] should be the SAME ODD number


bool acclaim::connect(Skeleton& actor, Motion& m1, Motion& m2, int numInterFrames, int methodMask) { // set maximum searching range; maximum number of frames would be cut in m2
    int numSrhFrames = 50;
    std::vector<float> jntWeight(m1.getSkeleton()->getBoneNum());
    // set default length of window to 5
    // the length of blendWeight and frameWeight should be the SAME ODD number
    std::vector<float> frameWeight(5), blendWeight(5);

    // set default joint weights
    for (int i = 0; i < jntWeight.size(); i++) {
        if (actor.getJointDOF(i) != 0)
            jntWeight[i] = 1.0;
        else
            jntWeight[i] = 0.0;  // ignore dummy bones
    }
    jntWeight[LowerBack] = 30.0;
    jntWeight[UpperBack] = 20.0;
    jntWeight[Chest] = 20.0;
    jntWeight[LeftShoulder] = 10.0;
    jntWeight[LeftElbow] = 10.0;
    jntWeight[RightShoulder] = 10.0;
    jntWeight[RightElbow] = 10.0;
    jntWeight[LeftHip] = 30.0;
    jntWeight[LeftKnee] = 10.0;
    jntWeight[RightHip] = 30.0;
    jntWeight[RightKnee] = 10.0;
    weightNormalization(jntWeight);

    // set default frame and blending weights
    float n = frameWeight.size();
    for (int i = 0; i < frameWeight.size(); i++) {
        frameWeight[i] = 1.0;

        // ease in/out blending weight
        blendWeight[i] = (1.0 + sin(i / (n - 1) * M_PI - M_PI / 2)) / 2.0;
    }
    weightNormalization(frameWeight);

    if (methodMask & BLENDING)
        return blending(actor, m1, m2, blendWeight, frameWeight, jntWeight, numSrhFrames, methodMask);

    // ErrHandler.setErrorno(UNKNOWN_METHOD);
    return false;
}

bool matching(Motion& m1, Motion& m2, const std::vector<float>& frameWeight, const std::vector<float>& jntWeight,
              int numSrhFrames, int methodMask, int& closestFrame1, float& dist1, int& closestFrame2, float& dist2) {
    // foreward search: pick a window of frames in the first motion and
    //					find the best matched window in the second motion
    Motion target1(m1, m1.getFrameNum() - frameWeight.size(), m1.getFrameNum());
    closestFrame1 = m2.winDistMatching(target1, 0, numSrhFrames, frameWeight, jntWeight, dist1);
    std::cout << "forward search result: closest frame " << closestFrame1 << " distance " << dist1 << std::endl;

    // backward search: pick a window of frames in the second motion and
    //					find the best matched window in the first motion
    Motion target2(m2, 0, frameWeight.size());
    closestFrame2 =
        m1.winDistMatching(target2, m1.getFrameNum() - numSrhFrames, m1.getFrameNum(), frameWeight, jntWeight, dist2);
    std::cout << "backward search: closest frame " << closestFrame2 << " distance " << dist2 << std::endl;

    if (methodMask & DISTANCE_CHECK) {
        if (dist1 > DIST_THRESHOLD && dist2 > DIST_THRESHOLD) return false;
        if ((methodMask & MATCH_FORWARD) && dist1 > DIST_THRESHOLD) return false;
        if ((methodMask & MATCH_BACKWARD) && dist2 > DIST_THRESHOLD) return false;
    }

    return true;
}


// blend two motion of same length
// result = (1-weight)*m1 + weight*m2
Motion blend(Motion& m1, Motion& m2, const std::vector<float>& weight) {
    // Check if two skeletons are the same.
    // This is not an exact check, but a simply way to do it!
    // A more sphosticated method might be needed.
    assert(m1.getNumBones() == m2.getNumBones());
    assert(m1.getNumFrames() == m2.getNumFrames());

    Motion result(m1.m_NumFrames, m1.getNumBones());

    for (int f = 0; f < result.getFrameNum(); f++) {
        // do blending in quternion space
        QuaterPosture qp1 = Posture2QuaterPosture(m1.getPosture(f));
        QuaterPosture qp2 = Posture2QuaterPosture(m2.getPosture(f));
        QuaterPosture interPose = SlerpInterpolate(weight[f], qp1, qp2);

        result.setPosture(f, QuaterPosture2Posture(interPose));
    }

    return result;
}


bool acclaim::blending(Skeleton& actor, Motion& m1, Motion& m2, const std::vector<float>& blendWeight,
                       const std::vector<float>& frameWeight, const std::vector<float>& jntWeight, int numSrhFrames,
                       int methodMask) {
    if (blendWeight.size() != frameWeight.size()) {
        // ErrHandler.setErrorno(BAD_BLEND_WIN_LEN);
        return false;
    }

    int closestFrame1, closestFrame2;
    float dist1, dist2;
    bool flag =
        matching(m1, m2, frameWeight, jntWeight, numSrhFrames, methodMask, closestFrame1, dist1, closestFrame2, dist2);
    if (flag == false) {
        // ErrHandler.setErrorno(MATCHING_FAIL);
        return false;
    }

    int n = blendWeight.size();
    int half_span = (n - 1) / 2;

    if (methodMask & MATCH_FORWARD) {
        // transform m2's root position and orientation before blending
        Posture lastPose = m1.getPosture(m1.m_NumFrames - n);
        float faceAng = lastPose.getFacingAngle();
        m2.remove(0, closestFrame1 - half_span);
        m2.transform(faceAng, lastPose.m_RootPos);

        // do blending
        Motion bm1(m1, m1.m_NumFrames - n, m1.m_NumFrames);
        Motion bm2(m2, 0, frameWeight.size());
        Motion bm = blend(bm1, bm2, blendWeight);

        // assemble motion frames
        m1.remove(m1.m_NumFrames - n, m1.m_NumFrames);
        m2.remove(0, frameWeight.size());
        m1.concatenate(bm);
        m1.concatenate(m2);
    } else if (methodMask & MATCH_BACKWARD) {
        // transform m2's root position and orientation before blending
        Posture lastPose = m1.getPosture(closestFrame2 - half_span);
        float faceAng = lastPose.getFacingAngle();
        m2.transform(faceAng, lastPose.m_RootPos);
        m1.remove(closestFrame2 + half_span + 1, m1.m_NumFrames);

        // do blending
        Motion bm1(m1, m1.m_NumFrames - n, m1.m_NumFrames);
        Motion bm2(m2, 0, frameWeight.size());
        Motion bm = blend(bm1, bm2, blendWeight);

        // assemble motion frames
        m1.remove(m1.m_NumFrames - n, m1.m_NumFrames);
        m2.remove(0, frameWeight.size());
        m1.concatenate(bm);
        m1.concatenate(m2);
    }

    return true;
}
