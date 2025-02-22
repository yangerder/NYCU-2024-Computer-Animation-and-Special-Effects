#pragma once

#include <vector>
#include "Eigen/Core"

#include "motion.h"
#include "posture.h"
#include "skeleton.h"

#define MAX_NUM_EDGES 200
#define MAX_NUM_MOTIONS 100

namespace acclaim {
class MotionNode {
 public:
    MotionNode();
    void addEdgeTo(int node, double edge_weight);

    int num_edges;
    int edge[MAX_NUM_EDGES];
    double weight[MAX_NUM_EDGES];
};

class MotionGraph {
 public:
    MotionGraph(std::vector<Motion> motionList, int inputSSize, int inputBSize, int inputThres);

    void constructGraph();
    void traverse();

    std::vector<MotionNode> m_graph;
    int numNodes;
    int numMotions;

    int currIdx, nextIdx;
    Motion currSegment, nextSegment;
    

 private:
    void computeDistMatrix(int blendWindowSize);

    std::vector<Motion> segmentList;
    std::vector<int> EndSegments;
    
    std::vector<double> jointWeights;
    std::vector<double> blendWeights;
    std::vector<std::vector<double>> distMatrix;

    int numBones;
    int segmentSize, blendWindowSize;
    double edgeCostThreshold;
    // vector m_RootPosWorld, m_RootOriWorld;
    // float m_FacingWorld, m_curFacing;

    // ErrorMsgType m_ErrorType;
    // int m_endFrames[MAX_NUM_MOTIONS];

    
};
}  // namespace acclaim
