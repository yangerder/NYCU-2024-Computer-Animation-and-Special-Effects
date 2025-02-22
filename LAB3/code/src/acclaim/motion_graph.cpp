#include "acclaim/motion_graph.h"
#include <cmath>
#include <iostream>
#include "acclaim/motion.h"
#include "acclaim/posture.h"

#define M_PI 3.1415926

namespace acclaim {

MotionNode::MotionNode() {
    num_edges = 0;
    for (int j = 0; j < MAX_NUM_EDGES; j++) weight[j] = 0.0;
}

void MotionNode::addEdgeTo(int node, double edge_weight) {
    assert(num_edges < MAX_NUM_EDGES);

    edge[num_edges] = node;
    weight[num_edges] = edge_weight;
    num_edges++;
}

MotionGraph::MotionGraph(std::vector<Motion> motionList, int inputSSize, int inputBSize, int inputThres) {
    segmentSize = inputSSize;
    blendWindowSize = inputBSize;

    edgeCostThreshold = inputThres;
    // sigma = inputSigma;

    // Cut motions into segments
    int begin, end;
    for (int i = 0; i < motionList.size(); i++) {
        int totalFrames = motionList[i].getFrameNum();
        begin = 0;
        end = segmentSize;
        while (end <= totalFrames) {
            if (totalFrames - end < segmentSize) {
                end = totalFrames;
            }
            Motion segment(motionList[i], begin, end);
            segmentList.push_back(segment);
            begin += segmentSize;
            end += segmentSize;
        }
        EndSegments.push_back(segmentList.size() - 1);
        // std::cout << "This Motion has " << totalFrames << " frames" << std::endl;
    }

    distMatrix.resize(segmentList.size());
    for (int i = 0; i < segmentList.size(); ++i) {
        distMatrix[i].resize(segmentList.size());
    }

    nextIdx = 0;
    nextSegment = segmentList[0];

    // Initialize parameters
    numNodes = segmentList.size();
    numBones = motionList[0].getSkeleton()->getBoneNum();

    std::vector<double> tempJointWeights(numBones, 0.0);
    std::vector<double> tempBlendWeights;

    tempJointWeights[2] = 50.0;
    tempJointWeights[3] = 30.0;
    tempJointWeights[4] = 15.0;
    tempJointWeights[5] = 5.0;
    tempJointWeights[7] = 50.0;
    tempJointWeights[8] = 30.0;
    tempJointWeights[9] = 15.0;
    tempJointWeights[10] = 5.0;
    tempJointWeights[11] = 50.0;
    tempJointWeights[12] = 40.0;
    tempJointWeights[13] = 30.0;
    tempJointWeights[14] = 20.0;
    tempJointWeights[15] = 15.0;
    tempJointWeights[16] = 5.0;
    tempJointWeights[17] = 30.0;
    tempJointWeights[18] = 15.0;
    tempJointWeights[19] = 5.0;
    tempJointWeights[24] = 30.0;
    tempJointWeights[25] = 15.0;
    tempJointWeights[26] = 5.0;

    for (int i = 0; i < tempJointWeights.size(); i++) tempJointWeights[i] /= 480;

    for (int i = 0; i < blendWindowSize; i++) {
        tempBlendWeights.push_back((1.0 + sin(double(i) / double(blendWindowSize - 1) * M_PI - M_PI / 2)) / 2.0);
    }

    jointWeights = tempJointWeights;
    blendWeights = tempBlendWeights;
    printf("Joint weights: ");
    for (auto w : jointWeights) {
        printf("%lf ", w);
    }
    printf("\n");
    printf("Blend weights: ");
    for (auto w : blendWeights) {
        printf("%lf\n", w);
    }
    printf("\n");
}

bool isNotInVector(const std::vector<int>& vec, int value) {
    for (auto n : vec) {
        if (value == n) return false;
    }
    return true;
}

bool isInVector(const std::vector<int>& vec, int value) {
    for (auto n : vec) {
        if (value == n) return true;
    }
    return false;
}

int getMotionSrc(const std::vector<int>& vec, int segNum) {
    int result = 0;
    while (segNum > vec[result]) {
        ++result;
    }
    return result;
}

void MotionGraph::computeDistMatrix(int blendWindowSize) {
    Posture p1, p2;
    double dist;

    for (int i = 0; i < numNodes; i++) {
        printf("Calculating costs for segment %d\n", i);
        for (int j = 0; j < numNodes; j++) {
            if (i == j) {
                distMatrix[i][j] = edgeCostThreshold;
                continue;
            }
            dist = 0;
            Motion m1 = segmentList[i];
            Motion m2 = segmentList[j];
            int m1_frames = m1.getFrameNum();
            // std::cout << "calculating cost between segment " << i << " and segment " << j << std::endl;

            for (int f = 0; f < blendWindowSize; f++) {
                p1 = m1.getPosture(m1_frames - blendWindowSize + f);
                p2 = m2.getPosture(f);
                dist += p1.PoseDist(p2, numBones, jointWeights);
            }
            distMatrix[i][j] = dist;
        }
    }
}

void MotionGraph::constructGraph() {
    computeDistMatrix(blendWindowSize);
    /*
    for (int a = 0; a < numNodes; a++) {
        for (int b = 0; b < numNodes; b++) {
            std::cout << distMatrix[a][b] << " ";
        }
        std::cout << std::endl;
    }*/

    m_graph.clear();
    for (int i = 0; i < numNodes; i++) {
        MotionNode* m = new (MotionNode);
        m_graph.push_back(*m);
    }

    for (int i = 0; i < numNodes; i++) {
        // **TODO**
        // Task: For each node in the motion graph, construct the edge using MotionNode::addEdgeTo()
        // Hint: 1. Each node in m_graph represents a motion segment from the original three motion clips.
        //
        //       2. An outgoing edge from m_graph[i] to m_graph[j] can be constructed under two circumstances:
        //              (a) j = i+1, which means these are two consecutive segments from a original motion clip
        //              (b) distMatrix[i][j] < edgeCostThreshold
        //          Circumstance (a) should NOT be applied when m_graph[i] is the final segment of a motion clip.
        //
        //       3. You can freely decide how to distribute the edge weights for each node, as long as it produces a
        //       reasonable graph.
        //          One way is to give a constant weight to the edge pointing to the node's consecutive segment (eg.
        //          0.5), and for the rest edges, the weight is distributed by the values in distMatrix[i][j], the
        //          higher the distance, the smaller the weight Make sure that the sum of weights of all outgoing edges
        //          should be 1.0 for every node.
        //
        //       4. It is okay for the nodes which represent the final segment of the motion clips to have zero outgoing
        //       edges.
        //          You can also prune some existing edges to get a better result.
        double totalWeight = 0.0;

        if (!isInVector(EndSegments, i) && i + 1 < numNodes) {
            m_graph[i].addEdgeTo(i + 1, 1000);  
            totalWeight += 1000;
        }


        for (int j = 0; j < numNodes; j++) {
            if (i != j && distMatrix[i][j] < edgeCostThreshold) {
                double weight = distMatrix[i][j];  
                m_graph[i].addEdgeTo(j, weight);
                totalWeight += weight;
            }
        }

        if (totalWeight > 0.0) {
            for (int k = 0; k < m_graph[i].num_edges; k++) {
                m_graph[i].weight[k] /= totalWeight;
            }
        }
    }
}

void MotionGraph::traverse() {
    // srand((int) &current);
    currIdx = nextIdx;
    currSegment = nextSegment;

    double prob = rand() / (double)RAND_MAX;
    double sum = 0.0;

    std::cout << "Current segment index: " << currIdx << std::endl;

    if (m_graph[currIdx].num_edges == 0) {
        nextIdx = 0;
    } else {
        for (int i = 0; i < m_graph[currIdx].num_edges; i++) {
            sum += m_graph[currIdx].weight[i];
            if (sum >= prob) {
                nextIdx = m_graph[currIdx].edge[i];
                break;
            }
        }
    }

    if (nextIdx - currIdx == 1) {
        nextSegment = segmentList[nextIdx];  // no jump
        return;
    } else {
        printf("rand: %lf, sum: %lf, jump from %d to %d\n", prob, sum, currIdx, nextIdx);
        Posture lastPose = currSegment.getPosture(currSegment.getFrameNum() - blendWindowSize);
        // double faceAng = lastPose.getFacingAngle();
        segmentList[nextIdx].transform(lastPose.bone_rotations[0], lastPose.bone_translations[0]);
        nextSegment = segmentList[nextIdx];
        Motion bm = currSegment.blending(nextSegment, blendWeights, blendWindowSize);

        for (int i = nextIdx; isNotInVector(EndSegments, i);) {
            lastPose = segmentList[i].getPosture(segmentList[i].getFrameNum() - 1);
            // faceAng = lastPose.getFacingAngle();
            segmentList[++i].transform(lastPose.bone_rotations[0], lastPose.bone_translations[0]);
        }

        Motion tempCurr(currSegment);
        Motion tempNext(nextSegment);
        tempCurr.remove(tempCurr.getFrameNum() - blendWindowSize, tempCurr.getFrameNum());
        tempCurr.concatenate(bm);
        tempNext.remove(0, blendWindowSize);

        currSegment = tempCurr;
        nextSegment = tempNext;
        return;
    }
}
}  // namespace acclaim