/*
Copyright (c) 2014 Mika Rantanen

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef __ROADMAP_H__
#define __ROADMAP_H__

#include <ext/pb_ds/priority_queue.hpp>
#include <unordered_map>
#include <vector>
#include <cfloat>

#include "Common.h"
#include "DisjointSets.h"
#include "Solver.h"

class Configuration;


/*****************************************************************************/
/* RoadmapNode                                                               */
/*****************************************************************************/

class RoadmapNode: public DisjointSetsNode
{
    friend class Roadmap;

public:
    struct RoadmapNodeCmpRL: public std::binary_function<
            RoadmapNode*,
            RoadmapNode*,
            bool>
    {
        inline bool operator()(const RoadmapNode* left,
                               const RoadmapNode* right) const
        {
            return right->compareElement < left->compareElement;
        }
    };

    struct RoadmapNodeCmpLR: public std::binary_function<
            RoadmapNode*,
            RoadmapNode*,
            bool>
    {
        inline bool operator()(const RoadmapNode* left,
                               const RoadmapNode* right) const
        {
            return left->compareElement < right->compareElement;
        }
    };

private:
    enum RoadmapNodeStatus
    {
        ROADMAPNODE_STATUS_ACTIVE = 0,
        ROADMAPNODE_STATUS_REMOVED
    };

    struct Edge
    {
        double length;
    };

    typedef __gnu_pbds::priority_queue<
                            RoadmapNode*,
                            RoadmapNodeCmpRL, // Smallest element is the first
                            __gnu_pbds::pairing_heap_tag> PriorityQueue;

    RoadmapNode(const RMID id, const Configuration* const config);

public:
    int getDegree() const;
    bool isIsolated() const;
    bool isLeaf() const;
    bool isActive() const;
    bool isRemoved() const;

    const RMID id;
    const Configuration* configuration;

    // Used by nearest neighbor search and Dijkstra's algorithm, for example
    mutable double compareElement;

    // Used by LSH algorithm
    mutable bool flagLSH;

private:
    std::unordered_map<RMID, Edge> m_edges;
    RoadmapNodeStatus m_status;

    // Used by Dijkstra's algorithm
    RMID previousElement;
    PriorityQueue::point_iterator queueIterator;
};



/*****************************************************************************/
/* Roadmap                                                                   */
/*****************************************************************************/

class Roadmap
{
public:
    Roadmap();
    ~Roadmap();

    RMID addInitConfiguration(const Configuration* const config);
    RMID addGoalConfiguration(const Configuration* const config);
    RMID addConfiguration(const Configuration* const config);
    void addEdge(const RMID node1, const RMID node2, const double length);
    bool hasEdge(const RMID node1, const RMID node2) const;
    bool isConnected() const;
    bool isConnected(const RMID node1, const RMID node2) const;
    bool removeNode(const RMID node);
    int getCurrentNodeCount() const;
    int getGeneratedNodeCount() const;
    int getGeneratedEdgeCount() const;
    int getActiveEdgeCount() const;
    int getRemovedEdgeCount() const;
    int getActiveNodeCount() const;
    int getRemovedNodeCount() const;
    const RoadmapNode* getNode(RMID id) const;
    const std::vector<RMID> getEdges(RMID id) const;
    bool getPath(RoadmapNodePath& path,
                 double& length,
                 RMID initNode = -1,
                 RMID goalNode = -1,
                 double maxLength = DBL_MAX) const;
    int getComponentCount() const;

    Solver* solver;

private:
    RMID m_initNode;
    RMID m_goalNode;
    int m_activeEdgeCount;
    int m_removedEdgeCount;
    int m_generatedNodeCount;
    int m_removedNodeCount;
    std::vector<RoadmapNode*> m_nodes;
    mutable DisjointSets m_sets;
    int m_removedSets;
};

#endif
