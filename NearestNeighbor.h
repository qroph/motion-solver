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

#ifndef __NEARESTNEIGHBOR_H__
#define __NEARESTNEIGHBOR_H__

#include <vector>
#include <ext/pb_ds/priority_queue.hpp>

#include "Common.h"
#include "Roadmap.h"

class Configuration;
class Solver;


/*****************************************************************************/
/* NearestNeighbor                                                           */
/*****************************************************************************/

class NearestNeighbor
{
public:
    NearestNeighbor();
    virtual ~NearestNeighbor();

    virtual void initialize(const Solver* solver);
    virtual void getNeighbors(std::vector<const RoadmapNode*>& neighbors,
                              const Configuration* const config) = 0;
    virtual void addRoadmapNode(const RMID node) = 0;
    virtual void init() = 0;
    virtual void print(std::ostream& out);

protected:
    typedef __gnu_pbds::priority_queue<
        const RoadmapNode*,
        RoadmapNode::RoadmapNodeCmpLR, // Largest element is the first
        __gnu_pbds::binary_heap_tag> PriorityQueue;

    const Solver* m_solver;
    bool m_sort;
};



/*****************************************************************************/
/* NearestNeighborList                                                       */
/*****************************************************************************/

class NearestNeighborList: public NearestNeighbor
{
public:
    NearestNeighborList();
    virtual ~NearestNeighborList();

    virtual void getNeighbors(std::vector<const RoadmapNode*>& neighbors,
                              const Configuration* const config);
    virtual void addRoadmapNode(const RMID node);
    virtual void init() {}
    virtual void print(std::ostream& out);
};



/*****************************************************************************/
/* NearestNeighborLSH                                                        */
/*****************************************************************************/

typedef quint64 LSHKey;

class NearestNeighborLSH: public NearestNeighbor
{
public:
    NearestNeighborLSH();
    virtual ~NearestNeighborLSH();

    virtual void initialize(const Solver* solver);
    virtual void getNeighbors(std::vector<const RoadmapNode*>& neighbors,
                              const Configuration* const config);
    virtual void addRoadmapNode(const RMID node);
    virtual void init();
    virtual void print(std::ostream& out);

private:
    LSHKey calculateHashKey(const Configuration *const config, const int L);

    int m_L;
    int m_k;

    std::vector<std::vector<Configuration*>> m_centroids;
    std::vector<std::vector<std::vector<RMID>>> m_hashTable;
};

#endif
