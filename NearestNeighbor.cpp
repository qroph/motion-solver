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

#include "NearestNeighbor.h"
#include "Distance.h"
#include "Object.h"
#include "Roadmap.h"
#include "Solver.h"
#include "CollisionDetection.h"


/*****************************************************************************/
/* NearestNeighbor                                                           */
/*****************************************************************************/

NearestNeighbor::NearestNeighbor():
    m_solver(NULL)
{
}

NearestNeighbor::~NearestNeighbor()
{
}

void NearestNeighbor::initialize(const Solver* solver)
{
    m_solver = solver;
}

void NearestNeighbor::print(std::ostream& out)
{
    out << "NearestNeighbor" << std::endl;
}



/*****************************************************************************/
/* NearestNeighborList                                                       */
/*****************************************************************************/

NearestNeighborList::NearestNeighborList(): NearestNeighbor()
{
}

NearestNeighborList::~NearestNeighborList()
{
}

void NearestNeighborList::getNeighbors(
        std::vector<const RoadmapNode*>& neighbors,
        const Configuration* const config)
{
    const unsigned int maxK = m_solver->settings->
            value("Parameters/maxNeighbors", UINT_MAX).toUInt();
    const double maxRadius = m_solver->settings->
            value("Parameters/maxRadius", DBL_MAX).toDouble();

    PriorityQueue queue;

    // Go through all active nodes
    const int count = m_solver->roadmap->getCurrentNodeCount();

    for (int i = 0; i < count; ++i)
    {
        const RoadmapNode* node = m_solver->roadmap->getNode(i);
        if (!node->isActive())
        {
            continue;
        }

        if (node->configuration != config)
        {
            node->compareElement =
                    m_solver->distance->calculateDistance(
                        node->configuration, config);

            if (node->compareElement <= maxRadius)
            {
                if (queue.size() < maxK)
                {
                    queue.push(node);
                }
                else if (node->compareElement <
                         queue.top()->compareElement)
                {
                    queue.pop();
                    queue.push(node);
                }
            }
        }
    }

    const int neighCount = queue.size();
    neighbors.resize(neighCount);

    const bool sort = m_solver->settings->
            value("Parameters/sort", true).toBool();
    if (sort)
    {
        for (int i = neighCount - 1; i >= 0; --i)
        {
            neighbors[i] = queue.top();
            queue.pop();
        }
    }
    else
    {
        std::copy(queue.begin(), queue.end(), neighbors.begin());
    }
}

void NearestNeighborList::addRoadmapNode(const RMID /*node*/)
{
    // No need to do anything
}

void NearestNeighborList::print(std::ostream& out)
{
    out << "NearestNeighborList" << std::endl;
}



/*****************************************************************************/
/* NearestNeighborLSH                                                        */
/*****************************************************************************/

NearestNeighborLSH::NearestNeighborLSH(): NearestNeighbor(),
    m_L(0),
    m_k(0)
{
}

NearestNeighborLSH::~NearestNeighborLSH()
{
    for (int L = 0; L < m_L; ++L)
    {
        for (int k = 0; k < m_k; ++k)
        {
            delete m_centroids[L][k];
        }
    }
}

void NearestNeighborLSH::init()
{
    const double workspaceMinX =
            m_solver->settings->value("Environment/minX").toDouble();
    const double workspaceMinY =
            m_solver->settings->value("Environment/minY").toDouble();
    const double workspaceMinZ =
            m_solver->settings->value("Environment/minZ").toDouble();
    const double workspaceMaxX =
            m_solver->settings->value("Environment/maxX").toDouble();
    const double workspaceMaxY =
            m_solver->settings->value("Environment/maxY").toDouble();
    const double workspaceMaxZ =
            m_solver->settings->value("Environment/maxZ").toDouble();

    const double workspaceSizeX = workspaceMaxX - workspaceMinX;
    const double workspaceSizeY = workspaceMaxY - workspaceMinY;
    const double workspaceSizeZ = workspaceMaxZ - workspaceMinZ;

    // Initialize hash data structures
    m_hashTable.resize(m_L);
    m_centroids.resize(m_L);

    for (int L = 0; L < m_L; ++L)
    {
        m_hashTable[L].resize(m_k);
        m_centroids[L].resize(m_k);

        int halton = 1; // Random::getUniform() * 10000;

        for (int k = 0; k < m_k; ++k)
        {
            Configuration_Rigid3D* config = new Configuration_Rigid3D();

            do
            {
                config->position[0] = workspaceSizeX *
                        Random::getHalton(halton, 2);
                config->position[1] = workspaceSizeY *
                        Random::getHalton(halton, 3);
                config->position[2] = workspaceSizeZ *
                        Random::getHalton(halton, 5);

                halton += 409;

                config->setRandomRotation();

                m_solver->robot->setConfiguration(config);
            }
            while (m_solver->collisionDetection->isCollision());

            m_centroids[L][k] = config;
        }
    }
}

LSHKey NearestNeighborLSH::calculateHashKey(
        const Configuration* const config, const int L)
{
    LSHKey hashKey = 0;

    const Configuration_Rigid3D* const conf1 =
            static_cast<const Configuration_Rigid3D* const>(config);

    double dist = DBL_MAX;
    for (int k = 0; k < m_k; ++k)
    {
        const Configuration_Rigid3D* const conf2 =
                static_cast<const Configuration_Rigid3D* const>(
                    m_centroids[L][k]);

        const double d = (conf2->position - conf1->position).squaredNorm();

        if (d < dist)
        {
            dist = d;
            hashKey = k;
        }
    }

    return hashKey;
}

void NearestNeighborLSH::initialize(const Solver* solver)
{
    NearestNeighbor::initialize(solver);

    if (!m_solver->settings->contains("Parameters/L"))
    {
        FATALERROR("Parameter 'L' is missing");
    }
    m_L = m_solver->settings->value("Parameters/L", INT_MAX).toInt();

    if (!m_solver->settings->contains("Parameters/k"))
    {
        FATALERROR("Parameter 'k' is missing");
    }
    m_k = m_solver->settings->value("Parameters/k", INT_MAX).toInt();
}

void NearestNeighborLSH::getNeighbors(
        std::vector<const RoadmapNode*>& neighbors,
        const Configuration* const config)
{
    // Requires that the flag of each node is set to false

    const double maxRadius = m_solver->settings->
            value("Parameters/maxRadius", DBL_MAX).toDouble();
    const unsigned int maxK = m_solver->settings->
            value("Parameters/maxNeighbors", UINT_MAX).toUInt();

    PriorityQueue queue;

    const unsigned int activeCount = m_solver->roadmap->getActiveNodeCount();
    if (activeCount <= maxK)
    {
        const int generatedCount = m_solver->roadmap->getCurrentNodeCount();
        for (int i = 0; i < generatedCount; ++i)
        {
            const RoadmapNode* node = m_solver->roadmap->getNode(i);
            if (!node->isActive())
            {
                continue;
            }

            node->compareElement =
                    m_solver->distance->calculateDistance(
                        node->configuration, config);

            if (node->compareElement <= maxRadius)
            {
                // We can add all nodes because there will be maxK nodes
                // at maximum
                queue.push(node);
            }
        }
    }
    else
    {
        std::vector<const RoadmapNode*> usedNodes;

        for (int L = 0; L < m_L; ++L)
        {
            const LSHKey hashKey = calculateHashKey(config, L);

            const std::vector<int>& ids = m_hashTable[L][hashKey];
            const int count = ids.size();

            for (int i = 0; i < count; ++i)
            {
                const RoadmapNode* node = m_solver->roadmap->getNode(ids[i]);

                if (!node->flagLSH &&
                    node->configuration != config &&
                    node->isActive())
                {
                    node->compareElement =
                            m_solver->distance->calculateDistance(
                                node->configuration, config);

                    if (node->compareElement <= maxRadius)
                    {
                        if (queue.size() < maxK)
                        {
                            queue.push(node);
                        }
                        else if (node->compareElement <
                                 queue.top()->compareElement)
                        {
                            queue.pop();
                            queue.push(node);
                        }

                        usedNodes.push_back(node);
                        node->flagLSH = true;
                    }
                }
            }
        }

        // Set flags to zero
        for (auto it = usedNodes.cbegin(); it != usedNodes.cend(); ++it)
        {
            (*it)->flagLSH = false;
        }

        if (queue.size() < maxK)
        {
            queue.clear();

            const int generatedCount =
                    m_solver->roadmap->getCurrentNodeCount();
            for (int i = 0; i < generatedCount; ++i)
            {
                const RoadmapNode* node = m_solver->roadmap->getNode(i);
                if (!node->isActive())
                {
                    continue;
                }

                node->compareElement =
                        m_solver->distance->calculateDistance(
                            node->configuration, config);

                if (node->compareElement <= maxRadius)
                {
                    if (queue.size() < maxK)
                    {
                        queue.push(node);
                    }
                    else if (node->compareElement <
                             queue.top()->compareElement)
                    {
                        queue.pop();
                        queue.push(node);
                    }
                }
            }
        }
    }

    const int neighCount = queue.size();
    neighbors.resize(neighCount);

    const bool sort = m_solver->settings->
            value("Parameters/sort", true).toBool();
    if (sort)
    {
        for (int i = neighCount - 1; i >= 0; --i)
        {
            neighbors[i] = queue.top();
            queue.pop();
        }
    }
    else
    {
        std::copy(queue.begin(), queue.end(), neighbors.begin());
    }
}

void NearestNeighborLSH::addRoadmapNode(const RMID node)
{
    const RoadmapNode* newNode = m_solver->roadmap->getNode(node);

    for (int L = 0; L < m_L; ++L)
    {
        const LSHKey hashKey =
                calculateHashKey(newNode->configuration, L);

        m_hashTable[L][hashKey].push_back(newNode->id);
    }
}

void NearestNeighborLSH::print(std::ostream& out)
{
    out << "NearestNeighborLSH" << std::endl;
}
