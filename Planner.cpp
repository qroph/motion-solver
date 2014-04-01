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

#include "Planner.h"
#include "DisjointSets.h"
#include "Distance.h"
#include "Generator.h"
#include "LocalPlanner.h"
#include "Common.h"
#include "NearestNeighbor.h"
#include "Roadmap.h"
#include "Solver.h"
#include "Configuration.h"
#include "CollisionDetection.h"
#include "Object.h"


/*****************************************************************************/
/* Planner                                                                   */
/*****************************************************************************/

Planner::Planner():
    m_solver(NULL)
{
}

Planner::~Planner()
{
}

void Planner::initialize(const Solver* solver)
{
    m_solver = solver;
}

void Planner::print(std::ostream& out)
{
    out << "Planner" << std::endl;
}



/*****************************************************************************/
/* PlannerNormal                                                             */
/*****************************************************************************/

bool PlannerNormal::solve()
{
    std::vector<const RoadmapNode*> neighbors;

    while (!m_solver->roadmap->isConnected())
    {
        // Generate a random configuration
        const Configuration* cfg =
                m_solver->generator->generateFreeConfiguration();

        // Get nearest neighbors
        neighbors.clear();
        m_solver->nearestNeighbor->getNeighbors(neighbors,cfg);
        const int count = neighbors.size();

        // Add configuration to the roadmap
        const RMID id = m_solver->roadmap->addConfiguration(cfg);

        // Go through the neighbors
        for (int i = 0; i < count; ++i)
        {
            if (!m_solver->roadmap->isConnected(id, neighbors[i]->id))
            {
                if (m_solver->localPlanner
                    ->isPathFree(cfg, neighbors[i]->configuration))
                {
                    const double length = m_solver->distance->
                        calculateDistance(cfg, neighbors[i]->configuration);
                    m_solver->roadmap->addEdge(id, neighbors[i]->id, length);
                }
            }
        }

        // Remove a leaf node
        if (m_solver->roadmap->getNode(id)->isLeaf())
        {
            m_solver->roadmap->removeNode(id);
        }
        else
        {
            // If the node is ok, add it to the neighbor structure
            m_solver->nearestNeighbor->addRoadmapNode(id);
        }
    }

    return true;
}

void PlannerNormal::print(std::ostream& out)
{
    out << "PlannerNormal" << std::endl;
}
