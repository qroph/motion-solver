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

#include "Roadmap.h"
#include "Object.h"
#include "Distance.h"


/*****************************************************************************/
/* RoadmapNode                                                               */
/*****************************************************************************/

RoadmapNode::RoadmapNode(const RMID id, const Configuration* const config):
    DisjointSetsNode(),
    id(id),
    configuration(config),
    compareElement(0.0),
    flagLSH(false),
    m_status(ROADMAPNODE_STATUS_ACTIVE),
    previousElement(0)
{
}

int RoadmapNode::getDegree() const
{
    return m_edges.size();
}

bool RoadmapNode::isIsolated() const
{
    return m_edges.empty();
}

bool RoadmapNode::isLeaf() const
{
    return m_edges.size() == 1;
}

bool RoadmapNode::isActive() const
{
    return m_status == ROADMAPNODE_STATUS_ACTIVE;
}

bool RoadmapNode::isRemoved() const
{
    return m_status == ROADMAPNODE_STATUS_REMOVED;
}



/*****************************************************************************/
/* Roadmap                                                                   */
/*****************************************************************************/

Roadmap::Roadmap():
    m_initNode(-1),
    m_goalNode(-1),
    m_activeEdgeCount(0),
    m_removedEdgeCount(0),
    m_generatedNodeCount(0),
    m_removedNodeCount(0),
    m_removedSets(0)
{
}

Roadmap::~Roadmap()
{
    // Delete all nodes and configurations
    for (auto iter = m_nodes.begin(); iter != m_nodes.end(); ++iter)
    {
        if (*iter)
        {
            delete (*iter)->configuration;
            (*iter)->configuration = 0;

            delete *iter;
            *iter = 0;
        }
    }
}

RMID Roadmap::addInitConfiguration(const Configuration* const config)
{
    m_initNode = addConfiguration(config);
    return m_initNode;
}

RMID Roadmap::addGoalConfiguration(const Configuration* const config)
{
    m_goalNode = addConfiguration(config);
    return m_goalNode;
}

RMID Roadmap::addConfiguration(const Configuration* const config)
{
    RoadmapNode* node = new RoadmapNode(m_nodes.size(), config);
    node->m_status = RoadmapNode::ROADMAPNODE_STATUS_ACTIVE;

    m_nodes.push_back(node);
    m_sets.makeSet(*node);

    ++m_generatedNodeCount;

    return node->id;
}

void Roadmap::addEdge(const RMID node1, const RMID node2, const double length)
{
    ++m_activeEdgeCount;

    RoadmapNode::Edge edge;
    edge.length = length;

    m_nodes[node1]->m_edges.insert(
                std::pair<RMID, RoadmapNode::Edge>(node2, edge));
    m_nodes[node2]->m_edges.insert(
                std::pair<RMID, RoadmapNode::Edge>(node1, edge));

    m_sets.uniteSets(*m_nodes[node1], *m_nodes[node2]);
}

bool Roadmap::hasEdge(const RMID node1, const RMID node2) const
{
    return m_nodes[node1]->m_edges.find(node2) !=
           m_nodes[node1]->m_edges.end();
}

bool Roadmap::isConnected() const
{
    return m_sets.sameComponent(*m_nodes[m_initNode],
                                *m_nodes[m_goalNode]);
}

bool Roadmap::isConnected(const RMID node1, const RMID node2) const
{
    return m_sets.sameComponent(*m_nodes[node1], *m_nodes[node2]);
}

bool Roadmap::removeNode(const RMID node)
{
    m_nodes[node]->m_status = RoadmapNode::ROADMAPNODE_STATUS_REMOVED;
    ++m_removedNodeCount;

    if (m_nodes[node]->m_edges.size() == 1)
    {
        RMID otherNode = m_nodes[node]->m_edges.begin()->first;
        m_nodes[otherNode]->m_edges.erase(node);

        --m_activeEdgeCount;
        ++m_removedEdgeCount;
    }
    else if (m_nodes[node]->m_edges.size() == 0)
    {
        // This was the only node in the set...
        ++m_removedSets;
    }
    else
    {
        // TODO
        FATALERROR("Unable to remove a node that has more than one edge");
    }

    delete m_nodes[node]->configuration;
    m_nodes[node]->configuration = NULL;

    delete m_nodes[node];
    m_nodes[node] = NULL;

    // If this was the lastly added node, we can remove it completely
    if (node == static_cast<RMID>(m_nodes.size()) - 1)
    {
        m_nodes.pop_back();
    }

    return true;
}

bool Roadmap::getPath(RoadmapNodePath& path,
                      double& length,
                      RMID initNode,
                      RMID goalNode,
                      double maxLength) const
{
    if (initNode == -1)
    {
        initNode = m_initNode;
    }
    if (goalNode == -1)
    {
        goalNode = m_goalNode;
    }

    if (!isConnected(initNode, goalNode))
    {
        return false;
    }

    RoadmapNode::PriorityQueue queue;

    // Initialize comapreElements
    const int totalNodeCount = m_nodes.size();
    for (RMID id = 0; id < totalNodeCount; ++id)
    {
        m_nodes[id]->compareElement = -1;
    }

    // Add the initial node
    m_nodes[initNode]->previousElement = -1;
    m_nodes[initNode]->compareElement = 0.0;
    m_nodes[initNode]->queueIterator = queue.push(m_nodes[initNode]);

    // Add the goal node
    m_nodes[goalNode]->previousElement = -1;
    m_nodes[goalNode]->compareElement = DBL_MAX;
    m_nodes[goalNode]->queueIterator = queue.push(m_nodes[goalNode]);

    while (!queue.empty())
    {
        // Get the node from the queue
        RoadmapNode* nodeU = queue.top();

        // Stop if this is a goal node
        if (nodeU->id == goalNode)
        {
            break;
        }

        // Remove from the queue and mark as visited
        queue.pop();
        nodeU->queueIterator = queue.end();

        const double distU = nodeU->compareElement;

        // Go through all edges
        for (auto iter = nodeU->m_edges.cbegin();
             iter != nodeU->m_edges.cend();
             ++iter)
        {
            RoadmapNode* nodeV = m_nodes[(*iter).first];

            if (nodeV->compareElement < 0.0)
            {
                // The node is not yet in the queue
                if (nodeV->m_edges.size() > 1)
                {
                    const double alt =
                            distU + (*iter).second.length; // distU + distUtoV

                    if (maxLength != DBL_MAX)
                    {
                        const double distVtoGoal =
                                solver->distance->calculateDistance(
                                    nodeV->configuration,
                                    m_nodes[goalNode]->configuration);
                        if (alt + distVtoGoal > maxLength) // Max length
                        {
                            continue;
                        }
                    }

                    // Add the node if it has more than one edge
                    nodeV->previousElement = nodeU->id;
                    nodeV->compareElement = alt;
                    nodeV->queueIterator = queue.push(nodeV);
                }
                else
                {
                    // This node can be ignored
                    nodeV->compareElement = DBL_MAX;
                    nodeV->queueIterator = queue.end();
                }
            }
            else
            {
                // Skip if this node has already been visited
                if (nodeV->queueIterator == queue.end())
                {
                    continue;
                }

                // Check the distance and update if necessary
                const double alt =
                        distU + (*iter).second.length; // distU + distUtoV

                if (maxLength != DBL_MAX)
                {
                    const double distVtoGoal =
                            solver->distance->calculateDistance(
                                nodeV->configuration,
                                m_nodes[goalNode]->configuration);
                    if (alt + distVtoGoal > maxLength) // Max length
                    {
                        continue;
                    }
                }

                if (alt < nodeV->compareElement) // alt < distV
                {
                    nodeV->previousElement = nodeU->id;
                    nodeV->compareElement = alt;
                    queue.modify(nodeV->queueIterator, nodeV);
                }
            }
        }
    }

    // Return false if the path was not found
    if (m_nodes[goalNode]->previousElement == -1)
    {
        return false;
    }

    // Retrieve the path from the initial node to the goal node
    RMID i = goalNode;
    while (m_nodes[i]->previousElement != -1)
    {
        path.push_front(m_nodes[i]);
        i = m_nodes[i]->previousElement;
    }
    path.push_front(m_nodes[initNode]);

    length = m_nodes[goalNode]->compareElement;

    return true;
}

int Roadmap::getCurrentNodeCount() const
{
    return m_nodes.size();
}

int Roadmap::getGeneratedNodeCount() const
{
    return m_generatedNodeCount;
}

int Roadmap::getGeneratedEdgeCount() const
{
    return m_activeEdgeCount + m_removedEdgeCount;
}

int Roadmap::getActiveEdgeCount() const
{
    return m_activeEdgeCount;
}

int Roadmap::getRemovedEdgeCount() const
{
    return m_removedEdgeCount;
}

int Roadmap::getActiveNodeCount() const
{
    return m_generatedNodeCount - m_removedNodeCount;
}

int Roadmap::getRemovedNodeCount() const
{
    return m_removedNodeCount;
}

const RoadmapNode* Roadmap::getNode(RMID id) const
{
    Q_ASSERT((unsigned int)id < m_nodes.size());

    return m_nodes[id];
}

const std::vector<RMID> Roadmap::getEdges(RMID id) const
{
    Q_ASSERT((unsigned int)id < m_nodes.size());

    std::vector<RMID> edges;

    for (auto iter = m_nodes[id]->m_edges.begin();
         iter != m_nodes[id]->m_edges.end(); ++iter)
    {
        edges.push_back(iter->first);
    }

    return edges;
}

int Roadmap::getComponentCount() const
{
    return m_sets.getSetCount() - m_removedSets;
}
