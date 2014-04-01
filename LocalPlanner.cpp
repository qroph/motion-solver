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

#include <vector>

#include "LocalPlanner.h"
#include "CollisionDetection.h"
#include "Configuration.h"
#include "Distance.h"
#include "Object.h"
#include "Solver.h"


/*****************************************************************************/
/* LocalPlanner                                                              */
/*****************************************************************************/

LocalPlanner::LocalPlanner():
    m_solver(NULL),
    m_translationStepsize(0.0),
    m_rotationStepsize(0.0)
{
}

void LocalPlanner::initialize(const Solver* solver)
{
    m_solver = solver;

    if (!m_solver->settings->contains("Parameters/translationStepsize"))
    {
        FATALERROR("Parameter 'translationStepsize' is missing");
    }
    m_translationStepsize =
        m_solver->settings->value("Parameters/translationStepsize").toDouble();

    if (!m_solver->settings->contains("Parameters/rotationStepsize"))
    {
        FATALERROR("Parameter 'rotationStepsize' is missing");
    }
    m_rotationStepsize =
        m_solver->settings->value("Parameters/rotationStepsize").toDouble();
}

bool LocalPlanner::isPathFree(const Configuration* const config1,
                              const Configuration* const config2)
{
    return checkPath(0, config1, config2);;
}

void LocalPlanner::getPath(LocalPath& path,
                           const Configuration* const config1,
                           const Configuration* const config2)
{
    checkPath(&path, config1, config2);
}

void LocalPlanner::print(std::ostream& out)
{
    out << "LocalPlanner" << std::endl;
}



/*****************************************************************************/
/* LocalPlannerStraightLine                                                  */
/*****************************************************************************/

LocalPlannerStraightLine::LocalPlannerStraightLine():LocalPlanner()
{
}

bool LocalPlannerStraightLine::checkPath(LocalPath* path,
                                         const Configuration* const config1,
                                         const Configuration* const config2)
{
    // Calculate how many steps are needed
    int temp1 = m_solver->distance->
        calculateTranslationDistance(config1, config2) / m_translationStepsize;

    int temp2 = m_solver->distance->
        calculateRotationDistance(config1, config2) / m_rotationStepsize;

    int stepCount = qMax(temp1, temp2);

    if (path)
    {
        // When a path is created,
        // the halton points must be saved in a correct order
        std::vector<double> order;
        for (int i = 0; i <= stepCount; ++i)
        {
            order.push_back(Random::getHalton(i, 2));
        }
        std::sort(order.begin(), order.end());

        // No collision checks. Just save the path
        for (int i = 0; i <= stepCount; ++i)
        {
            Configuration* cfg = new Configuration_Rigid3D;
            config1->interpolate(config2, order[i], cfg);
            path->push_back(cfg);
        }
    }
    else
    {
        for (int i = 0; i <= stepCount; ++i)
        {
            config1->interpolate(config2,
                                 Random::getHalton(i, 2),
                                 m_solver->robot->getConfiguration());

            if (m_solver->collisionDetection->isCollision())
            {
                return false;
            }
        }
    }

    return true;
}

void LocalPlannerStraightLine::print(std::ostream& out)
{
    out << "LocalPlannerStraightLine" << std::endl;
}
