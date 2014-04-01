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

#include "Generator.h"
#include "CollisionDetection.h"
#include "Common.h"
#include "Object.h"
#include "Solver.h"
#include "Distance.h"


/*****************************************************************************/
/* Generator                                                                 */
/*****************************************************************************/

Generator::Generator():
    m_solver(NULL),
    m_workspaceSizeX(0),
    m_workspaceSizeY(0),
    m_workspaceSizeZ(0),
    m_workspaceMinX(0),
    m_workspaceMinY(0),
    m_workspaceMinZ(0),
    m_workspaceMaxX(0),
    m_workspaceMaxY(0),
    m_workspaceMaxZ(0)
{
}

void Generator::initialize(const Solver* solver)
{
    m_solver = solver;

    if (!m_solver->settings->contains("Environment/minX"))
    {
        FATALERROR("Parameter 'minX' is missing");
    }
    if (!m_solver->settings->contains("Environment/minY"))
    {
        FATALERROR("Parameter 'minY' is missing");
    }
    if (!m_solver->settings->contains("Environment/maxX"))
    {
        FATALERROR("Parameter 'maxX' is missing");
    }
    if (!m_solver->settings->contains("Environment/maxY"))
    {
        FATALERROR("Parameter 'maxY' is missing");
    }
    if (!m_solver->settings->contains("Environment/maxZ"))
    {
        FATALERROR("Parameter 'maxZ' is missing");
    }
    if (!m_solver->settings->contains("Environment/minZ"))
    {
        FATALERROR("Parameter 'minZ' is missing");
    }

    m_workspaceMinX = m_solver->settings->value("Environment/minX").toDouble();
    m_workspaceMinY = m_solver->settings->value("Environment/minY").toDouble();
    m_workspaceMinZ = m_solver->settings->value("Environment/minZ").toDouble();
    m_workspaceMaxX = m_solver->settings->value("Environment/maxX").toDouble();
    m_workspaceMaxY = m_solver->settings->value("Environment/maxY").toDouble();
    m_workspaceMaxZ = m_solver->settings->value("Environment/maxZ").toDouble();

    m_workspaceSizeX = m_workspaceMaxX - m_workspaceMinX;
    m_workspaceSizeY = m_workspaceMaxY - m_workspaceMinY;
    m_workspaceSizeZ = m_workspaceMaxZ - m_workspaceMinZ;
}

void Generator::print(std::ostream& out)
{
    out << "Generator" << std::endl;
}



/*****************************************************************************/
/* GeneratorUniform_Rigid3D                                                  */
/*****************************************************************************/

GeneratorUniform_Rigid3D::GeneratorUniform_Rigid3D():
    Generator()
{
}

void GeneratorUniform_Rigid3D::initialize(const Solver* solver)
{
    Generator::initialize(solver);
}

Configuration* GeneratorUniform_Rigid3D::generateFreeConfiguration()
{
    Configuration_Rigid3D* cfg = new Configuration_Rigid3D;

    do
    {
        cfg->position[0] =
                Random::getUniform() * m_workspaceSizeX + m_workspaceMinX;
        cfg->position[1] =
                Random::getUniform() * m_workspaceSizeY + m_workspaceMinY;
        cfg->position[2] =
                Random::getUniform() * m_workspaceSizeZ + m_workspaceMinZ;

        cfg->setRandomRotation();

        m_solver->robot->setConfiguration(cfg);
    }
    while (m_solver->collisionDetection->isCollision());

    return cfg;
}

void GeneratorUniform_Rigid3D::print(std::ostream& out)
{
    out << "GeneratorUniform_Rigid3D" << std::endl;
}
