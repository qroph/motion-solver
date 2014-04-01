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

#include "Distance.h"
#include "Common.h"
#include "Object.h"
#include "Solver.h"


/*****************************************************************************/
/* Distance                                                                  */
/*****************************************************************************/

Distance::Distance():
    m_solver(NULL),
    m_translationWeight(0),
    m_rotationWeight(0)
{
}

void Distance::initialize(const Solver* solver)
{
    m_solver = solver;

    if (!m_solver->settings->contains("Parameters/translationWeight"))
    {
        FATALERROR("Parameter 'translationWeight' is missing");
    }
    m_translationWeight =
        m_solver->settings->value("Parameters/translationWeight").toDouble();

    if (!m_solver->settings->contains("Parameters/rotationWeight"))
    {
        FATALERROR("Parameter 'rotationWeight' is missing");
    }
    m_rotationWeight =
        m_solver->settings->value("Parameters/rotationWeight").toDouble();
}

void Distance::print(std::ostream& out)
{
    out << "Distance" << std::endl;
}



/*****************************************************************************/
/* DistanceNormal_Rigid3D                                                    */
/*****************************************************************************/

DistanceNormal_Rigid3D::DistanceNormal_Rigid3D():
    Distance()
{
}

double DistanceNormal_Rigid3D::calculateDistance(
        const Configuration* const config1,
        const Configuration* const config2) const
{
    const Configuration_Rigid3D* const conf1 =
            static_cast<const Configuration_Rigid3D* const>(config1);
    const Configuration_Rigid3D* const conf2 =
            static_cast<const Configuration_Rigid3D* const>(config2);

    return m_translationWeight
            * (conf1->position - conf2->position).norm()
            + m_rotationWeight
            * conf1->rotation.angularDistance(conf2->rotation);
}

double DistanceNormal_Rigid3D::calculateTranslationDistance(
        const Configuration* const config1,
        const Configuration* const config2) const
{
    const Configuration_Rigid3D* const conf1 =
            static_cast<const Configuration_Rigid3D* const>(config1);
    const Configuration_Rigid3D* const conf2 =
            static_cast<const Configuration_Rigid3D* const>(config2);

    return (conf1->position - conf2->position).norm();
}

double DistanceNormal_Rigid3D::calculateRotationDistance(
        const Configuration* const config1,
        const Configuration* const config2) const
{
    const Configuration_Rigid3D* const conf1 =
            static_cast<const Configuration_Rigid3D* const>(config1);
    const Configuration_Rigid3D* const conf2 =
            static_cast<const Configuration_Rigid3D* const>(config2);

    return conf1->rotation.angularDistance(conf2->rotation);
}

void DistanceNormal_Rigid3D::print(std::ostream& out)
{
    out << "DistanceNormal_Rigid3D" << std::endl;
}
