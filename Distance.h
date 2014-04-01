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

#ifndef __DISTANCE_H__
#define __DISTANCE_H__

#include "Common.h"
#include "Roadmap.h"

class Configuration;
class Solver;


/*****************************************************************************/
/* Distance                                                                  */
/*****************************************************************************/

class Distance
{
public:
    Distance();
    virtual ~Distance() {}

    virtual void initialize(const Solver* solver);
    virtual double calculateDistance(
            const Configuration* const config1,
            const Configuration* const config2) const = 0;
    virtual double calculateTranslationDistance(
            const Configuration* const config1,
            const Configuration* const config2) const = 0;
    virtual double calculateRotationDistance(
            const Configuration* const config1,
            const Configuration* const config2) const = 0;
    virtual void print(std::ostream& out);

protected:
    const Solver* m_solver;
    double m_translationWeight;
    double m_rotationWeight;
};



/*****************************************************************************/
/* DistanceNormal_Rigid3D                                                    */
/*****************************************************************************/

class DistanceNormal_Rigid3D: public Distance
{
public:
    DistanceNormal_Rigid3D();

    virtual double calculateDistance(
            const Configuration* const config1,
            const Configuration* const config2) const;
    virtual double calculateTranslationDistance(
            const Configuration* const config1,
            const Configuration* const config2) const;
    virtual double calculateRotationDistance(
            const Configuration* const config1,
            const Configuration* const config2) const;
    virtual void print(std::ostream& out);
};

#endif
