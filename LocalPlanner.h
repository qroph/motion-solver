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

#ifndef __LOCALPLANNER_H__
#define __LOCALPLANNER_H__

#include <list>

#include "Common.h"

class Configuration;
class Solver;

typedef std::list<Configuration*> LocalPath;


/*****************************************************************************/
/* LocalPlanner                                                              */
/*****************************************************************************/

class LocalPlanner
{
public:
    LocalPlanner();
    virtual ~LocalPlanner() {}

    virtual void initialize(const Solver* solver);
    bool isPathFree(const Configuration* const config1,
                    const Configuration* const config2);
    void getPath(LocalPath& path,
                 const Configuration* const config1,
                 const Configuration* const config2);
    virtual void print(std::ostream& out);

protected:
    virtual bool checkPath(LocalPath* path,
                           const Configuration* const config1,
                           const Configuration* const config2) = 0;

    const Solver* m_solver;
    double m_translationStepsize;
    double m_rotationStepsize;
};



/*****************************************************************************/
/* LocalPlannerStraightLine                                                  */
/*****************************************************************************/

class LocalPlannerStraightLine: public LocalPlanner
{
public:
    LocalPlannerStraightLine();

    virtual void print(std::ostream& out);

protected:
    virtual bool checkPath(LocalPath* path,
                           const Configuration* const config1,
                           const Configuration* const config2);
};


#endif
