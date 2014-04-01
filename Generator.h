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

#ifndef __GENERATOR_H__
#define __GENERATOR_H__

#include "Common.h"

class Configuration;
class Solver;


/*****************************************************************************/
/* Generator                                                                 */
/*****************************************************************************/

class Generator
{
public:
    Generator();
    virtual ~Generator() {}

    virtual void initialize(const Solver* solver);
    virtual Configuration* generateFreeConfiguration() = 0;
    virtual Configuration* generateConfiguration() = 0;
    virtual void print(std::ostream& out);

protected:
    const Solver* m_solver;
    double m_workspaceSizeX;
    double m_workspaceSizeY;
    double m_workspaceSizeZ;
    double m_workspaceMinX;
    double m_workspaceMinY;
    double m_workspaceMinZ;
    double m_workspaceMaxX;
    double m_workspaceMaxY;
    double m_workspaceMaxZ;
};



/*****************************************************************************/
/* GeneratorUniform_Rigid3D                                                  */
/*****************************************************************************/

class GeneratorUniform_Rigid3D: public Generator
{
public:
    GeneratorUniform_Rigid3D();

    virtual void initialize(const Solver* solver);
    virtual Configuration* generateFreeConfiguration();
    virtual Configuration* generateConfiguration() { return NULL; }
    virtual void print(std::ostream& out);
};

#endif
