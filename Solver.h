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

#ifndef __SOLVER_H__
#define __SOLVER_H__

#include <QDir>
#include <QSettings>
#include <vector>

#include "Common.h"

class CollisionDetection;
class Distance;
class Generator;
class LocalPlanner;
class NearestNeighbor;
class Object;
class Planner;
class Roadmap;

class Solver
{
public:
    Solver();

    void initialize(const QString& filename);
    void uninitialize();
    void savePath(const RoadmapNodePath& path);
    double run();

    Generator* generator;
    Planner* planner;
    NearestNeighbor* nearestNeighbor;
    CollisionDetection* collisionDetection;
    Distance* distance;
    LocalPlanner* localPlanner;
    Roadmap* roadmap;
    Object* robot;
    std::vector<Object*> obstacles;
    QSettings* settings;
    QDir m_taskDir;

private:
    // Start and goal configurations
    Configuration* m_initConfiguration;
    Configuration* m_goalConfiguration;
};

#endif
