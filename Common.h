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

#ifndef __COMMON_H__
#define __COMMON_H__

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Geometry>

#include <QHash>
#include <QString>
#include <QVariant>
#include <QFileInfo>

#define FATALERROR(error) \
{ \
    QFileInfo __fileInfo(__FILE__); \
    qDebug("ERROR: \"%s\", file: %s, line: %i", \
           QString(error).toUtf8().data(), \
           __fileInfo.fileName().toUtf8().data(), \
           __LINE__); \
    exit(-1); \
}


class Configuration;
class RoadmapNode;

typedef int RMID;
typedef QHash<QString, QVariant> Parameters;

typedef std::list<RoadmapNode*> RoadmapNodePath;

namespace Random
{
    void initialize();
    void setSeed(unsigned int seed);
    unsigned int getSeed();
    double getNormal();
    double getUniform();
    double getHalton(const unsigned int index, const unsigned int prime);
}

#endif
