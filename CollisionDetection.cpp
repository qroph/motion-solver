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

#include <QStringList>
#include <QTextStream>

#include "CollisionDetection.h"
#include "Configuration.h"
#include "Object.h"

#include "coldet/src/coldet.h"


/*****************************************************************************/
/* CollisionDetection                                                        */
/*****************************************************************************/

CollisionDetection::CollisionDetection():
    m_solver(NULL),
    m_robot(NULL),
    m_workspaceMinX(0),
    m_workspaceMinY(0),
    m_workspaceMinZ(0),
    m_workspaceMaxX(0),
    m_workspaceMaxY(0),
    m_workspaceMaxZ(0)
{
}

CollisionDetection::~CollisionDetection()
{
}

void CollisionDetection::initialize(const Solver* solver)
{
    m_solver = solver;
}

void CollisionDetection::print(std::ostream& out)
{
    out << "CollisionDetection" << std::endl;
}



/*****************************************************************************/
/* CollisionDetection_Rigid3D                                                */
/*****************************************************************************/

CollisionDetection_Rigid3D::CollisionDetection_Rigid3D():
    CollisionDetection(),
    m_robotModel(NULL)
{
}

CollisionDetection_Rigid3D::~CollisionDetection_Rigid3D()
{
    delete m_robotModel;
    m_robotModel = NULL;

    for (size_t i = 0; i < m_obstacleModels.size(); ++i)
    {
        delete m_obstacleModels[i];
        m_obstacleModels[i] = NULL;
    }
}

void CollisionDetection_Rigid3D::loadModel(const QString& filename,
                                           CollisionModel3D* model)
{
    QFile file(filename);
    if (file.open(QFile::ReadOnly))
    {
        QTextStream inStream(&file);

        bool ok1;
        bool ok2;
        bool ok3;

        std::vector<Position_Rigid3D> vertices;
        std::vector<unsigned int> indices;

        while (!inStream.atEnd())
        {
            QStringList list = inStream
                               .readLine()
                               .split(" ", QString::SkipEmptyParts);

            if (list.empty())
            {
                continue;
            }

            if (list[0] == "v")
            {
                Q_ASSERT(list.size() == 4);

                Position_Rigid3D pos;
                pos[0] = list[1].toFloat(&ok1);
                pos[1] = list[2].toFloat(&ok2);
                pos[2] = list[3].toFloat(&ok3);
                Q_ASSERT(ok1 && ok2 && ok3);

                vertices.push_back(pos);
            }
            else if (list[0] == "f")
            {
                Q_ASSERT(list.size() == 4);

                for (int i = 0; i < 3; ++i)
                {
                    QStringList indexList = list[i + 1].split("/");
                    Q_ASSERT(indexList.size() == 3);

                    // -1 because the first index in the obj files is 1
                    float index = indexList[0].toUInt(&ok1) - 1;
                    Q_ASSERT(ok1);

                    indices.push_back(index);
                }
            }
        }

        Q_ASSERT(vertices.size() > 0);
        Q_ASSERT(indices.size() > 0);

        int faceCount = indices.size() / 3;
        for (int i = 0; i < faceCount; ++i)
        {
            float p1[3], p2[3], p3[3];
            p1[0] = vertices[indices[i * 3 + 0]][0];
            p1[1] = vertices[indices[i * 3 + 0]][1];
            p1[2] = vertices[indices[i * 3 + 0]][2];
            p2[0] = vertices[indices[i * 3 + 1]][0];
            p2[1] = vertices[indices[i * 3 + 1]][1];
            p2[2] = vertices[indices[i * 3 + 1]][2];
            p3[0] = vertices[indices[i * 3 + 2]][0];
            p3[1] = vertices[indices[i * 3 + 2]][1];
            p3[2] = vertices[indices[i * 3 + 2]][2];
            model->addTriangle(p1, p2, p3);
        }
        model->finalize();
    }
    else
    {
        FATALERROR("Unable to open a file");
    }

    file.close();
}

void CollisionDetection_Rigid3D::addRobot(Object* robot)
{
    m_robotModel = newCollisionModel3D();
    loadModel(robot->getFilename(), m_robotModel);

    m_robot = robot;
}

void CollisionDetection_Rigid3D::addObstacle(Object* obstacle)
{
    CollisionModel3D* obstacleModel = newCollisionModel3D(true);
    loadModel(obstacle->getFilename(), obstacleModel);

    m_obstacleModels.push_back(obstacleModel);
}


bool CollisionDetection_Rigid3D::isCollision() const
{
    // Robot rotation
    Configuration_Rigid3D* conf =
            static_cast<Configuration_Rigid3D*>(m_robot->getConfiguration());

    // Create a transformation matrix for the robot
    const float w = conf->rotation.w();
    const float x = conf->rotation.x();
    const float y = conf->rotation.y();
    const float z = conf->rotation.z();
    const float tx = conf->position[0];
    const float ty = conf->position[1];
    const float tz = conf->position[2];
    const float matrix[] = {
        w*w+x*x-y*y-z*z,    2*w*z+2*x*y,        2*x*z-2*w* y,       0,
        2*x*y-2*w*z,        w*w-x*x+y*y-z*z,    2*w*x+2*y*z,        0,
        2*w*y+2*x*z,        2*y*z-2*w*x,        w*w-x*x-y*y+z*z,    0,
        tx,                 ty,                 tz,                 1 };

    m_robotModel->setTransform(matrix);

    // Collision result
    bool collision = false;

    // Check collisions with obstacles
    const int count = m_obstacleModels.size();
    for (int i = 0; i < count; ++i)
    {
        collision = m_robotModel->collision(m_obstacleModels[i]);
        if (collision)
        {
            break;
        }
    }

    return collision;
}

void CollisionDetection_Rigid3D::print(std::ostream& out)
{
    out << "CollisionDetection_Rigid3D" << std::endl;
}
