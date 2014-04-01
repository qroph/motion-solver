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

#include <ctime>
#include <fstream>

#include "Solver.h"
#include "CollisionDetection.h"
#include "Distance.h"
#include "Generator.h"
#include "LocalPlanner.h"
#include "NearestNeighbor.h"
#include "Object.h"
#include "Planner.h"
#include "Roadmap.h"


Solver::Solver():
    generator(NULL),
    planner(NULL),
    nearestNeighbor(NULL),
    collisionDetection(NULL),
    distance(NULL),
    localPlanner(NULL),
    roadmap(NULL),
    robot(NULL),
    settings(NULL)
{
}

void Solver::initialize(const QString& filename)
{
    const QFile settingsFile(filename);
    if (!settingsFile.exists())
    {
        FATALERROR("Task file does not exist");
    }

    m_taskDir = QDir(QFileInfo(filename).absolutePath());
    settings = new QSettings(filename, QSettings::IniFormat);

    // Robot
    if (!settings->contains("Environment/robotFilename"))
    {
        FATALERROR("Parameter 'robotFilename' is missing");
    }

    robot = new Object_Rigid3D;
    robot->setFilename(m_taskDir.absoluteFilePath(
        settings->value("Environment/robotFilename").toString()));
    robot->initializeRobot();

    // Obstacles
    if (!settings->contains("Environment/obstacleFilenames"))
    {
        FATALERROR("Parameter 'obstacleFilenames' is missing");
    }

    QStringList obstacleFilenames =
        settings->value("Environment/obstacleFilenames").toString().split(" ");

    foreach (QString obstacleFilename, obstacleFilenames)
    {
        obstacles.push_back(new Object_Rigid3D);
        obstacles.back()->setFilename(
            m_taskDir.absoluteFilePath(obstacleFilename));
        obstacles.back()->initializeObstacle();
    }

    // Initial configurations
    m_initConfiguration = new Configuration_Rigid3D;
    m_goalConfiguration = new Configuration_Rigid3D;

    QStringList start = settings->value("Environment/robotStartConfiguration")
                        .toString().split(" ");
    QStringList goal = settings->value("Environment/robotGoalConfiguration")
                       .toString().split(" ");

    if (start.count() != 7)
    {
        FATALERROR("Parameter 'robotStartConfiguration' is invalid");
    }

    if (goal.count() != 7)
    {
        FATALERROR("Parameter 'robotGoalConfiguration' is invalid");
    }

    bool ok1, ok2;
    std::vector<double> s(7);
    std::vector<double> g(7);
    for (int i = 0; i < 7; ++i)
    {
        s[i] = start.at(i).toDouble(&ok1);
        g[i] = goal.at(i).toDouble(&ok2);
        if (!ok1)
        {
            FATALERROR("Unable to parse parameter 'robotStartConfiguration'");
        }
        if (!ok2)
        {
            FATALERROR("Unable to parse parameter 'robotGoalConfiguration'");
        }
    }

    static_cast<Configuration_Rigid3D*>(m_initConfiguration)->position =
            Position_Rigid3D(s[0], s[1], s[2]);
    static_cast<Configuration_Rigid3D*>(m_initConfiguration)->rotation =
            Rotation_Rigid3D(s[3], s[4], s[5], s[6]);
    static_cast<Configuration_Rigid3D*>(m_goalConfiguration)->position =
            Position_Rigid3D(g[0], g[1], g[2]);
    static_cast<Configuration_Rigid3D*>(m_goalConfiguration)->rotation =
            Rotation_Rigid3D(g[3], g[4], g[5], g[6]);

    // Random seed
    if (settings->contains("Environment/randomSeed"))
    {
        Random::setSeed(settings->value("Environment/randomSeed").toUInt());
    }
    else
    {
        Random::setSeed(time(0));
    }

    // Collision detection
    collisionDetection = new CollisionDetection_Rigid3D;
    collisionDetection->initialize(this);
    for (auto iter = obstacles.begin(); iter != obstacles.end(); ++iter)
    {
        collisionDetection->addObstacle(*iter);
    }
    collisionDetection->addRobot(robot);

    // Distance metric
    distance = new DistanceNormal_Rigid3D;
    distance->initialize(this);

    // Local planner
    localPlanner = new LocalPlannerStraightLine;
    localPlanner->initialize(this);

    // Nearest neighbor algorithm
    //nearestNeighbor = new NearestNeighborList;
    nearestNeighbor = new NearestNeighborLSH;
    nearestNeighbor->initialize(this);

    // Configuration generator
    generator = new GeneratorUniform_Rigid3D;
    generator->initialize(this);

    // Planner
    planner = new PlannerNormal;
    planner->initialize(this);

    // Roadmap
    roadmap = new Roadmap;
    roadmap->solver = this;
}

void Solver::uninitialize()
{
    delete collisionDetection;
    collisionDetection = NULL;

    delete distance;
    distance = NULL;

    delete localPlanner;
    localPlanner = NULL;

    delete nearestNeighbor;
    nearestNeighbor = NULL;

    delete roadmap;
    roadmap = NULL;

    for (size_t i = 0; i < obstacles.size(); ++i)
    {
        delete obstacles[i];
        obstacles[i] = NULL;
    }

    delete planner;
    planner = NULL;

    delete generator;
    generator = NULL;

    delete robot;
    robot = NULL;

    delete settings;
    settings = NULL;
}

void Solver::savePath(const RoadmapNodePath& path)
{
    std::ofstream output;
    output.open(
        m_taskDir.absoluteFilePath("path.txt").toLocal8Bit(), std::ios::out);

    if (!output.is_open())
    {
        FATALERROR("Unable to open a file");
    }

    const Configuration* cfg = roadmap->getNode(0)->configuration;

    output << *static_cast<const Configuration_Rigid3D*>(cfg) << std::endl;

    RoadmapNode* prev = NULL;
    for (auto iter = path.begin(); iter != path.end(); ++iter)
    {
        if (prev)
        {
            LocalPath localPath;
            localPlanner->getPath(localPath,
                                  prev->configuration,
                                  (*iter)->configuration);

            auto localPathIter = localPath.begin();

            // Delete the first configuration
            delete *localPathIter;
            *localPathIter = NULL;

            ++localPathIter;
            for (; localPathIter != localPath.end(); ++localPathIter)
            {
                cfg = *localPathIter;
                output << *static_cast<const Configuration_Rigid3D*>(cfg)
                       << std::endl;

                // Configuration must be deleted
                delete cfg;
                cfg = NULL;
            }
        }
        prev = *iter;
    }

    cfg = roadmap->getNode(1)->configuration;
    output << *static_cast<const Configuration_Rigid3D*>(cfg) << std::endl;

    output.close();
}

double Solver::run()
{
    clock_t startTime = clock();

    nearestNeighbor->init();

    RMID idInit = roadmap->addInitConfiguration(m_initConfiguration);
    nearestNeighbor->addRoadmapNode(idInit);
    RMID idGoal = roadmap->addGoalConfiguration(m_goalConfiguration);
    nearestNeighbor->addRoadmapNode(idGoal);

    planner->solve();

    clock_t endTime = clock();
    double staticUsedTime = (endTime - startTime) * (1000.0 / CLOCKS_PER_SEC);

    std::cout << "Time used: " << staticUsedTime << " ms" << std::endl;

    // Get the path length
    RoadmapNodePath path;
    double pathLength = -1.0;
    bool pathFound = roadmap->getPath(path, pathLength);

    // Save the path to the file
    savePath(path);

    // Print results to the screen
    if (pathFound)
    {
        std::cout << "Path length: " << pathLength << std::endl;
    }
    else
    {
        std::cout << "Path length: -" << std::endl;
    }

    std::cout << "Generated nodes: " << roadmap->getGeneratedNodeCount()
              << " (active: " << roadmap->getActiveNodeCount()
              << ", removed: " << roadmap->getRemovedNodeCount()
              << ")" << std::endl

              << "Generated edges: " << roadmap->getGeneratedEdgeCount()
              << " (active: " << roadmap->getActiveEdgeCount()
              << ", removed: " << roadmap->getRemovedEdgeCount()
              << ")" << std::endl

              << "Roadmap components: " << roadmap->getComponentCount()
              << std::endl;

    std::cout << "Random seed: " << Random::getSeed() << std::endl;

    return staticUsedTime;
}
