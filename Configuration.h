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

#ifndef __CONFIGURATION_H__
#define __CONFIGURATION_H__

#include <vector>

#include "Common.h"

typedef Eigen::Vector3d Position_Rigid3D;
typedef Eigen::Quaterniond Rotation_Rigid3D;


/*****************************************************************************/
/* Configuration                                                             */
/*****************************************************************************/

class Configuration
{
public:
    virtual ~Configuration()
    {
    }

    virtual void interpolate(const Configuration* const config,
                             double position,
                             Configuration* const outConfig) const = 0;
};



/*****************************************************************************/
/* Configuration_Rigid3D                                                     */
/*****************************************************************************/

class Configuration_Rigid3D: public Configuration
{
public:
    Configuration_Rigid3D(): Configuration()
    {
    }

    Configuration_Rigid3D(const Configuration_Rigid3D& config)
    {
        position = config.position;
        rotation = config.rotation;
    }

    const Configuration_Rigid3D& operator=(const Configuration_Rigid3D& config)
    {
        this->position = config.position;
        this->rotation = config.rotation;

        return *this;
    }

    friend std::ostream& operator<<(std::ostream& out,
                                    const Configuration_Rigid3D& config)
    {
        out.precision(4);

        out << std::fixed
            << config.position[0] << " "
            << config.position[1] << " "
            << config.position[2] << " "
            << config.rotation.w() << " "
            << config.rotation.x() << " "
            << config.rotation.y() << " "
            << config.rotation.z();

        return out;
    }

    void setRandomRotation()
    {
        const double u1 = Random::getUniform();
        const double u2 = Random::getUniform();
        const double u3 = Random::getUniform();

        rotation = Rotation_Rigid3D(sqrt(1 - u1) * sin(2 * M_PI * u2),
                                    sqrt(1 - u1) * cos(2 * M_PI * u2),
                                    sqrt(u1) * sin(2 * M_PI * u3),
                                    sqrt(u1) * cos(2 * M_PI * u3));
    }

    virtual void interpolate(const Configuration* const config,
                             double t,
                             Configuration* const outConfig) const
    {
        const Configuration_Rigid3D* const conf =
                static_cast<const Configuration_Rigid3D* const>(config);
        Configuration_Rigid3D* outConf =
                static_cast<Configuration_Rigid3D*>(outConfig);

        outConf->position = position * (1.0 - t) + conf->position * t;
        outConf->rotation = rotation.slerp(t, conf->rotation);
    }

    Position_Rigid3D position;
    Rotation_Rigid3D rotation;
};

#endif
