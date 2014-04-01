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

#ifndef __GRDMODEL_H__
#define __GRDMODEL_H__

#include <iostream>
#include <string>

#include "Common.h"
#include "Configuration.h"


/*****************************************************************************/
/* Object                                                                    */
/*****************************************************************************/

class Object
{
public:
    Object():
        m_ID(-1),
        m_configuration(NULL)
    {
    }

    virtual ~Object()
    {
        delete m_configuration;
        m_configuration = NULL;
    }

    void setFilename(const QString filename)
    {
        m_filename = filename;
    }

    int getID() const
    {
        return m_ID;
    }

    void setID(const int id)
    {
        m_ID = id;
    }

    const QString& getFilename() const
    {
        return m_filename;
    }


    Configuration* getConfiguration() const
    {
        return m_configuration;
    }

    virtual void setConfiguration(const Configuration* const config) = 0;

    virtual void initializeRobot() = 0;

    virtual void initializeObstacle() = 0;

    friend std::ostream& operator<<(std::ostream& out,
                                    const Object& object)
    {
        out << "Object " << object.m_ID << " " << object.m_configuration;

        return out;
    }

protected:
    int m_ID;
    QString m_filename;
    Configuration* m_configuration;
};



/*****************************************************************************/
/* Object_Rigid3D                                                            */
/*****************************************************************************/

class Object_Rigid3D: public Object
{
public:
    Object_Rigid3D(): Object()
    {
        // The destructor of Object will delete this
        m_configurationRigid3D = new Configuration_Rigid3D;
        m_configuration = m_configurationRigid3D;
    }

    virtual ~Object_Rigid3D()
    {
        // Nothing
    }

    void setPosition(const Position_Rigid3D& position)
    {
        m_configurationRigid3D->position = position;
    }

    void setRotation(const Rotation_Rigid3D& rotation)
    {
        m_configurationRigid3D->rotation = rotation;
    }

    virtual void setConfiguration(const Configuration* const config)
    {
        m_configurationRigid3D->position =
                static_cast<const Configuration_Rigid3D*>(config)->position;
        m_configurationRigid3D->rotation =
                static_cast<const Configuration_Rigid3D*>(config)->rotation;
    }

    virtual void initializeRobot()
    {
        // Nothing
    }

    virtual void initializeObstacle()
    {
        // Nothing
    }

protected:
    Configuration_Rigid3D* m_configurationRigid3D;
};

#endif
