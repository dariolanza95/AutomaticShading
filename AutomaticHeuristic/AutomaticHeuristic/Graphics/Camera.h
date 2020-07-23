/****************************************************************************
    Copyright (C) 2012 Adrian Blumer (blumer.adrian@gmail.com)
    Copyright (C) 2012 Pascal Spörri (pascal.spoerri@gmail.com)
    Copyright (C) 2012 Sabina Schellenberg (sabina.schellenberg@gmail.com)

    All Rights Reserved.

    You may use, distribute and modify this code under the terms of the
    MIT license (http://opensource.org/licenses/MIT).
*****************************************************************************/

#ifndef CAMERA_H
#define CAMERA_H

#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/type_ptr.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
// Backwards compatibility
#define GLM_FORCE_RADIANS
using namespace glm;

class Camera
{
public:

    Camera();
    Camera(glm::vec3 position);
    void SetProjection(float angle=45.0f, float aspect=300.0f/200.0f, float near=0.1f, float far=100.0f);

    void SetAspectRatio(float aspect);

    /// Matrix access
    const glm::mat4x4& ProjMatrix();
    const glm::mat4x4& ViewMatrix();

    const glm::mat4x4& RIBMatrix();


    glm::vec3 Position();
    // Transform
    void TranslateGlobal(const glm::vec3& delta);
    void TranslateLocal(const glm::vec3& delta);
    void LocalRotate(const glm::vec3& axis, float angle);
    void LocalRotateRIB(const vec3 &axis, float angle);
    float _aspect;

    void GlobalRotate(const glm::vec3& axis, float angle);
    float GetScalingCostant();
    void  SetScalingCostant(float _scaling_costant);
    float rotX;
    float rotY;
    float rotZ;
protected:

    void recomputeViewMatrix();
   void recomputeRIBMatrix();

protected:

    glm::vec3   _position;
    glm::fquat  _forward;
    glm::fquat  _forward_RIB;
    glm::fquat  _upward;

    glm::mat4x4 _viewMatrix;
    glm::mat4x4 _RIBMatrix;
    glm::mat4x4 _projMatrix;


    float _scaling_costant;
    float _angle;
    float _near, _far;

};



#endif // CAMERA_H
