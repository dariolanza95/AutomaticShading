/****************************************************************************
    Copyright (C) 2012 Adrian Blumer (blumer.adrian@gmail.com)
    Copyright (C) 2012 Pascal Spörri (pascal.spoerri@gmail.com)
    Copyright (C) 2012 Sabina Schellenberg (sabina.schellenberg@gmail.com)

    All Rights Reserved.

    You may use, distribute and modify this code under the terms of the
    MIT license (http://opensource.org/licenses/MIT).
*****************************************************************************/

#include "Camera.h"
#include <iostream>
#include <glm/gtc/matrix_transform.hpp>
#define GLM_ENABLE_EXPERIMENTAL;
#include <glm/gtx/quaternion.hpp>
using namespace glm;
Camera::Camera() :rotX(0),rotY(0),rotZ(0)
{
    SetProjection();
}

Camera::Camera(vec3 position):rotX(0),rotY(0),rotZ(0)
{

    //glm::vec4 xAxis = correctiveMatrix * glm::vec4(1, 0, 0, 0);
    //glm::vec4 yAxis = correctiveMatrix * glm::vec4(0, 1, 0, 0);
    //glm::vec4 zAxis = correctiveMatrix * glm::vec4(0, 0, 1, 0);
    _position = position;
      glm::quat qPitch = glm::angleAxis(0.0f,glm::vec3(1,0,0) );
      glm::quat qYaw = glm::angleAxis(0.0f, glm::vec3(0, 1,0));
      glm::quat qRoll = glm::angleAxis(0.0f,glm::vec3(0,0,1));

      glm::quat orientation = qPitch * qYaw*qRoll;
      orientation = glm::normalize(orientation);
      glm::mat4 rotate = glm::mat4_cast(orientation);

   // _forward = glm::fquat(glm::vec3(0,0,0));
    _forward = orientation;
    _forward_RIB = _forward;
    SetProjection();

}
  glm::vec3 Camera::Position()
 {
     return _position;
 }

  float Camera::GetScalingCostant(){return _scaling_costant;}
  void  Camera::SetScalingCostant(float scaling_costant){_scaling_costant = scaling_costant;}



void Camera::SetProjection(float angle, float aspect, float near, float far)
{
    _angle = angle;
    _aspect = aspect;
    _near = near;
    _far = far;
    _projMatrix = glm::perspectiveFov(angle,300.0f,200.0f,near,far);
}

void Camera::SetAspectRatio(float aspect)
{
    _aspect = aspect;
//    _projMatrix = glm::perspective(_angle,_aspect,_near,_far);
    _projMatrix = glm::perspectiveFov(_angle,300.0f,200.0f,_near,_far);
}

const glm::mat4x4 &Camera::ProjMatrix()
{
    return _projMatrix;
}

const glm::mat4x4 &Camera::ViewMatrix()
{
    //glm::mat4_cast(_forward);
    recomputeViewMatrix();
    return _viewMatrix;
}



const glm::mat4x4 &Camera::RIBMatrix()
{
    //glm::mat4_cast(_forward);
    recomputeRIBMatrix();
    return _RIBMatrix;
}

void Camera::TranslateGlobal(const vec3 &delta)
{
    _position += delta;
    recomputeViewMatrix();
}


void Camera::TranslateLocal(const vec3 &delta)
{
    vec4 d = vec4(delta,0.0f);

    if( (_forward.x == 0) & (_forward.y == 0) & (_forward.z == 0) & (_forward.w == 0) )
        d = mat4_cast((_forward))*d;
    else
         d = mat4_cast(inverse(_forward))*d;

    _position += vec3(d.x,d.y,d.z);
}

void Camera::LocalRotate(const vec3 &axis, float angle)
{
    // deg to rad
    float a = M_PI*angle/180.0f;

    vec3 ax = normalize(axis);

    float sina = sinf(a/2.0f);
    float cosa = cosf(a/2.0f);
    if(ax[0]==1 && ax[1]==0 && ax[2]==0)
    {
        rotX += angle;
        std::cout<<"Rot along X"<<-1*rotX<<std::endl;
    }
    else
    {
        if(ax[0]==0 && ax[1]==1 && ax[2]==0)
        {
            rotY += angle;
            std::cout<<"Rot along Y "<<rotY<<std::endl;
        }
        else
            if(ax[0]==0 && ax[1]==0 && ax[2]==1)
            {
                rotZ += angle;
                std::cout<<"Rot along Z"<<rotZ<<std::endl;
            }
    }
    ax *= sina;
    float s = cosa;

    fquat offset(s,ax);

        _forward = offset * _forward;



    //    fquat RIBoffset(-s,ax);
    //    _forward_RIB = RIBoffset * _forward_RIB;

}

void Camera::LocalRotateRIB(const vec3 &axis, float angle)
{
    // deg to rad
    float a = M_PI*angle/180.0f;

    vec3 ax = normalize(axis);

    float sina = sinf(a/2.0f);
    float cosa = cosf(a/2.0f);
    ax *= sina;
    float s = cosa;

    fquat offset(s,ax);

        _forward_RIB = offset * _forward_RIB;

}




void Camera::GlobalRotate(const vec3 &axis, float angle)
{
    // deg to rad
    float a = M_PI*angle/180.0f;

    vec3 ax = normalize(axis);
    float sina = sinf(a/2.0f);
    float cosa = cosf(a/2.0f);
    if(ax[0]==1 && ax[1]==0 && ax[2]==0)
    {
        rotX += angle;
        std::cout<<"Rot along X"<<-1*rotX<<std::endl;
    }
    else
    {
        if(ax[0]==0 && ax[1]==1 && ax[2]==0)
        {
            rotY += angle;
            std::cout<<"Rot along Y "<<rotY<<std::endl;
        }
        else
            if(ax[0]==0 && ax[1]==0 && ax[2]==1)
            {
                rotZ += angle;
                std::cout<<"Rot along Z"<<rotZ<<std::endl;
            }
    }
    ax *= sina;
    float s = cosa;


    fquat offset(s,ax);

    _forward =  _forward*offset;
}



void Camera::recomputeRIBMatrix()
{
    normalize(_forward_RIB);

    glm::mat4 Identity = glm::mat4(1.0f); // identity matrix
//    before was
//    _viewMatrix =  mat4_cast(_forward)*glm::translate(Identity,-_position);

    _RIBMatrix =  glm::translate(Identity,-_position)*mat4_cast(_forward_RIB);

}

void Camera::recomputeViewMatrix()
{
    normalize(_forward);
    glm::mat4 Identity = glm::mat4(1.0f); // identity matrix
//Identity = Mirror;
    //    before was
//    _viewMatrix =  mat4_cast(_forward)*glm::translate(Identity,-_position);

    _viewMatrix =  glm::translate(Identity,-_position)*mat4_cast(_forward);

}
