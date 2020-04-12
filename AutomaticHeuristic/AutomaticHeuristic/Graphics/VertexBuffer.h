/****************************************************************************
    Copyright (C) 2012 Adrian Blumer (blumer.adrian@gmail.com)

    All Rights Reserved.

    You may use, distribute and modify this code under the terms of the
    MIT license (http://opensource.org/licenses/MIT).
*****************************************************************************/

#ifndef VERTEXBUFFER_H
#define VERTEXBUFFER_H

#include "platform_includes.h"
#include "Grid2D.h"
#include "GLWrapper.h"
#ifdef Success
  #undef Success
#endif
#include <pcl/point_cloud.h>
#include <pcl/cloud_iterator.h>
#include <pcl/kdtree/kdtree_flann.h>


namespace Graphics
{

template<typename T>
class VertexBuffer
{
public:
    VertexBuffer();
    ~VertexBuffer();

    void MapData(uint location, bool normalized = false);
    void Bind();
    void SetData(const Grid2D<T>& grid);
    void SetData(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcl);
    void SetData(const std::vector<glm::vec3> points);
protected:
    GLuint _id;
};

// Implementation
///////////////////////////////////////////////

template<typename T>
inline VertexBuffer<T>::VertexBuffer()
{
    glGenBuffers(1,&_id);
}

template<typename T>
inline VertexBuffer<T>::~VertexBuffer()
{
    glDeleteBuffers(1,&_id);
}

template<typename T>
inline void VertexBuffer<T>::MapData(uint location, bool normalized)
{
    glBindBuffer(GL_ARRAY_BUFFER,_id);
    glEnableVertexAttribArray(location);
    glVertexAttribPointer(location,GL::TypeInfo<T>::ElementCount,GL::TypeInfo<T>::ElementType,normalized,0,0);
}

template<typename T>
inline void VertexBuffer<T>::Bind()
{
    glBindBuffer(GL_ARRAY_BUFFER,_id);
}

template<typename T>
inline void VertexBuffer<T>::SetData(const Grid2D<T> &grid)
{
    glBindBuffer(GL_ARRAY_BUFFER,_id);
    uint bytesize = sizeof(T)*grid.size();
    glBufferData(GL_ARRAY_BUFFER, bytesize, grid.ptr(),GL_STATIC_DRAW);
}


template<typename T>

inline void VertexBuffer<T>::SetData(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcl)
{

    glBindBuffer(GL_ARRAY_BUFFER,_id);
    uint bytesize = sizeof(T)*pcl->points.size();
    pcl::PointXYZ pp = pcl->points[0];
    std::cout<< "BOJA "<<pp.x<<std::endl;
    glBufferData(GL_ARRAY_BUFFER, bytesize, &pcl->points[0],GL_STATIC_DRAW);
}


template<typename T>

inline void VertexBuffer<T>::SetData(const std::vector<glm::vec3> points)
{

    glBindBuffer(GL_ARRAY_BUFFER,_id);
    uint bytesize = sizeof(T)* points.size();
    glBufferData(GL_ARRAY_BUFFER, bytesize, points.data(),GL_STATIC_DRAW);
}


// Instances
///////////////////////////////////////////////

//template class VertexBuffer<glm::vec3>;
//template class VertexBuffer<float>;
//template class VertexBuffer<glm::vec2>;

}

#endif // VERTEXBUFFER_H
