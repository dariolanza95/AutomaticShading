/****************************************************************************
    Copyright (C) 2012 Adrian Blumer (blumer.adrian@gmail.com)

    All Rights Reserved.

    You may use, distribute and modify this code under the terms of the
    MIT license (http://opensource.org/licenses/MIT).
*****************************************************************************/

#include "IndexBuffer.h"

namespace Graphics
{

IndexBuffer::IndexBuffer()
{
    glGenBuffers(1,&_id);
}

IndexBuffer::~IndexBuffer()
{
    glDeleteBuffers(1,&_id);
}

void IndexBuffer::Bind()
{
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, _id);
}

uint IndexBuffer::IndexCount() const
{
    return _indexCount;
}

GLuint IndexBuffer::IndexType() const
{
    return _indexType;
}

/*
template<typename T>
void IndexBuffer::SetData(const std::vector<T>& data)
{
    assert(data.size() > 0);

    // Remember number of indices in data
    _indexCount = data.size();

    // Remember the index type
    rememberType(data.front());

    uint byteSize = _indexCount*sizeof(T);

    // Store data on the GPU
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER,_id);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, byteSize, &data.front(), GL_STATIC_DRAW);
}
*/

template<>
void IndexBuffer::rememberType(const uint& value)
{
    _indexType = GL_UNSIGNED_INT;
}

template<>
void IndexBuffer::rememberType(const ushort& value)
{
    _indexType = GL_UNSIGNED_SHORT;
}


} // namespace Graphics



