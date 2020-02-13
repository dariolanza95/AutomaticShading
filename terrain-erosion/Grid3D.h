/****************************************************************************
    Copyright (C) 2012 Adrian Blumer (blumer.adrian@gmail.com)
    Copyright (C) 2012 Pascal Spörri (pascal.spoerri@gmail.com)
    Copyright (C) 2012 Sabina Schellenberg (sabina.schellenberg@gmail.com)

    All Rights Reserved.

    You may use, distribute and modify this code under the terms of the
    MIT license (http://opensource.org/licenses/MIT).
*****************************************************************************/

#ifndef GRID3D_H
#define GRID3D_H

#include "platform_includes.h"
#include <vector>

template<typename T>
class Grid3D
{
protected:
    uint _width;
    uint _height;
    uint _depth;
    uint _size;
    std::vector<T> _data;

public:
    Grid3D(uint w=0, uint h=0,uint d = 0)
        : _width(w), _height(h), _depth(d),_size(w*h*d), _data(_size)
    {}

    uint width() const { return _width; }
    uint height() const { return _height;}
    uint depth() const {return _depth;}
    uint size() const { return _size; }

    void resize(uint w, uint h,uint d)
    {
        _width = w;
        _height = h;
        _depth = d;
        _size = w*h*d;
        _data.resize(w*h);
    }

    T& operator ()(uint y, uint x,uint z);
    const T& operator ()(uint y, uint x,uint z) const;

    T& operator ()(uint i);
    const T& operator ()(uint i) const;

    T* ptr() { return &_data[0]; }
    const T* ptr() const { return &_data[0]; }
};

template<typename T>
T &Grid3D<T>::operator ()(uint z,uint y, uint x)
{
    return _data[y * _width + z * _depth + x];
}

template<typename T>
const T& Grid3D<T>::operator ()(uint z,uint y, uint x) const
{
    return _data[y * _width + z * _depth + x];
}


template<typename T>
T &Grid3D<T>::operator ()(uint i)
{
    return _data[i];
}

template<typename T>
const T& Grid3D<T>::operator ()(uint i) const
{
    return _data[i];
}


#endif // GRID3D_H
