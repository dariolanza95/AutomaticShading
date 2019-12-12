#ifndef DEFAULTDATALOADER_H
#define DEFAULTDATALOADER_H
#include "DataLoader.h"
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>


typedef OpenMesh::TriMesh_ArrayKernelT<>  TryMesh;
using namespace std;
using namespace OpenMesh;
/*
template <class Kernel>
class TriMeshT : public PolyMeshT<Kernel>
{


}*/
template <class TryMesh>
class DefaultDataLoader : public DataLoader<TryMesh>
{
public:
    DefaultDataLoader();
};

#endif // DEFAULTDATALOADER_H
