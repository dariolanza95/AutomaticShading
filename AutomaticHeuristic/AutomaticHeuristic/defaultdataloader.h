

#ifndef DEFAULTDATALOADER_H
#define DEFAULTDATALOADER_H
#include "dataloader.h"
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include <istream>

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
    DefaultDataLoader(istream& objfile);
private:
    istream& obj;
};

#endif // DEFAULTDATALOADER_H


