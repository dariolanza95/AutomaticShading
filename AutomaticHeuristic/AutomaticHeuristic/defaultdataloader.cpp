#include "defaultdataloader.h"
#include "istream"
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>


template <typename TryMesh>
DefaultDataLoader<TryMesh>::DefaultDataLoader(istream& objfile)
{

        obj=objfile;

}
