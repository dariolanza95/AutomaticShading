/*

#ifndef DEFAULTDATALOADER_H
#define DEFAULTDATALOADER_H
#include "dataloader.h"
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include <istream>



typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;
using namespace std;
using namespace OpenMesh;

template <class MyMesh>
class DefaultDataLoader : public DataLoader<MyMesh>
{
public:
    DefaultDataLoader<MyMesh>(string objfile,string simulation_file,MyMesh mesh);
    void LoadData();
protected:
    void LoadMesh();
    void CalculateNormals();
    void LoadGeometryData();
    void AttachDataFromSimulationToEachVertex();
    void LoadGeometryData(ifstream& mesh_ifstream,MyMesh& mesh);

    string obj_file;
    string simulation_file;
    MyMesh mesh;

};

#endif // DEFAULTDATALOADER_H

*/
