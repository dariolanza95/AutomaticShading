/*#include "defaultdataloader.h"

#include <istream>
#include <fstream>
#include <string>
#include <iostream>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include <OpenMesh/Core/IO/MeshIO.hh>


using namespace std;
using namespace OpenMesh;



DefaultDataLoader<MyMesh>::DefaultDataLoader<MyMesh> (string objfile,string simulation_file,MyMesh mesh)
{
 cout<<"hola";
}




template <>
void DefaultDataLoader<MyMesh>::AttachDataFromSimulationToEachVertex()
{
    string line;
    int counter=0;
    ifstream inputfile  (simulation_file);
    auto simulation_data = getOrMakeProperty<VertexHandle, float>(mesh, "simulation_data");
    for (auto& vertex_handle : mesh.vertices())
    {
        getline(inputfile,line);
        counter++;
       // simulation_data[vertex_handle] = stof(line );
        simulation_data[vertex_handle] = 0;
    }
cout<<"total == "<< counter<<endl;
}

template <>
void DefaultDataLoader<MyMesh>::LoadMesh()
{

    OpenMesh::IO::Options opt;
    if ( ! OpenMesh::IO::read_mesh(mesh,obj_file, opt))
    {

      std::cerr  << "Error loading mesh from file " << obj_file<< endl;

    }

}
template <>
void DefaultDataLoader<MyMesh>::CalculateNormals()
{
    OpenMesh::IO::Options opt;
    if ( !opt.check( OpenMesh::IO::Options::VertexNormal ) )
    {
        // we need face normals to update the vertex normals
        mesh.request_face_normals();

        // let the mesh update the normals
        mesh.update_normals();

    }
}
template <>
void DefaultDataLoader<MyMesh>::LoadGeometryData()
{
    mesh.request_vertex_normals();
    if (!mesh.has_vertex_normals())
    {
      std::cerr << "ERROR: Standard vertex property 'Normals' not available!\n";
    }
    LoadMesh();
    CalculateNormals();
}
template <>
 void DefaultDataLoader<MyMesh>::LoadData()
{
    LoadGeometryData();
    AttachDataFromSimulationToEachVertex();
}

*/

