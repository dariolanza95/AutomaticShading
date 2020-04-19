#ifndef ACLASSIFIER_H
#define ACLASSIFIER_H
#include <stdlib.h>
#include "ShaderParameters.h"
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include "simulationdata.h"

typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;
using namespace std;



class AClassifier
{
protected:
    OpenMesh::PropertyManager<typename OpenMesh::HandleToPropHandle<MyMesh::VertexHandle , SimulationData*>::type, MyMesh> simulation_data_wrapper;
    MyMesh _mesh;
    static int _id;
    template <typename FuncType>
    map<MyMesh::VertexHandle,ShaderParameters*> SelectClassVertices(FuncType functor);

public:
    virtual map<MyMesh::VertexHandle,ShaderParameters*> ClassifyVertices() = 0;
    AClassifier(MyMesh mesh);
};
template <typename FuncType>
map<MyMesh::VertexHandle,ShaderParameters*> AClassifier::SelectClassVertices(FuncType functor)
{
    MyMesh::VertexIter vertex_iterator,vertex_iterator_end(_mesh.vertices_end());
    map<MyMesh::VertexHandle,ShaderParameters*> class_vertices;
    for(vertex_iterator=_mesh.vertices_begin();vertex_iterator != vertex_iterator_end;++vertex_iterator)
    {
        SimulationData* sd = simulation_data_wrapper[*vertex_iterator];
        //pred(*vertex_vertex_iterator)
        if ( functor(sd) )
        {
            ShaderParameters* sp = functor.getValue(sd);
            class_vertices.insert(pair<MyMesh::VertexHandle,ShaderParameters*>(*vertex_iterator,sp));
        }
    }
return class_vertices;
}



#endif // ACLASSIFIER_H
