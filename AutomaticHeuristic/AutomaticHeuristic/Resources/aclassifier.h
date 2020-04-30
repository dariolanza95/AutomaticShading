#ifndef ACLASSIFIER_H
#define ACLASSIFIER_H
#include <stdlib.h>
#include "ShaderWrapper.h"
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include "simulationdata.h"
#include "Ashader.h"
typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;
using namespace std;



class AClassifier
{
protected:
    OpenMesh::PropertyManager<typename OpenMesh::HandleToPropHandle<MyMesh::VertexHandle , SimulationData*>::type, MyMesh> simulation_data_wrapper;
    MyMesh& _mesh;
    static int _id;
    template <typename FuncType>
    map<MyMesh::VertexHandle,AShader*> SelectClassVertices(FuncType functor);
    AShader* _shader;
public:
    virtual map<MyMesh::VertexHandle,AShader*> ClassifyVertices() = 0;
    AShader* GetShader();
    int GetId();
    AClassifier(MyMesh& mesh);
};
template <typename FuncType>
map<MyMesh::VertexHandle,AShader*> AClassifier::SelectClassVertices(FuncType functor)
{
    MyMesh::VertexIter vertex_iterator,vertex_iterator_end(_mesh.vertices_end());
    map<MyMesh::VertexHandle,AShader*> class_vertices;
    for(vertex_iterator=_mesh.vertices_begin();vertex_iterator != vertex_iterator_end;++vertex_iterator)
    {
        SimulationData* sd = simulation_data_wrapper[*vertex_iterator];
        //pred(*vertex_vertex_iterator)
        if ( functor(sd) )
        {
            AShader* sp = functor.getValue(sd);
            class_vertices.insert(pair<MyMesh::VertexHandle,AShader*>(*vertex_iterator,sp));
        }
    }
return class_vertices;
}



#endif // ACLASSIFIER_H
