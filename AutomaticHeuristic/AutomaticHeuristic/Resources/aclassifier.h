#ifndef ACLASSIFIER_H
#define ACLASSIFIER_H
#include <stdlib.h>
#include "ShaderWrapper.h"
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include "simulationdata.h"
#include "Ashader.h"
#include "mydefwrapper.h"
#include <memory>
#include <OpenMesh/Tools/Subdivider/Adaptive/Composite/RuleInterfaceT.hh>
#include <OpenMesh/Tools/Subdivider/Adaptive/Composite/CompositeT.hh>
//typedef OpenMesh::TriMesh_ArrayKernelT<OpenMesh::Subdivider::Adaptive::CompositeTraits>  MyMesh;
using namespace std;



class AClassifier
{
protected:

    OpenMesh::PropertyManager<typename OpenMesh::HandleToPropHandle<MyMesh::VertexHandle , SimulationData*>::type, MyMesh> simulation_data_wrapper;
    MyMesh& _mesh;
    static int _id;
    template <typename FuncType>
    map<typename MyMesh::VertexHandle,AShader*> SelectClassVertices(FuncType functor);
    AShader* _shader;
public:
    virtual void ClassifyVertices(std::vector<glm::vec3>& list_of_points,std::vector<AShader*>& list_of_data,float& details) = 0;

//    virtual map<MyMesh::VertexHandle,AShader*> ClassifyVertices() = 0;
    AShader* GetShader();
    int GetId();
    AClassifier(MyMesh& mesh);
};
template <typename FuncType>
map<typename MyMesh::VertexHandle,AShader*> AClassifier::SelectClassVertices(FuncType functor)
{
    typename MyMesh::VertexIter vertex_iterator,vertex_iterator_end(_mesh.vertices_end());
    map<typename MyMesh::VertexHandle,AShader*> class_vertices;
    for(vertex_iterator=_mesh.vertices_begin();vertex_iterator != vertex_iterator_end;++vertex_iterator)
    {
        SimulationData* sd = simulation_data_wrapper[*vertex_iterator];
        //pred(*vertex_vertex_iterator)
        if ( functor(sd) )
        {
            AShader* sp = functor.getValue(sd);
            class_vertices.insert(pair<typename MyMesh::VertexHandle,AShader*>(*vertex_iterator,sp));
        }
    }
return class_vertices;
}



#endif // ACLASSIFIER_H
