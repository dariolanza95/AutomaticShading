#ifndef ACLASSIFIER_H
#define ACLASSIFIER_H
#include <stdlib.h>
#include "ShaderWrapper.h"
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include "simulationdata.h"
#include "AShader.h"
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
    MyMesh _mesh;
    int _id;
    static int shared_id;
    template <typename FuncType>
    map<typename MyMesh::VertexHandle,std::shared_ptr<AShader>> SelectClassVertices(FuncType functor);
    std::shared_ptr<AShader> _shader;
public:
    virtual void ClassifyVertices(std::vector<glm::vec3>& list_of_points,std::vector<std::shared_ptr<AShader>>& list_of_data,float& details) = 0;
    virtual ~AClassifier();
    virtual std::shared_ptr<AShader> GetShader() = 0;

//    virtual map<MyMesh::VertexHandle,std::shared_ptr<AShader>> ClassifyVertices() = 0;
    int GetId();
    AClassifier(MyMesh mesh);
};
template <typename FuncType>
map<typename MyMesh::VertexHandle,std::shared_ptr<AShader>> AClassifier::SelectClassVertices(FuncType functor)
{
    typename MyMesh::VertexIter vertex_iterator,vertex_iterator_end(_mesh.vertices_end());
    map<typename MyMesh::VertexHandle,std::shared_ptr<AShader>> class_vertices;
    for(vertex_iterator=_mesh.vertices_begin();vertex_iterator != vertex_iterator_end;++vertex_iterator)
    {
        SimulationData* sd = simulation_data_wrapper[*vertex_iterator];
        //pred(*vertex_vertex_iterator)
        if ( functor(sd) )
        {
            std::shared_ptr<AShader> sp = functor.getValue(sd);
            class_vertices.insert(pair<typename MyMesh::VertexHandle,std::shared_ptr<AShader>>(*vertex_iterator,sp));
        }
    }
return class_vertices;
}



#endif // ACLASSIFIER_H
