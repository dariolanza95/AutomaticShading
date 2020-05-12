#ifndef SEDIMENTATIONCLASSIFIER_H
#define SEDIMENTATIONCLASSIFIER_H
#include "aclassifier.h"

#include <glm/vec3.hpp>
#include <glm/glm.hpp>

#include "flowshader.h"


struct sedimentationData{
    glm::vec3 initial_position;
    std::vector<float> sediment_history;
    sedimentationData(glm::vec3 initial_position,std::vector<float> sediment_history): sediment_history(sediment_history)
      ,initial_position(initial_position){}

};

class SedimentationClassifier : public AClassifier
{

    OpenMesh::PropertyManager<typename OpenMesh::HandleToPropHandle<MyMesh::VertexHandle , SimulationData*>::type, MyMesh> simulation_data_wrapper;
    map<MyMesh::VertexHandle, sedimentationData*> SelectSedimentationPoints();
public:
    SedimentationClassifier(MyMesh mesh);
    map<MyMesh::VertexHandle,AShader*> ClassifyVertices();
};

#endif // SEDIMENTATIONCLASSIFIER_H
