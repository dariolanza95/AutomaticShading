#include "materialclassifier.h"

/*
class SelectMaterialVerticesFunctorClass
{
    public:
    std::shared_ptr<AShader> getValue(SimulationData* sd) {
        float hardness;
        sd->getData(SimulationDataEnum::hardness,hardness);
        std::shared_ptr<AShader> shader_parameter = new MaterialShader(_id,1.0f,hardness);
        return shader_parameter;
    }
    SelectMaterialVerticesFunctorClass(){}
    SelectMaterialVerticesFunctorClass (MyMesh mesh,int id): _mesh(mesh),_id(id){}
            int operator() (SimulationData *sd) {
   //         auto simulation_data_wrapper = OpenMesh::getOrMakeProperty<MyMesh::VertexHandle,SimulationData*>(_mesh, "simulation_data");
   //         SimulationData* sd  = simulation_data_wrapper[vertex_handle];
                float hardness;
                sd->getData(SimulationDataEnum::hardness,hardness);

                if(hardness>=0.0f)
                return 1;
            else
                return 0;
            }

    private:
            MyMesh _mesh;
            int _id;
};



MaterialClassifier::MaterialClassifier(MyMesh& mesh) : AClassifier(mesh)
{
    _shader = new MaterialShader(_id);
}
map<MyMesh::VertexHandle,std::shared_ptr<AShader>> MaterialClassifier::ClassifyVertices()
{

    SelectMaterialVerticesFunctorClass functor(_mesh,_id);
   return SelectClassVertices(functor);

}
*/
