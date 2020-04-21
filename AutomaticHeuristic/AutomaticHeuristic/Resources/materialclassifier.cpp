#include "materialclassifier.h"


class SelectMaterialVerticesFunctorClass
{
    public:
    AShader* getValue(SimulationData* sd) {
        float hardness;
        sd->getData(SimulationDataEnum::hardness,hardness);
        AShader* shader_parameter = new MaterialShader(_id,1.0f,hardness);
//        shader_parameter->AddParameter(ShaderParametersEnum::hardness,hardness);// setValue(0,hardness);
//        glm::vec3 flow_normal;
//        sd->getData(SimulationDataEnum::flow_normal,flow_normal);
//        shader_parameter->AddParameter(ShaderParametersEnum::flow_normal,flow_normal);
    /*        if(!isnan(flow_normal[0]))
                shader_parameter->setValue(1,flow_normal[0]);
            else
                shader_parameter->setValue(2,0);
            if(!isnan(flow_normal[1]))
                shader_parameter->setValue(2,flow_normal[1]);
            else
                shader_parameter->setValue(1,0);
            if(!isnan(flow_normal[2]))
                shader_parameter->setValue(3,flow_normal[2]);
            else
                shader_parameter->setValue(3,0);*/
            //shader_parameter->setValue(2,flow_normal[1]);
            //shader_parameter->setValue(3,flow_normal[2]);



        //if(val > 20 && val<26)
        //{
        //    shader_parameter->setValue(1,1);
        //    shader_parameter->setValue(2,0.1);
        //    shader_parameter->setValue(3,0);
        //}
        //else
        //{
        //    shader_parameter->setValue(1,1);
        //    shader_parameter->setValue(2,0.4);
        //    shader_parameter->setValue(3,0);
        //}

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



MaterialClassifier::MaterialClassifier(MyMesh mesh) : AClassifier(mesh)
{
    _shader = new MaterialShader(_id);
}
map<MyMesh::VertexHandle,AShader*> MaterialClassifier::ClassifyVertices()
{

    SelectMaterialVerticesFunctorClass functor(_mesh,_id);
   return SelectClassVertices(functor);

}

