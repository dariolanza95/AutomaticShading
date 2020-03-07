#include "materialclassifier.h"


class SelectMaterialVerticesFunctorClass
{
    public:
    ShaderParameters* getValue(SimulationData* sd) {
        ShaderParameters* shader_parameter = new ShaderParameters(_id,4);
         float val = boost::any_cast<float>(sd->_map.at("hardness"));
        shader_parameter->setValue(0,val);


        if(val > 20 && val<26)
        {
            shader_parameter->setValue(1,1);
            shader_parameter->setValue(2,0.4);
            shader_parameter->setValue(3,0);
        }
        else
        {
            shader_parameter->setValue(1,1);
            shader_parameter->setValue(2,0.1);
            shader_parameter->setValue(3,0);
        }

        return shader_parameter;
    }
    SelectMaterialVerticesFunctorClass(){}
    SelectMaterialVerticesFunctorClass (MyMesh mesh,int id): _mesh(mesh),_id(id){}
            int operator() (SimulationData *sd) {
   //         auto simulation_data_wrapper = OpenMesh::getOrMakeProperty<MyMesh::VertexHandle,SimulationData*>(_mesh, "simulation_data");
   //         SimulationData* sd  = simulation_data_wrapper[vertex_handle];
            if(boost::any_cast<float>(sd->_map.at("hardness"))>=0.0f)
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
}
map<MyMesh::VertexHandle,ShaderParameters*> MaterialClassifier::ClassifyVertices()
{

    SelectMaterialVerticesFunctorClass functor(_mesh,_id);
   map<MyMesh::VertexHandle,ShaderParameters*>  res;
  res = SelectClassVertices(functor);
return res;
}
