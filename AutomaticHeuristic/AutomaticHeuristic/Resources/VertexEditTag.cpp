#include "VertexEditTag.h"

using namespace  std;
VertexEditTag::VertexEditTag(vector<int> list_of_indices_inside_shader_parameters) : indices_to_be_changed(list_of_indices_inside_shader_parameters){}

VertexEditTag::VertexEditTag(){}
void VertexEditTag::AddVertexChange(VertexChanges _vertex_changes)
{
    list_of_vertex_changes.push_back(_vertex_changes);
}

string VertexEditTag::GetVertexPath()
{
    stringstream result(" [");
    for(uint i = 0;i<list_of_vertex_changes.size();i++)
    {
         VertexChanges vertex_changes = list_of_vertex_changes[i];
          result << " "<<vertex_changes.vertex_path.size();
          for(uint j = 0;j<vertex_changes.vertex_path.size();j++)
          {
              result << " "<<vertex_changes.vertex_path[j];
          }
    }
    result << " ] "<<endl;
return result.str();
}

string VertexEditTag::GetAttributesToBeChanged()
{
    stringstream result(" [");
    for(uint i = 0; i< indices_to_be_changed.size();i++)
    {
        std::cout<<"BIG ERROR"<<std::endl;
        result << " \"set\" ";
        result << "\"shader_parameter_"<<indices_to_be_changed[i]<<"\"";
        result << " \"value\"";
    }
    result<< " ] "<<endl;
    return result.str();
}
string VertexEditTag::GetValuesToBeChanged()
{
    stringstream result(" [");
    for(uint i = 0;i<list_of_vertex_changes.size();i++)
    {
         VertexChanges vertex_changes = list_of_vertex_changes[i];
          for(uint j = 0;j<vertex_changes.new_values.size();j++)
          {
            result << " "<< vertex_changes.new_values[j];
          }
    }
    result << " ] "<<endl;
    return result.str();
}
