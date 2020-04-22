#include "flowclassifier.h"
using namespace  OpenMesh;

FlowClassifier::FlowClassifier(MyMesh mesh) : AClassifier(mesh),_shader_parameter_size(3)
{
    simulation_data_wrapper = OpenMesh::getOrMakeProperty<MyMesh::VertexHandle,SimulationData*>(_mesh, "simulation_data");
     _shader = new FlowShader(_id);
}

 VertexEditTag FlowClassifier::GetVertexEditTag()
 {
     return _vertex_edit_tag;
 }

 void FlowClassifier::TemporaryUpdate(map<MyMesh::VertexHandle,FlowShader*> selected_vertices)
 {
     auto shader_parameters_property = getOrMakeProperty<VertexHandle, FlowShader*>(_mesh, "flow_shader_temp_propery");
     for (auto const& x : selected_vertices)
     {
         MyMesh::VertexHandle vertex_handle = x.first;
         FlowShader* shader = x.second;
         shader_parameters_property[vertex_handle] = shader;
         //ShadersWrapper* shaders_wrapper = shader_parameters_property[vertex_handle];
         //shaders_wrapper->AddShaderParameters(shader);
 //        shader_parameters_property[vertex_handle] = shader_parameter;

     }
 }


map<MyMesh::VertexHandle,AShader*> FlowClassifier::ClassifyVertices()
{

    map<MyMesh::VertexHandle,AShader*> selected_vertices;
    map<MyMesh::VertexHandle,FlowShader*> temporary_selected_vertices;

    auto flow_vertices = selectFlowVertices();
    ComputeShaderParameters(flow_vertices);
    //TemporaryUpdate(temporary_selected_vertices);
    selected_vertices = LIC(10,1,20);

    return selected_vertices;
}


//void FlowClassifier::RefineShaderParameters(MyMesh::VertexHandle vertex,ShaderParameters* shader_parameters)
//{
//    //except for border vertices they should all have valence 6
//    int valence = _mesh.valence(vertex);
//    int i = 0;
//    for (MyMesh::VertexIHalfedgeIter vertex_half_edge_iterator = _mesh.vih_iter(vertex); vertex_half_edge_iterator.is_valid(); ++vertex_half_edge_iterator)
//    {
//        i++;
//        MyMesh::Halfedge half_edge = _mesh.halfedges(*vertex_half_edge_iterator);
//
//        OpenMesh::Concepts::MeshItems::HalfedgeT< Refs_ >::face_handle 	( 		) 	const
//        MyMesh::Vertex opposite_vertex = _mesh.opposite_vertex_handle(*vertex_half_edge_iterator);
//        MyMesh::Face face = _mesh.faces(*vertex_half_edge_iterator);
//    }
//
//}

/*map<MyMesh::VertexHandle,ShaderParameters*> FlowClassifier::DebugFunction(map<MyMesh::VertexHandle,glm::vec3> flow_vertices )
{
    map<MyMesh::VertexHandle,ShaderParameters*> map;

    //_vertex_edit_tag.AddVertexChange();
    for(pair<MyMesh::VertexHandle,glm::vec3> entry : flow_vertices)
    {
        MyMesh::VertexFaceIter vertex_face_circulator;

        ShaderParameters* shader_parameters = new ShaderParameters(_id,1.0f);

        glm::vec3 final_vector = glm::vec3(1,0,0);
        shader_parameters->AddParameter(ShaderParametersEnum::flow_normal,final_vector);
        //shader_parameters->setValue(0,final_vector[0]);
        //shader_parameters->setValue(1,final_vector[1]);
        //shader_parameters->setValue(2,final_vector[2]);
       // RefineShaderParameters(entry.first,shader_parameters);
        map.insert(make_pair(entry.first,shader_parameters));

    }
    vector<int>   vertex_path;
    vector<float> new_values;
    VertexChanges vertex_changes();
    return        map;
}
*/

//map<MyMesh::VertexHandle,AShader*>
void FlowClassifier::ComputeShaderParameters(map<MyMesh::VertexHandle,glm::vec3>  flow_vertices)
{
  //  map<MyMesh::VertexHandle,FlowShader*> map;
    glm::vec3 up(0,1,0);
  //  map<MyMesh::VertexHandle,AShader*> map;
    auto shader_parameters_property = getOrMakeProperty<VertexHandle, FlowShader*>(_mesh, "flow_shader_temp_propery");
    for(pair<MyMesh::VertexHandle,glm::vec3> entry : flow_vertices)
    {

        if(!glm::any(glm::isnan(entry.second)))
        {


            //the shaders requires the orthogoanl vector
          glm::vec3 orthogonal_vector = glm::cross(entry.second,up);

          //for(int i=0;i<3;i++)
          //  orthogonal_vector[i] = orthogonal_vector[i]>0.01 ? orthogonal_vector[i]:0;
          orthogonal_vector = glm::normalize (orthogonal_vector);          
          glm::vec3 tangent_vector = glm::cross(entry.second,orthogonal_vector);

         MyMesh::Normal openMesh_normal = _mesh.normal(entry.first);
         glm::vec3 normal = glm::vec3(openMesh_normal[0],openMesh_normal [1],openMesh_normal[2]);
         float res =   fabs(dot(normal,orthogonal_vector));
         float res_2 = fabs(dot(normal,tangent_vector));
         glm::vec3 final_vector = res > res_2 ? tangent_vector : orthogonal_vector;
         //final_vector = tangent_vector;
         FlowShader* shader = new FlowShader(_id,1.0f,final_vector);
         //shader_parameters->AddParameter(ShaderParametersEnum::flow_normal,final_vector);
         //shader_parameters->setValue(0,final_vector[0]);
         //shader_parameters->setValue(1,final_vector[1]);
         //shader_parameters->setValue(2,final_vector[2]);
         //shader_parameters->setVector(tangent_vector);
         glm::vec3 victor = shader->GetFlowNormal();

         shader_parameters_property[entry.first] = shader;
//         ShadersWrapper* shaders_wrapper = shader_parameters_property[entry.first];
//         shaders_wrapper->AddShaderParameters(shader);

        //  map.insert(make_pair(entry.first,shader));
    }
    }
//return map;
}



void FlowClassifier::ContrastEnhancement(map<MyMesh::VertexHandle,AShader*> map,std::vector<int> Pdf)
{
    int L = Pdf.size();
    float value_counter= 0;
    float a = 0;
    float b = 1;
    float c = 0;
    float d = 1;
    int num_vertices = map.size();
    for(int i = 0;i<L;i++)
    {
        value_counter += Pdf[i];
        if ( value_counter < num_vertices * 5 /100  )
            if(i!=0)
                c =(float) (i-1)/L;
            else
                c= (float) i/L;
        if ( value_counter > num_vertices* 95 /100  )
        {
            if(i!=num_vertices)
                d =(float) (i+1)/L;
            else
                d= (float) i/L;
            break;
        }
    }

    MyMesh::VertexIter vertex_iterator;//=_mesh.vertices_begin();
    //MyMesh::VertexIter vertex_iterator_end=_mesh.vertices_end();
    //shader_parameters_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, ShaderParameters*>(_mesh, "shader_parameters");

    FlowShader* shader;

    for (auto const& x : map)
    {
      shader =(FlowShader*) x.second;
      float val = shader->GetLicValue();
      val = (val - c)*((b-a)/(d-c)) + a;
      shader->SetLicValue(val);
    }
      // for(;vertex_iterator!= vertex_iterator_end;++vertex_iterator)
      // {
           //shader_param = shader_parameters_data_wrapper[mesh_vertex_iterator];


         //shader_param->setValue(5,val);
         //shader_parameters_data_wrapper[mesh_vertex_iterator] = shader_param;
         //new_point_cloud->points[k].intensity = val;
    //   }
}



map<MyMesh::VertexHandle,AShader*> FlowClassifier::LIC(float box_length,float step_size,float frequency)
{
    FastNoise noise;
    map<MyMesh::VertexHandle,AShader*> map;
    std::cout<<"InterpolationDone"<<std::endl;
    MyMesh::Point actual_point;
    MyMesh::Point circulator_point;
    MyMesh::Point offset;
    glm::vec3 flow_vector(0,0,0);
    MyMesh::VertexVertexIter vvit;
    MyMesh::VertexVertexIter next_point;
    ShadersWrapper*  shader_wrapper;
    FlowShader* flow_shader;
    auto shader_parameters_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, FlowShader*>(_mesh, "flow_shader_temp_propery");

    float dist;
    float min_dist = INFINITY;
    int k = 0;
    int L = 1000;
    int quantizied_level;
    std::vector<int> Pdf(L,0) ;
    MyMesh::VertexIter mesh_vertex_iterator;
    MyMesh::VertexIter vertex_iterator_end(_mesh.vertices_end());


    int num_vertices = _mesh.n_vertices();
    int percentual_part = num_vertices/100;
    k = 0;
    float total_value = 0;
    mesh_vertex_iterator=_mesh.vertices_begin();

    for(;mesh_vertex_iterator!= vertex_iterator_end;++mesh_vertex_iterator,k++)
    {
     if(k%percentual_part == 0)
         std::cout<< "LIC map "<<k / percentual_part<< " %  completed"<< std::endl;

        //fw advection
 float val = 0;
 int w = 0;
 flow_shader = shader_parameters_data_wrapper[mesh_vertex_iterator];

 if(flow_shader==nullptr)
      continue;
 else{
  for(int j=0;j<2;j++)
  {
      float samepoint_iter = 1;
      actual_point = _mesh.point(*mesh_vertex_iterator);
      flow_shader = shader_parameters_data_wrapper[mesh_vertex_iterator];
       flow_vector = flow_shader->GetFlowNormal();
       if(glm::any(glm::isnan(flow_vector)))
       {
           std::cout<<"Error flow_vect is empty AT THE BEGGINING"<<std::endl;
       }
      // if(shader_param == nullptr)
      //     std::cout<<"Err shader param is empty"<<std::endl;
      // else
      //   flow_vector = glm::vec3( shader_param->getValue(1),shader_param->getValue(2),shader_param->getValue(3));

      vvit = _mesh.vv_iter(mesh_vertex_iterator);
      // = _mesh.vv_iter(*mesh_vertex_iterator);

         for(float i = 0;i<box_length;i++)
             {
                 if(j==1)
                     flow_vector = -flow_vector;
                //  MyMesh::VertexVertexIter vvit = _mesh.vv_iter(next_point);
                  actual_point[0] += flow_vector[0]*step_size;
                  actual_point[1] += flow_vector[1]*step_size;
                  actual_point[2] += flow_vector[2]*step_size;

                  dist = 0;
                  min_dist = INFINITY;
                  if(i!=0 && samepoint_iter == 1)
                      vvit = _mesh.vv_iter(vvit);
                  else
                       vvit = _mesh.vv_iter(mesh_vertex_iterator);
                  for ( ;vvit.is_valid(); ++vvit)
                  {
                      circulator_point = _mesh.point(*vvit);
                     offset = actual_point - circulator_point;
                      dist = dot(offset,offset);
                     if(dist<min_dist)
                     {
                         min_dist = dist;
                         next_point = vvit;
                     }
                  }
                  float n = noise.GetPerlin(actual_point[0] * frequency ,
                                            actual_point[1] * frequency,
                                            actual_point[2] * frequency);


 //                 if( next_point.is_valid())
 //                    actual_point = _mesh.point(*next_point);

                  val+=n;
                  w++;
                  if( next_point.is_valid())
                  {
                      glm::vec3 next_flow_vector ;
                      flow_shader = shader_parameters_data_wrapper[next_point];
                      if(flow_shader!=nullptr)
                      {
                          next_flow_vector = flow_shader->GetFlowNormal();
                          flow_vector = next_flow_vector;

                      }


                //      shader_wrapper = shader_parameters_data_wrapper[mesh_vertex_iterator];
                //      shader_wrapper->GetListOfShaders(list);
                //      for(ShaderParameters sp : list)
                //      {
                //          sp.GetParameter(ShaderParametersEnum::flow_normal,next_flow_vector);
                //
                //      }


                   //   shader_param = shader_parameters_data_wrapper[next_point];
                   //   glm::vec3 next_flow_vector = glm::vec3( shader_param->getValue(1),shader_param->getValue(2),shader_param->getValue(3));
 //                     flow_vector = flow_vector* (1-1/(min_dist+1)) + next_flow_vector*(1-min_dist/(min_dist+1));
             //         flow_vector = next_flow_vector;
                      //vvit  = _mesh.vv_iter(next_point);
                      vvit = next_point;
                     samepoint_iter = 1;
                  }
                  else
                  {
                     samepoint_iter++;
                  }
             }
     }
  val = val/ w;

  //float a  = 0;
  //float b = 1;
  //float c = 0.43;
  //float d = 0.57;
  //
  //
  val = 0.5 + 0.5 * val ;
  //val = (val - c)*((b-a)/(d-c)) + a;
  val = val > 1 ? 1 : val;
  total_value += val;
  quantizied_level =  roundf(val*L);
  Pdf[quantizied_level]++;
  actual_point = _mesh.point(*mesh_vertex_iterator);
  FlowShader* old_flow_shader = shader_parameters_data_wrapper[mesh_vertex_iterator];
  //glm::vec3 victor = old_flow_shader->GetFlowNormal();
  AShader* new_flow_shader = new FlowShader(_id,1.0,old_flow_shader->GetFlowNormal(),val);
//std::pair<MyMesh::VertexHandle,AShader*> pair= std::make_pair(mesh_vertex_iterator,new_flow_shader);
  map.insert(std::make_pair(mesh_vertex_iterator,new_flow_shader));
  //std::cout<<"val "<<shader_param->getValue(5)<<std::endl;
  //shader_parameters_data_wrapper[mesh_vertex_iterator] = shader_param;
 }
    }
    ContrastEnhancement(map,Pdf);

return map;


}
/*
void FlowClassifier::ComputeOutputs()
{

    FastNoise noise;
    SubdividerAndInterpolator<MyMesh> catmull;
    if(_subdiv_levels>0)
    {
        catmull.attach(_mesh);
        catmull(_subdiv_levels);
        catmull.detach();

    }

auto shader_parameters_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, ShadersWrapper*>(_mesh, "shader_parameters");


std::cout<<"InterpolationDone"<<std::endl;
MyMesh::Point actual_point;
MyMesh::Point circulator_point;
MyMesh::Point offset;
glm::vec3 flow_vector(0,0,0);
MyMesh::VertexVertexIter vvit;
MyMesh::VertexVertexIter next_point;
ShaderParameters  shader_param;
ShadersWrapper*  shader_wrapper;

float dist;
float min_dist = INFINITY;
int k = 0;
int L = 1000;
int quantizied_level;
std::vector<int> Pdf(L,0) ;
MyMesh::VertexIter mesh_vertex_iterator;
MyMesh::VertexIter vertex_iterator_end(_mesh.vertices_end());


int num_vertices = _mesh.n_vertices();
int percentual_part = num_vertices/100;
k = 0;
float total_value = 0;
mesh_vertex_iterator=_mesh.vertices_begin();
   for(;mesh_vertex_iterator!= vertex_iterator_end;++mesh_vertex_iterator,k++)
   {
    if(k%percentual_part == 0)
        std::cout<< "LIC map "<<k / percentual_part<< " %  completed"<< std::endl;

       //fw advection
float val = 0;
int w = 0;

 for(int j=0;j<2;j++)
 {
     float samepoint_iter = 1;
     actual_point = _mesh.point(*mesh_vertex_iterator);

      shader_wrapper = shader_parameters_data_wrapper[mesh_vertex_iterator];
      std::vector<ShaderParameters> list;
      shader_wrapper->GetListOfShaders(list);
      for(ShaderParameters sp : list)
      {
          sp.GetParameter(ShaderParametersEnum::flow_normal,flow_vector);

      }
     // if(shader_param == nullptr)
     //     std::cout<<"Err shader param is empty"<<std::endl;
     // else
     //   flow_vector = glm::vec3( shader_param->getValue(1),shader_param->getValue(2),shader_param->getValue(3));

     vvit = _mesh.vv_iter(mesh_vertex_iterator);
     // = _mesh.vv_iter(*mesh_vertex_iterator);

        for(float i = 0;i<box_length;i++)
            {
                if(j==1)
                    flow_vector = -flow_vector;
               //  MyMesh::VertexVertexIter vvit = _mesh.vv_iter(next_point);
                 actual_point[0] += flow_vector[0]*step_size;
                 actual_point[1] += flow_vector[1]*step_size;
                 actual_point[2] += flow_vector[2]*step_size;

                 dist = 0;
                 min_dist = INFINITY;
                 if(i!=0 && samepoint_iter == 1)
                     vvit = _mesh.vv_iter(vvit);
                 else
                      vvit = _mesh.vv_iter(mesh_vertex_iterator);
                 for ( ;vvit.is_valid(); ++vvit)
                 {
                     circulator_point = _mesh.point(*vvit);
                    offset = actual_point - circulator_point;
                     dist = dot(offset,offset);
                    if(dist<min_dist)
                    {
                        min_dist = dist;
                        next_point = vvit;
                    }
                 }
                 float n = noise.GetPerlin(actual_point[0] * frequency ,
                                           actual_point[1] * frequency,
                                           actual_point[2] * frequency);


//                 if( next_point.is_valid())
//                    actual_point = _mesh.point(*next_point);

                 val+=n;
                 w++;
                 if( next_point.is_valid())
                 {
                     glm::vec3 next_flow_vector ;
                     shader_wrapper = shader_parameters_data_wrapper[mesh_vertex_iterator];
                     shader_wrapper->GetListOfShaders(list);
                     for(ShaderParameters sp : list)
                     {
                         sp.GetParameter(ShaderParametersEnum::flow_normal,next_flow_vector);

                     }


                  //   shader_param = shader_parameters_data_wrapper[next_point];
                  //   glm::vec3 next_flow_vector = glm::vec3( shader_param->getValue(1),shader_param->getValue(2),shader_param->getValue(3));
//                     flow_vector = flow_vector* (1-1/(min_dist+1)) + next_flow_vector*(1-min_dist/(min_dist+1));
                     flow_vector = next_flow_vector;
                     //vvit  = _mesh.vv_iter(next_point);
                     vvit = next_point;
                    samepoint_iter = 1;
                 }
                 else
                 {
                    samepoint_iter++;
                 }
            }
    }
 val = val/ w;

 //float a  = 0;
 //float b = 1;
 //float c = 0.43;
 //float d = 0.57;
 //
 //
 val = 0.5 + 0.5 * val ;
 //val = (val - c)*((b-a)/(d-c)) + a;
 val = val > 1 ? 1 : val;
 total_value += val;
 quantizied_level =  roundf(val*L);
Pdf[quantizied_level]++;
 actual_point = _mesh.point(*mesh_vertex_iterator);
 shader_param = shader_parameters_data_wrapper[mesh_vertex_iterator];
 shader_param.AddParameter(ShaderParametersEnum::LIC,val);

 //std::cout<<"val "<<shader_param->getValue(5)<<std::endl;
 shader_parameters_data_wrapper[mesh_vertex_iterator] = shader_param;


 new_point_cloud->points[k].x = actual_point[0] ;
 new_point_cloud->points[k].y = actual_point[1] ;
 new_point_cloud->points[k].z = actual_point[2] ;
 new_point_cloud->points[k].intensity = val;

   }


   //Contrast Enhanchement

   float a  = 0;
   float b = 1;
   float c=0; //= 0.43;
   float d=1; //= 0.57;
   k = 0;
   float value_counter = 0;

   for(int i = 0;i<L;i++)
   {
       value_counter += Pdf[i];
       if ( value_counter < num_vertices * 5 /100  )
           if(i!=0)
               c =(float) (i-1)/L;
           else
               c= (float) i/L;
       if ( value_counter > num_vertices* 95 /100  )
       {
           if(i!=num_vertices)
               d =(float) (i+1)/L;
           else
               d= (float) i/L;
           break;
       }
   }

   mesh_vertex_iterator=_mesh.vertices_begin();
   shader_parameters_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, ShaderParameters*>(_mesh, "shader_parameters");



      for(;mesh_vertex_iterator!= vertex_iterator_end;++mesh_vertex_iterator)
      {
          //shader_param = shader_parameters_data_wrapper[mesh_vertex_iterator];

          shader_wrapper = shader_parameters_data_wrapper[mesh_vertex_iterator];
          std::vector<ShaderParameters> list;
          shader_wrapper->GetListOfShaders(list);
           float val;
          for(ShaderParameters sp : list)
          {
                shader_param.GetParameter(ShaderParametersEnum::LIC,val);
          }


        val = (val - c)*((b-a)/(d-c)) + a;

        shader_param->setValue(5,val);
        shader_parameters_data_wrapper[mesh_vertex_iterator] = shader_param;
        new_point_cloud->points[k].intensity = val;
      }

      shader_parameters_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, ShaderParameters*>(_mesh, "shader_parameters");


std::cout<<" c " << c << " d "<< d << std::endl;
box_length /= 2 ;
std::fill(Pdf.begin(), Pdf.end(), 0);
   //second pass
  std::cout<<"------SECOND PASS---------"<<std::endl;
    mesh_vertex_iterator=_mesh.vertices_begin();
    k = 0;
   for(;mesh_vertex_iterator!= vertex_iterator_end;++mesh_vertex_iterator,k++)
   {
    if(k%percentual_part == 0)
        std::cout<< "LIC map "<<k / percentual_part<< " %  completed"<< std::endl;

       //fw advection
float val = 0;
int w = 0;



 for(int j=0;j<2;j++)
 {
     float samepoint_iter = 1;
    actual_point = _mesh.point(*mesh_vertex_iterator);

//      shader_param = shader_parameters_data_wrapper[mesh_vertex_iterator];
      shader_wrapper = shader_parameters_data_wrapper[mesh_vertex_iterator];
      std::vector<ShaderParameters> list;
      shader_wrapper->GetListOfShaders(list);
      for(ShaderParameters sp : list)
      {
          sp.GetParameter(ShaderParametersEnum::flow_normal,flow_vector);

      }
      //previously
      //if(shader_param == nullptr)
      //    std::cout<<"Err shader param is empty"<<std::endl;
      //else
      //  flow_vector = glm::vec3( shader_param->getValue(1),shader_param->getValue(2),shader_param->getValue(3));

     vvit = _mesh.vv_iter(mesh_vertex_iterator);
     // = _mesh.vv_iter(*mesh_vertex_iterator);

        for(float i = 0;i<box_length;i++)
            {
                if(j==1)
                    flow_vector = -flow_vector;
               //  MyMesh::VertexVertexIter vvit = _mesh.vv_iter(next_point);
                 actual_point[0] += flow_vector[0]*step_size;
                 actual_point[1] += flow_vector[1]*step_size;
                 actual_point[2] += flow_vector[2]*step_size;

                 dist = 0;
                 min_dist = INFINITY;
                 if(i!=0 && samepoint_iter == 1)
                     vvit = _mesh.vv_iter(vvit);
                 else
                      vvit = _mesh.vv_iter(mesh_vertex_iterator);
                 for ( ;vvit.is_valid(); ++vvit)
                 {
                     circulator_point = _mesh.point(*vvit);
                    offset = actual_point - circulator_point;
                     dist = dot(offset,offset);
                    if(dist<min_dist)
                    {
                        min_dist = dist;
                        next_point = vvit;
                    }
                 }
//                 float n =0;// shader_param->getValue(5);
                 float n = noise.GetPerlin(actual_point[0] * frequency ,
                                           actual_point[1] * frequency,
                                           actual_point[2] * frequency);
           //     if(n<0 || n>1)
           //         std::cout<<"Err in getVal"<< n <<std::endl;
//                 if( next_point.is_valid())
//                    actual_point = _mesh.point(*next_point);

                 val+=n;
                 w++;
                 if( next_point.is_valid())
                 {
                     //shader_param = shader_parameters_data_wrapper[next_point];
                     shader_wrapper = shader_parameters_data_wrapper[next_point];
                     std::vector<ShaderParameters> list;
                     shader_wrapper->GetListOfShaders(list);
                     glm::vec3 next_flow_vector;
                      for(ShaderParameters sp : list)
                     {
                           shader_param.GetParameter(ShaderParametersEnum::flow_normal,next_flow_vector);
                     }
                     // = glm::vec3( shader_param->getValue(1),shader_param->getValue(2),shader_param->getValue(3));
                     //flow_vector = flow_vector* (1-1/(min_dist+1)) + next_flow_vector*(1-min_dist/(min_dist+1));
                     flow_vector = next_flow_vector;
                     //vvit  = _mesh.vv_iter(next_point);
                     vvit = next_point;
                     samepoint_iter = 1;
                  }
                  else
                  {
                     samepoint_iter++;
                  }

            }
    }

 val = val/ w;
 val = 0.5 + 0.5 * val ;
 if(val<0 || val>1)
     std::cout<<"Err in Val"<< val <<std::endl;
 val = val > 1 ? 1 : val;
 quantizied_level =  roundf(val*L);
Pdf[quantizied_level]++;
 actual_point = _mesh.point(*mesh_vertex_iterator);
 shader_wrapper = shader_parameters_data_wrapper[mesh_vertex_iterator];
 shader_param->setValue(5,val);

//shader_parameters_data_wrapper[mesh_vertex_iterator] = shader_param;

   }
   value_counter = 0;

   for(int i = 0;i<L;i++)
   {
       value_counter += Pdf[i];
       if ( value_counter < num_vertices * 2 /100  )
           if(i!=0)
               c =(float) (i-1)/L;
           else
               c= (float) i/L;
       if ( value_counter > num_vertices* 98 /100  )
       {
           if(i!=num_vertices)
               d =(float) (i+1)/L;
           else
               d= (float) i/L;
           break;
       }
   }

   mesh_vertex_iterator=_mesh.vertices_begin();
   k = 0;
     for(;mesh_vertex_iterator!= vertex_iterator_end;++mesh_vertex_iterator,k++)
      {
          shader_param = shader_parameters_data_wrapper[mesh_vertex_iterator];
        float val = shader_param->getValue(5);
        val = (val - c)*((b-a)/(d-c)) + a;
        shader_param->setValue(5,val);

      }



   Outputcloud = new_point_cloud;
   kdtree_output.setInputCloud (Outputcloud);

}
*/

map<MyMesh::VertexHandle,glm::vec3> FlowClassifier::selectFlowVertices()
{
    MyMesh::VertexIter vertex_iterator,vertex_iterator_end(_mesh.vertices_end());
    map<MyMesh::VertexHandle,glm::vec3> flow_vertices;
    for(vertex_iterator=_mesh.vertices_begin();vertex_iterator != vertex_iterator_end;++vertex_iterator)
    {
        SimulationData* sd = simulation_data_wrapper[*vertex_iterator];
        //normalize it!
        glm::vec3 normal;
        sd->getData(SimulationDataEnum::flow_normal,normal);
        if(normal[0] != 0 || normal[1] != 0 || normal[2] != 0)
        {
            flow_vertices.insert(make_pair(*vertex_iterator,normal));
        }
    }
    return flow_vertices;
}
/*
void FlowClassifier::LIC2(float box_length,float frequency,float step_size,MyMesh mesh)
{
    _mesh = mesh;

    FastNoise noise;
   pcl::PointCloud<pcl::PointXYZI>::Ptr new_point_cloud ( new pcl::PointCloud<pcl::PointXYZI>);

SubdividerAndInterpolator<MyMesh> catmull;


if(_subdiv_levels>0)
{
    catmull.attach(_mesh);
    catmull(_subdiv_levels);
    catmull.detach();

}

auto shader_parameters_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, ShadersWrapper*>(_mesh, "shader_parameters");


std::cout<<"InterpolationDone"<<std::endl;
 new_point_cloud->width = _mesh.n_vertices();//numpoints
 new_point_cloud->height = 1;
 new_point_cloud->points.resize (new_point_cloud->width * new_point_cloud->height);
MyMesh::Point actual_point;
MyMesh::Point circulator_point;
MyMesh::Point offset;
glm::vec3 flow_vector(0,0,0);
MyMesh::VertexVertexIter vvit;
MyMesh::VertexVertexIter next_point;
ShaderParameters  shader_param;
ShadersWrapper*  shader_wrapper;

float dist;
float min_dist = INFINITY;
int k = 0;
int L = 1000;
int quantizied_level;
std::vector<int> Pdf(L,0) ;
std::vector<int> Cdf(L,0) ;
//InterpolateData(_mesh);
float cdf_min = INFINITY;
MyMesh::VertexIter mesh_vertex_iterator;
MyMesh::VertexIter vertex_iterator_end(_mesh.vertices_end());


int num_vertices = _mesh.n_vertices();
int percentual_part = num_vertices/100;
k = 0;
float total_value = 0;
mesh_vertex_iterator=_mesh.vertices_begin();
   for(;mesh_vertex_iterator!= vertex_iterator_end;++mesh_vertex_iterator,k++)
   {
    if(k%percentual_part == 0)
        std::cout<< "LIC map "<<k / percentual_part<< " %  completed"<< std::endl;

       //fw advection
float val = 0;
int w = 0;

 for(int j=0;j<2;j++)
 {
     float samepoint_iter = 1;
     actual_point = _mesh.point(*mesh_vertex_iterator);

      shader_wrapper = shader_parameters_data_wrapper[mesh_vertex_iterator];
      std::vector<ShaderParameters> list;
      shader_wrapper->GetListOfShaders(list);
      for(ShaderParameters sp : list)
      {
          sp.GetParameter(ShaderParametersEnum::flow_normal,flow_vector);

      }
     // if(shader_param == nullptr)
     //     std::cout<<"Err shader param is empty"<<std::endl;
     // else
     //   flow_vector = glm::vec3( shader_param->getValue(1),shader_param->getValue(2),shader_param->getValue(3));

     vvit = _mesh.vv_iter(mesh_vertex_iterator);
     // = _mesh.vv_iter(*mesh_vertex_iterator);

        for(float i = 0;i<box_length;i++)
            {
                if(j==1)
                    flow_vector = -flow_vector;
               //  MyMesh::VertexVertexIter vvit = _mesh.vv_iter(next_point);
                 actual_point[0] += flow_vector[0]*step_size;
                 actual_point[1] += flow_vector[1]*step_size;
                 actual_point[2] += flow_vector[2]*step_size;

                 dist = 0;
                 min_dist = INFINITY;
                 if(i!=0 && samepoint_iter == 1)
                     vvit = _mesh.vv_iter(vvit);
                 else
                      vvit = _mesh.vv_iter(mesh_vertex_iterator);
                 for ( ;vvit.is_valid(); ++vvit)
                 {
                     circulator_point = _mesh.point(*vvit);
                    offset = actual_point - circulator_point;
                     dist = dot(offset,offset);
                    if(dist<min_dist)
                    {
                        min_dist = dist;
                        next_point = vvit;
                    }
                 }
                 float n = noise.GetPerlin(actual_point[0] * frequency ,
                                           actual_point[1] * frequency,
                                           actual_point[2] * frequency);


//                 if( next_point.is_valid())
//                    actual_point = _mesh.point(*next_point);

                 val+=n;
                 w++;
                 if( next_point.is_valid())
                 {
                     glm::vec3 next_flow_vector ;
                     shader_wrapper = shader_parameters_data_wrapper[mesh_vertex_iterator];
                     shader_wrapper->GetListOfShaders(list);
                     for(ShaderParameters sp : list)
                     {
                         sp.GetParameter(ShaderParametersEnum::flow_normal,next_flow_vector);

                     }


                  //   shader_param = shader_parameters_data_wrapper[next_point];
                  //   glm::vec3 next_flow_vector = glm::vec3( shader_param->getValue(1),shader_param->getValue(2),shader_param->getValue(3));
//                     flow_vector = flow_vector* (1-1/(min_dist+1)) + next_flow_vector*(1-min_dist/(min_dist+1));
                     flow_vector = next_flow_vector;
                     //vvit  = _mesh.vv_iter(next_point);
                     vvit = next_point;
                    samepoint_iter = 1;
                 }
                 else
                 {
                    samepoint_iter++;
                 }
            }
    }
 val = val/ w;

 //float a  = 0;
 //float b = 1;
 //float c = 0.43;
 //float d = 0.57;
 //
 //
 val = 0.5 + 0.5 * val ;
 //val = (val - c)*((b-a)/(d-c)) + a;
 val = val > 1 ? 1 : val;
 total_value += val;
 quantizied_level =  roundf(val*L);
Pdf[quantizied_level]++;
 actual_point = _mesh.point(*mesh_vertex_iterator);
 shader_param = shader_parameters_data_wrapper[mesh_vertex_iterator];
 shader_param.AddParameter(ShaderParametersEnum::LIC,val);

 //std::cout<<"val "<<shader_param->getValue(5)<<std::endl;
 shader_parameters_data_wrapper[mesh_vertex_iterator] = shader_param;


 new_point_cloud->points[k].x = actual_point[0] ;
 new_point_cloud->points[k].y = actual_point[1] ;
 new_point_cloud->points[k].z = actual_point[2] ;
 new_point_cloud->points[k].intensity = val;
}}

*/
