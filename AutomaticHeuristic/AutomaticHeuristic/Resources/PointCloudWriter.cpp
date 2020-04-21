#include "PointCloudWriter.h"

char* fromStringToChar(std::string input)
{
    char* output;
    output = (char *) malloc((input.size()+1) * sizeof(char));
    input.copy(output, input.size() + 1);
    output[input.size()] = '\0';
    return output;
}
void PointCloudWriter::Init()
{


}

void PointCloudWriter::Write()
{
    auto shader_parameters_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, ShadersWrapper*>(_mesh, "shader_parameters");
    int i=0;
    float point[3], normal[3];
    float radius = 0.02;
    MyMesh::Point mesh_point;
    float* data;// = (float *) malloc(_num_shader_parameters * sizeof(float));



    //OpenMesh::Subdivider::Uniform::CatmullClarkT<MyMesh> catmull;
    // Execute 1 subdivision steps
    std::cout<<_mesh.n_vertices()<<std::endl;

    //iterate for number of desired subdivisions
    if(_subdiv_levels>0)
    {SubdividerAndInterpolator<MyMesh> catmull;

        catmull.attach(_mesh);
        catmull( _subdiv_levels );
        catmull.detach();
    }

int k = 0;
    std::cout<<"n vertices "<<_mesh.n_vertices()<<std::endl;
    MyMesh::VertexIter vertex_handle;
    MyMesh::VertexIter vertex_iterator_end(_mesh.vertices_end());
    vertex_handle=_mesh.vertices_begin();

    for(_mesh.vertices_begin();vertex_handle!= vertex_iterator_end;++vertex_handle)
    {
        ShadersWrapper* const shader_wrapper = shader_parameters_data_wrapper[vertex_handle];
        std::vector<AShader*> list;
        shader_wrapper->GetListOfShaders(list);
        for(AShader* sp : list)
        {
            if(sp->GetId()==_shader->GetId())
            //if(sp->GetId()==3)
                sp->getSerializedData(&data);
       // float hardness;
       // sp.GetParameter(ShaderParametersEnum::hardness,hardness);
       // glm::vec3 flow_vector;
       // sp.GetParameter(ShaderParametersEnum::flow_normal,flow_vector);
       // data[1] = hardness;
       // data[2] = flow_vector[0];
       // data[3] = flow_vector[1];
       // data[4] = flow_vector[2];
        }

/*
        if(shader_param!=nullptr)
        {
            data[0] = shader_param->GetId();
            for(int i = 0;i<5-1;i++)
            {
                data[i+1] = shader_param->getValue(i);
            }

        }

*/
        mesh_point =  _mesh.point(*vertex_handle);
        point[0] = mesh_point[0];
        point[1] = mesh_point[1];
        point[2] = mesh_point[2];
//        float flow_res;
//        flow_res = _licmap.GetPoint(point);
       // std::cout<<"FlowRes "<< flow_res<<std::endl;
        //THIS PART MUST BE CHANGED

        //for(int i= 0;i<3;i++)
        //{
        //    data[i+1] = flow_dir[i];
        //}
     //   data[5] = flow_res;
     //   if(flow_res < 0 || flow_res>1)
     //       std::cout<<"err in flow_res "<< flow_res << std::endl;
        normal[0] = normal[1] = normal[2] = 0;
        PtcWriteDataPoint(_output_file, point, normal, radius, data);
    k++;
    }
    std::cout<<"k == "<<k<<std::endl;
    PtcClosePointCloudFile(_output_file);
    //delete *data;
}

//Useful debug function too check inside of a PointCloud file

void PointCloudWriter::Read()
{

    PtcPointCloud inptc = PtcSafeOpenPointCloudFile(fromStringToChar( _output_file_name));
        if (!inptc) {
          fprintf(stderr, "ptmerge error: unable to open input file %s\n",
                  _output_file);
          exit(1);
        }
        int nvars;
        char **vartypes;
        char **varnames;
        PtcGetPointCloudInfo(inptc, "nvars", &nvars);
        PtcGetPointCloudInfo(inptc, "vartypes", &vartypes);
        PtcGetPointCloudInfo(inptc, "varnames", &varnames);
        float point[3];
        float normal[3];
        float radius;
        float *data;
        int npoints;
        int datasize;
          PtcGetPointCloudInfo(inptc, "npoints", &npoints);
        std::cout<<"nvars "<<nvars<< "var types "<<vartypes << std::endl << "var names "<<varnames;
        PtcGetPointCloudInfo(inptc, "datasize", &datasize);
           data = (float *) malloc(datasize * sizeof(float));
        for (int p = 0; p < npoints; p++) {
                  PtcReadDataPoint(inptc, point, normal, &radius, data);
                  std::cout<< "x "<< point[0]<< " y "<<point[1]<< " z " << point[2];
                  for(int i= 0;i<datasize;i++)
                  {
                    std::cout<<"sp_"<<i<< " "<<data[i];
                  }
                  std::cout<<std::endl;
              }
        point[0] = 299;
        point[1] = 299;
        point[2] = -9.9;
        normal[0] = normal[1] = normal[2] = 0;
        float maxdist = 1;
        int numpoints = 1;
        int res = PtcGetNearestPointsData (inptc, point, normal,maxdist, numpoints, data);
        if(res==1)
        {
            std::cout<<"point read ";
            float val = data[0];
            std::cout<<"val is "<<val<<std::endl;
        }
        else
            std::cout<<"error"<<std::endl;

    delete *vartypes;
    delete vartypes;

    delete *varnames;
    delete varnames;

}


PointCloudWriter::PointCloudWriter(MyMesh mesh,std::string input_file_name,AShader* shader,int subdiv_levels):
    _mesh(mesh),
    _output_file_name(input_file_name),
    _shader(shader),
    _subdiv_levels(subdiv_levels)
{

    char* _file_name;
    _file_name = fromStringToChar(_output_file_name);
//    char *var_types[_num_shader_parameters];
//    char *var_names[_num_shader_parameters];
    char** var_types;
    char** var_names;
    int num_variables;
    _shader->getSerializedTypes(&var_types,&var_names,&num_variables);
    //for(int i = 0; i<_num_shader_parameters;i++)
    //{
    //    var_types[i] = fromStringToChar("float");
    //    std::string var_name("shader_parameter_");
    //    var_name+=std::to_string(i);
    //    var_names[i] = fromStringToChar(var_name);
    //}
    float world_to_eye [16];
    float world_to_ndc [16];
    for(int i = 0;i<16;i++)
    {
        world_to_eye[i] = world_to_ndc[i] = 0;
    }

    for(int i = 0;i<4;i++)  {
        std::cout<<" "<<*var_types[i]<<std::endl;
        std::cout<<" "<<*var_names[i]<<std::endl;
        //*types[i] = new char[];
        //*types[i] = AutomaticShaders::Utils::fromStringToChar("float");
    }

    float format[3];
    format[0] = 10;
    format[1] = 10;
    format[2] = 1.3;
     _output_file = PtcCreatePointCloudFile (_file_name,num_variables, var_types,var_names,
                                                          world_to_eye, world_to_ndc,format);
    if(_output_file == NULL )
    {
        std::cout<<"Err in the creation of the file... exiting";
        exit(1);
    }

}
