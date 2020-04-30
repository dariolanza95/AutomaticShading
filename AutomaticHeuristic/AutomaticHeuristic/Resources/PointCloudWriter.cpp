#include "PointCloudWriter.h"


void PointCloudWriter::Write()
{
    auto shader_parameters_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, ShadersWrapper*>(_mesh, "shader_parameters");
    float point[3], normal[3];
    float radius = 0.02;
    MyMesh::Point mesh_point;
    if(_writing_a_shader_mask){
        allocated_data.resize(1);
    }else{
        _shader->allocateData(allocated_data);
    }
    std::cout<<_mesh.n_vertices()<<std::endl;
    std::cout<<"n vertices "<<_mesh.n_vertices()<<std::endl;
    MyMesh::VertexIter vertex_handle;
    MyMesh::VertexIter vertex_iterator_end(_mesh.vertices_end());
    bool found = 0;
    vertex_handle=_mesh.vertices_begin();
    for(_mesh.vertices_begin();vertex_handle!= vertex_iterator_end;++vertex_handle)
    {
        ShadersWrapper* const shader_wrapper = shader_parameters_data_wrapper[vertex_handle];
        std::vector<AShader*> list;
        shader_wrapper->GetListOfShaders(list);
        for(AShader* sp : list)
        {
            if(sp==nullptr)
                continue;
            if(sp->GetId()==_shader->GetId()){
               found = true;
                if(_writing_a_shader_mask){
                    float conf = sp->GetConfidence();
                    allocated_data[0] = conf;
                }else{
                    sp->getSerializedData(allocated_data);
                }
            }

        }
        if(found){
            mesh_point =  _mesh.point(*vertex_handle);
            point[0] = mesh_point[0];
            point[1] = mesh_point[1];
            point[2] = mesh_point[2];
            normal[0] = normal[1] = normal[2] = 0;
            PtcWriteDataPoint(_output_file, point, normal, radius,&allocated_data[0] );
            found = false;
        }
    }
    PtcClosePointCloudFile(_output_file);
}

//Useful debug function too check inside of a PointCloud file

void PointCloudWriter::Read()
{

    PtcPointCloud inptc = PtcSafeOpenPointCloudFile(AutomaticShaders::Utils::fromStringToChar( _output_file_name));
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


char* PointCloudWriter::CreateMaskFile(AShader* shader, std::vector<char*>& var_types, std::vector<char*>& var_names, int& num_variables)
{
    char* _file_name;
    std::string output_file_name;
    shader->getCloudPathName(output_file_name);
    _output_file_name=_output_path + output_file_name+"_mask";
    _file_name = AutomaticShaders::Utils::fromStringToChar(_output_file_name);
    var_names.resize(1);
    var_types.resize(1);
    var_types[0] = AutomaticShaders::Utils::fromStringToChar("float");
    var_names[0] = AutomaticShaders::Utils::fromStringToChar("mask");
    num_variables = 1;
    return _file_name;
}

PointCloudWriter::PointCloudWriter(MyMesh mesh,AShader* shader,int subdiv_levels,std::string output_path,bool mask=true):
    _mesh(mesh),
    _shader(shader),
    _subdiv_levels(subdiv_levels),
    _writing_a_shader_mask(mask),
   _output_path(output_path)
{

    char* _file_name;
    int num_variables;
    std::vector<char*> var_types;
    std::vector<char*> var_names;

    if(mask){
        _file_name = CreateMaskFile(shader,var_types,var_names,num_variables);
    }
    else{
        std::string output_file_name;
        shader->getCloudPathName(output_file_name);

        _output_file_name= _output_path + output_file_name;
        _file_name = AutomaticShaders::Utils::fromStringToChar(_output_file_name);
        _shader->getSerializedTypes(var_types,var_names,num_variables);
    }

    float world_to_eye [16];
    float world_to_ndc [16];
    for(int i = 0;i<16;i++)
    {
        world_to_eye[i] = world_to_ndc[i] = 0;
    }


    float format[3];
    format[0] = 10;
    format[1] = 10;
    format[2] = 1.3;


    _output_file = PtcCreatePointCloudFile (_file_name,num_variables, &var_types[0],&var_names[0],
                                                          world_to_eye, world_to_ndc,format);
    if(_output_file == NULL )
    {
        std::cout<<"Err in the creation of the file... exiting";
        exit(1);
    }

}
