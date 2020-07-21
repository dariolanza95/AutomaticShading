#include "PointCloudWriter.h"


void RIBPointCloudWriter::WritePointCloud(std::shared_ptr<AShader> shader)
{
    char* _file_name;
    int num_variables;
    std::vector<char*> var_types;
    std::vector<char*> var_names;

    std::vector<const char*> var_types_n;
    std::vector<const char*> var_names_n;

    std::string empty_string("");
    var_names_n.resize(1);
    var_names_n[0] = empty_string.c_str();// AutomaticShaders::Utils::fromStringToChar("");
    var_types_n.resize(1);
    var_types_n[0] = empty_string.c_str();//AutomaticShaders::Utils::fromStringToChar("");
    std::string output_file_name;
    //TODO move this logic from here to WritePointcloud
    getCloudPathNameInterface(output_file_name,shader);
    //shader->getCloudPathNameInterface(output_file_name);
    //list_of_used_shaders->getCloudPathNameInterface(output_file_name);
    _output_file_name= _output_path + output_file_name;
    _file_name = AutomaticShaders::Utils::fromStringToChar(_output_file_name);
  //  shader->getSerializedTypesInterface(var_types,var_names,num_variables);
   getSerializedTypesInterface(var_types,var_names,num_variables,shader);
    float world_to_eye [16];
    float world_to_ndc [16];
    for(int i = 0;i<16;i++)
    {
        world_to_eye[i] = world_to_ndc[i] = 0;
    }

    float format[3];
    format[0] = 600;
    format[1] = 400;
    format[2] = 1.3;
    int tt = 0;
        _output_file = PtcCreateOrgPointCloudFile( _file_name,num_variables, &var_types[0],&var_names[0],tt,
                &var_types_n[0],&var_names_n[0], world_to_eye, world_to_ndc,format);
        //_output_file = PtcCreatePointCloudFile( _file_name,num_variables, &var_types[0],&var_names[0],
       //                                                       world_to_eye, world_to_ndc,format);

    if(_output_file == NULL )
    {
        std::cout<<"Err in the creation of the file... exiting";
        exit(1);
    }
    float point[3], normal[3];
    float radius = 0.1f;
 //   if(_writing_a_shader_mask){
 //       allocated_data.resize(1);
 //   }
    //else{
    //    _shader->allocateData(allocated_data);
    //}
  //   MyMesh::VertexIter vertex_handle;
    bool found = 0;
  //  vertex_handle=_mesh.vertices_begin();
//pcl::PointCloud<pcl::PointXYZL>::Ptr const point_cloud =_features_finder.getPointClouds();
//std::vector<std::shared_ptr<ShadersWrapper>> list_of_shader_wrapper = _features_finder.getListOfShadersWrapper();
size_t iterations = list_of_shaders_wrapper.size();

for(size_t j = 0;j<iterations;j++){

    std::shared_ptr<ShadersWrapper> shader_wrapper  = list_of_shaders_wrapper[j];
        std::vector<std::shared_ptr<AShader>> list;
        shader_wrapper->GetListOfShaders(list);
        for(std::shared_ptr<AShader> sp : list)
        {
            if(sp==nullptr)
                continue;
            if(sp->GetId()==shader->GetId()){
               found = true;
               // if(_writing_a_shader_mask){
               //     float conf = sp->GetConfidence();
               //     allocated_data[0] = conf;
               // }else
               {
                           getSerializedDataInterface(allocated_data,sp);
                //    sp->getSerializedDataInterface(allocated_data);
                    //sp->getSerializedData(allocated_data);
                }
            }

        }
        if(found){

            point[0] = list_of_used_points.at(j).x;//point_cloud->points[j].x;
            point[1] = list_of_used_points.at(j).y;//point_cloud->points[j].y;
            point[2] = list_of_used_points.at(j).z;//point_cloud->points[j].z;
            normal[0] = normal[1] = normal[2] = 0;
            PtcWriteDataPoint(_output_file, point, normal, radius,&allocated_data[0] );

        }
        found = false;
    }

    PtcClosePointCloudFile(_output_file);
}

//Useful debug function too check inside of a PointCloud file

void RIBPointCloudWriter::Read()
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


char* RIBPointCloudWriter::CreateMaskFile(std::shared_ptr<AShader> shader, std::vector<char*>& var_types, std::vector<char*>& var_names, int& num_variables)
{
    char* _file_name;
    std::string output_file_name;
    //shader->getCloudPathName(output_file_name);
    shader->getCloudPathNameInterface(output_file_name);
    _output_file_name=_output_path + output_file_name+"_mask";
    _file_name = AutomaticShaders::Utils::fromStringToChar(_output_file_name);
    var_names.resize(1);
    var_types.resize(1);
    var_types[0] = AutomaticShaders::Utils::fromStringToChar("float");
    var_names[0] = AutomaticShaders::Utils::fromStringToChar("mask");
    num_variables = 1;
    return _file_name;
}
RIBPointCloudWriter::RIBPointCloudWriter(MyMesh mesh,
                                         std::string output_path,
         std::vector<shared_ptr<AShader> > list_of_used_shaders,
         std::vector<shared_ptr<ShadersWrapper>> list_of_shaders_wrapper,
         std::vector<pcl::PointXYZL,Eigen::aligned_allocator<pcl::PointXYZL>>  list_of_used_points
         ): APointCloudWriter(list_of_used_shaders,list_of_shaders_wrapper,list_of_used_points),_output_path(output_path)

    {
   /* char* _file_name;
    int num_variables;
    std::vector<char*> var_types;
    std::vector<char*> var_names;

    std::vector<const char*> var_types_n;
    std::vector<const char*> var_names_n;

    std::string empty_string("");
    var_names_n.resize(1);
    var_names_n[0] = empty_string.c_str();// AutomaticShaders::Utils::fromStringToChar("");
    var_types_n.resize(1);
    var_types_n[0] = empty_string.c_str();//AutomaticShaders::Utils::fromStringToChar("");
    std::string output_file_name;
    //TODO move this logic from here to WritePointcloud
    list_of_used_shaders->getCloudPathNameInterface(output_file_name);
    _output_file_name= _output_path + output_file_name;
    _file_name = AutomaticShaders::Utils::fromStringToChar(_output_file_name);
    _shader->getSerializedTypesInterface(var_types,var_names,num_variables);

    float world_to_eye [16];
    float world_to_ndc [16];
    for(int i = 0;i<16;i++)
    {
        world_to_eye[i] = world_to_ndc[i] = 0;
    }

    float format[3];
    format[0] = 600;
    format[1] = 400;
    format[2] = 1.3;
    int tt = 0;
        _output_file = PtcCreateOrgPointCloudFile( _file_name,num_variables, &var_types[0],&var_names[0],tt,
                &var_types_n[0],&var_names_n[0],
                                                              world_to_eye, world_to_ndc,format);
    if(_output_file == NULL )
    {
        std::cout<<"Err in the creation of the file... exiting";
        exit(1);
    }*/
}


/*

RIBPointCloudWriter::RIBPointCloudWriter(MyMesh mesh, std::shared_ptr<AShader> shader, int subdiv_levels, std::string output_path, FeaturesFinder features_finder, bool mask=true):
    _mesh(mesh),
    _shader(shader),
    _subdiv_levels(subdiv_levels),
    _writing_a_shader_mask(mask),
   _output_path(output_path),
   _features_finder(features_finder)
{

    char* _file_name;
    int num_variables;
    std::vector<char*> var_types;
    std::vector<char*> var_names;

    std::vector<const char*> var_types_n;
    std::vector<const char*> var_names_n;

    var_names_n.resize(1);
    var_names_n[0] = AutomaticShaders::Utils::fromStringToChar("");
    var_types_n.resize(1);
    var_types_n[0] = AutomaticShaders::Utils::fromStringToChar("");

   // if(mask){
   //     _file_name = CreateMaskFile(shader,var_types,var_names,num_variables);
   //
   // }
   // else{
        std::string output_file_name;
        //shader->getCloudPathName(output_file_name);
        shader->getCloudPathNameInterface(output_file_name);
        _output_file_name= _output_path + output_file_name;
        _file_name = AutomaticShaders::Utils::fromStringToChar(_output_file_name);

        _shader->getSerializedTypesInterface(var_types,var_names,num_variables);
//        _shader->getSerializedTypes(var_types,var_names,num_variables);
    //}

    float world_to_eye [16];
    float world_to_ndc [16];
    for(int i = 0;i<16;i++)
    {
        world_to_eye[i] = world_to_ndc[i] = 0;
    }


    float format[3];
    format[0] = 600;
    format[1] = 400;
    format[2] = 1.3;
    int tt = 0;
   // if(mask){
        _output_file = PtcCreateOrgPointCloudFile( _file_name,num_variables, &var_types[0],&var_names[0],tt,
                &var_types_n[0],&var_names_n[0],
                                                              world_to_eye, world_to_ndc,format);

//    }else
//
//    _output_file = PtcCreatePointCloudFile (_file_name,num_variables, &var_types[0],&var_names[0],
                                                   //       world_to_eye, world_to_ndc,format);
    if(_output_file == NULL )
    {
        std::cout<<"Err in the creation of the file... exiting";
        exit(1);
    }

}
*/
