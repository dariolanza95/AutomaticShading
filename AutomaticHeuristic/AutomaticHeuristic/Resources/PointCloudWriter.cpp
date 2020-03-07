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
    char* _file_name;
    _file_name = fromStringToChar(_output_file_name);
    char *var_types[_num_shader_parameters];
    char *var_names[_num_shader_parameters];
    for(int i = 0; i<_num_shader_parameters;i++)
    {
        var_types[i] = fromStringToChar("float");
        std::string var_name("shader_parameter_");
        var_name+=std::to_string(i);
        var_names[i] = fromStringToChar(var_name);
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
     _output_file = PtcCreatePointCloudFile (_file_name,_num_shader_parameters, var_types,var_names,
                                                          world_to_eye, world_to_ndc,format);
    if(_output_file == NULL )
    {
        std::cout<<"Err in the creation of the file... exiting";
        exit(1);
    }

}

void PointCloudWriter::Write()
{
    auto shader_parameters_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, ShaderParameters*>(_mesh, "shader_parameters");
    int i=0;
    float point[3], normal[3];
    float radius = 0.02;
    MyMesh::Point mesh_point;
    float* data = (float *) malloc(_num_shader_parameters * sizeof(float));

    MyMesh::VertexIter vertex_handle;
    MyMesh::VertexIter vertex_iterator_end(_mesh.vertices_end());
    for(vertex_handle=_mesh.vertices_begin();vertex_handle!= vertex_iterator_end;++vertex_handle)
    {
        ShaderParameters* const shader_param = shader_parameters_data_wrapper[vertex_handle];

        data[0] = shader_param->getId();
        for(int i = 0;i<_num_shader_parameters-1;i++)
        {
            data[i+1] = shader_param->getValue(i);
        }
        mesh_point =  _mesh.point(*vertex_handle);
        point[0] = mesh_point[0];
        point[1] = mesh_point[1];
        point[2] = mesh_point[2];
        normal[0] = normal[1] = normal[2] = 0;
        PtcWriteDataPoint(_output_file, point, normal, radius, data);

    }
    PtcClosePointCloudFile(_output_file);

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
            float val = data[1];
            std::cout<<"val is "<<val<<std::endl;
        }
        else
            std::cout<<"error"<<std::endl;


}


PointCloudWriter::PointCloudWriter(MyMesh mesh,std::string input_file_name,int num_shader_parameters):
    _mesh(mesh),
    _output_file_name(input_file_name),
    _num_shader_parameters(num_shader_parameters)
{}
