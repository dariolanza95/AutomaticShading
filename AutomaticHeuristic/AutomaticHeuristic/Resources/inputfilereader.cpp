#include "inputfilereader.h"
InputFileReader::InputFileReader(std::string obj_file_path,
                                 std::string data_file_path,
                                 MyMesh& mesh,
                                 SimulationDataMap& simulation_data_map):
    obj_file_path(obj_file_path),
    data_file_path(data_file_path),
    mesh(mesh),
    simulation_data_map(simulation_data_map){}

void InputFileReader::ReadInputFiles(){
        LoadGeometryData();
        AttachDataFromSimulationToEachVertex();
}


bool InputFileReader::LoadMesh()
{

    OpenMesh::IO::Options opt;
    if ( ! OpenMesh::IO::read_mesh(mesh,obj_file_path, opt))
    {
      std::cerr << "Error loading mesh from file " << obj_file_path << std::endl;
      return false;
    }
    return true;
}


void InputFileReader::LoadGeometryData()
{
    LoadMesh();
    mesh.request_vertex_normals();
    if (!mesh.has_vertex_normals())
    {
      std::cerr << "ERROR: Standard vertex property 'Normals' not available!\n";
    }

    //Calculate normals
    OpenMesh::IO::Options opt;
    if ( !opt.check( OpenMesh::IO::Options::VertexNormal ) )
    {
        // we need face normals to update the vertex normals
        mesh.request_face_normals();

        // let the mesh update the normals
        mesh.update_normals();

    }
}

void InputFileReader::AttachDataFromSimulationToEachVertex()
{
    std::string line;
    int counter=0;

    std::ifstream inputfile(data_file_path);
    SimulationDataMap simulation_data_map_tmp;
    //    auto simulation_data = OpenMesh::getOrMakeProperty<MyMesh::VertexHandle,std::shared_ptr<SimulationData>>(mesh, "simulation_data");
    for (auto& vertex_handle : mesh.vertices())
    {
       std::getline(inputfile,line);
        if(line.empty())
        {
            std::cout<<"WARNING!! Empty line in the file "<<std::endl;
            continue;
        }
        counter++;
        try
        {
            std::shared_ptr<SimulationData> sd_uniq(new SimulationData(line));
            simulation_data_map_tmp.insert(std::make_pair(vertex_handle,sd_uniq));
       //     simulation_data[vertex_handle] = sd_uniq;

//            SimulationData *sd =new SimulationData(line);
//            simulation_data[vertex_handle] = sd;
        }
        catch(const char* excp)
        {

            std::cout<<excp<<std::endl;
            exit(EXIT_FAILURE);
        }
    }
    simulation_data_map = simulation_data_map_tmp;
}




