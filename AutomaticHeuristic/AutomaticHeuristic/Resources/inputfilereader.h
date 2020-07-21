#ifndef INPUTFILEREADER_H
#define INPUTFILEREADER_H
#include <string>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include "mydefwrapper.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <string.h>

#include <stdlib.h>

class InputFileReader
{
    std::string obj_file_path;
    std::string data_file_path;
    MyMesh& mesh;
    SimulationDataMap& simulation_data_map;
    void LoadGeometryData();
    bool LoadMesh();

    void AttachDataFromSimulationToEachVertex();
public:
    InputFileReader(std::string obj_file_path,std::string data_file_path,MyMesh& mesh,SimulationDataMap& simulation_data_map);
    void ReadInputFiles();

};

#endif // INPUTFILEREADER_H
