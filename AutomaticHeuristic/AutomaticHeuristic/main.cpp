/*#include <QCoreApplication>
#include <iostream>
#include <vector>

// -------------------- OpenMesh
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/BaseKernel.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyConnectivity.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;

using namespace std;
int main(int argc, char **argv)
{
  MyMesh  mesh;
  // check command line options
  if (argc != 4)
  {
    std::cerr << "Usage:  " << argv[0] << " #iterations infile outfile\n";
    return 1;
  }
  // read mesh from stdin
  if ( ! OpenMesh::IO::read_mesh(mesh, argv[2]) )
  {
    std::cerr << "Error: Cannot read mesh from " << argv[2] << std::endl;
    return 1;
  }
  // this vector stores the computed centers of gravity
  std::vector<MyMesh::Point>  cogs;
  std::vector<MyMesh::Point>::iterator cog_it;
  cogs.reserve(mesh.n_vertices());
  // smoothing mesh argv[1] times

  MyMesh::VertexIter          v_it, v_end(mesh.vertices_end());
  MyMesh::VertexVertexIter    vv_it;
  MyMesh::Point               cog;
  MyMesh::Scalar              valence;
  unsigned int                i, N(atoi(argv[1]));
  for (i=0; i < N; ++i)
  {
    cogs.clear();
    for (v_it=mesh.vertices_begin(); v_it!=v_end; ++v_it)
    {
      cog[0] = cog[1] = cog[2] = valence = 0.0;

      for (vv_it=mesh.vv_iter( *v_it ); vv_it.is_valid(); ++vv_it)
      {
        cog += mesh.point( *vv_it );
        ++valence;
      }
      cogs.push_back(cog / valence);
    }

    for (v_it=mesh.vertices_begin(), cog_it=cogs.begin();
         v_it!=v_end; ++v_it, ++cog_it)
      if ( !mesh.is_boundary( *v_it ) )
        mesh.set_point( *v_it, *cog_it );
  }
  // write mesh to stdout
  if ( ! OpenMesh::IO::write_mesh(mesh, argv[3]) )
  {
    std::cerr << "Error: cannot write mesh to " << argv[3] << std::endl;
    return 1;
  }
  return 0;
}*/
#include <iostream>
#include <sstream>
#include <fstream>
#include <string.h>
#include <math.h>
#include <stdlib.h>
// --------------------
#include <OpenMesh/Core/IO/MeshIO.hh>

#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include "dataloader.h"
#include "defaultdataloader.h"
typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;
using namespace std;
using namespace OpenMesh;
# define M_PI           3.14159265358979323846  /* pi */


vector<MyMesh::FaceHandle>  SelectFacesBasedOnSlope(MyMesh& mesh,float angle,float treshold)
{
    MyMesh::Face face;

    MyMesh::Normal up_direction = Vec3f(1,0,0);
    MyMesh::FaceIter face_iterator,face_iterator_end(mesh.faces_end());
    vector<MyMesh::FaceHandle> selected_faces ;
    int counter = 0;
    for(face_iterator=mesh.faces_begin();face_iterator != face_iterator_end;++face_iterator)
    {
        face = mesh.face(*face_iterator);
        MyMesh::Normal mynormal = mesh.normal(*face_iterator);

        float dot_result = dot(mynormal,up_direction);
        float resulting_angle_in_radians = acos(dot_result);
        float resulting_angle_in_degree = resulting_angle_in_radians* (180.0/  M_PI);
        resulting_angle_in_degree = 90 - resulting_angle_in_degree;
        resulting_angle_in_degree > 0 ? resulting_angle_in_degree:0;

        if(resulting_angle_in_degree <= angle + treshold && resulting_angle_in_degree >= angle - treshold)
        {
              cout<<"dot res "<< resulting_angle_in_degree <<endl;
              MyMesh::FaceHandle face_handle = face_iterator.handle();
              selected_faces.push_back(face_handle);
              counter++;
        }

    }
    return selected_faces;
}

void UpdateSimulationData(MyMesh& mesh, vector<MyMesh::FaceHandle> selected_faces)
{

    for(uint i = 0;i<selected_faces.size();i++)
    {

        MyMesh::FaceHandle face_handle = selected_faces[i];

        auto simulation_data = getOrMakeProperty<VertexHandle, float>(mesh, "simulation_data");
        MyMesh::FaceVertexIter face_vertex_circulator = mesh.fv_iter(face_handle);

        for(; face_vertex_circulator.is_valid(); ++face_vertex_circulator)
        {

            MyMesh::VertexHandle vertex_handle = face_vertex_circulator.handle();
            simulation_data[vertex_handle] = 3.14;
        }
    }
}

void FindTaluses(MyMesh& mesh)
{
    //angle of repose is usually between 33-37 degreee depending on the rock type
    float angle = 34;
    float treshold = 3;
     vector<MyMesh::FaceHandle> selected_faces;
     selected_faces = SelectFacesBasedOnSlope(mesh,angle,treshold);
     UpdateSimulationData(mesh,selected_faces);

}


void WriteSimulationDataOnOutputFile(MyMesh& mesh,ostream& outputfile)
{
    string line;
    auto simulation_data = getOrMakeProperty<VertexHandle, float>(mesh, "simulation_data");
    for (auto& vertex_handle : mesh.vertices())
    {

        outputfile<<simulation_data[vertex_handle] << endl;
    }
}
bool WriteSimulationData(string line,ifstream& myfile,ostream& omyfile,MyMesh& mesh)
{
    cout<<"writeSimData"<<endl;
    string newline;
    int counter = 0;
size_t found;
    while (counter <2)
    {
        found = line.find("]");
        if(found!=string::npos)
        {
            counter++;
            if(counter == 2)
                break;
        }
         omyfile << line<<'\n' ;
         if(!getline(myfile,line))
             break;

    }
    if (counter == 2)
    {
        string templine = line;
        line.erase(found + 1 ,line.length());
        omyfile << line<<'\n' ;
        string newstring = " \"varying float simulation_data\"[";
        omyfile<<newstring;
        WriteSimulationDataOnOutputFile(mesh,omyfile);
       /* while(getline(datafile, line))
        {
            newstring +=" " + line;
        }*/
        //newstring += " ] ";
        templine.erase(0,found +1);
        omyfile<<"]"<<templine<<'\n';
        return true;
          }
    return false;
}





void WriteVerticesDataOnOutputFile(MyMesh mesh,ofstream& outputfile)
{
     MyMesh::VertexIter v_it, v_end(mesh.vertices_end());
     MyMesh::Point p;
 //   if(! IO::write_mesh(mesh,outputfile,IO::Options));
    cout<<"num vertices ";
    for (v_it = mesh.vertices_sbegin() ; v_it!= v_end;++v_it )
    {
      //  cout<<"point " << mesh.point(*v_it)<<endl;
        outputfile<<" "<< mesh.point(*v_it)<<'\n';

    }
    cout<<mesh.n_vertices()<<endl;


}


bool WriteVerticesDataOnRibFile(MyMesh mesh,string& line,ifstream& myfile,ofstream& omyfile)
{

    string newline;

    size_t found = line.find("[");
    if (found != string::npos)
    {
       line.erase(found + 1 ,line.length());
        omyfile << line<<'\n' ;
        getline(myfile, line);
        found = line.find("]");
        while(  found == string::npos)
        {
            if(!getline(myfile, line))
                break;
            found = line.find("]");
        }
        if(found!=string::npos)
        {
            WriteVerticesDataOnOutputFile(mesh,omyfile);
            line.erase(0 ,found );

           // omyfile<<line<<'\n';
             return true;
        }

     }
        return false;
}

bool LoadMesh(MyMesh& mesh,string meshfile)
{

    OpenMesh::IO::Options opt;
    if ( ! OpenMesh::IO::read_mesh(mesh,meshfile, opt))
    {
      std::cerr << "Error loading mesh from file " << meshfile << std::endl;
      return false;
    }
    return true;
}

void CalculateNormals(MyMesh& mesh)
{
    OpenMesh::IO::Options opt;
    if ( !opt.check( OpenMesh::IO::Options::VertexNormal ) )
    {
        // we need face normals to update the vertex normals
        mesh.request_face_normals();

        // let the mesh update the normals
        mesh.update_normals();

    }
}

void Checks(int argc, char **argv,MyMesh mesh)
{
    if (argc!=2)
    {
      std::cerr << "Usage: " << argv[0] << " <input>\n";
      //return 1;
    }

    // request vertex normals, so the mesh reader can use normal information
    // if available
    mesh.request_vertex_normals();

    // assure we have vertex normals
    if (!mesh.has_vertex_normals())
    {
      std::cerr << "ERROR: Standard vertex property 'Normals' not available!\n";
      //return 1;
    }
}

void LoadGeometryData(string mesh_file,MyMesh& mesh)
{
    mesh.request_vertex_normals();
    if (!mesh.has_vertex_normals())
    {
      std::cerr << "ERROR: Standard vertex property 'Normals' not available!\n";
    }
    LoadMesh(mesh,mesh_file);
    CalculateNormals(mesh);
}



void AttachDataFromSimulationToEachVertex(string simulation_data_file,MyMesh &mesh)
{
    string line;
    int counter=0;
    ifstream inputfile  (simulation_data_file);
    auto simulation_data = getOrMakeProperty<VertexHandle, float>(mesh, "simulation_data");
    for (auto& vertex_handle : mesh.vertices())
    {
        getline(inputfile,line);
        counter++;
       // simulation_data[vertex_handle] = stof(line );
        simulation_data[vertex_handle] = 0;
    }
cout<<"total == "<< counter<<endl;
}

void WriteOnRibFile()
{
      string line;
      MyMesh mesh;

      char target[] = " \"vertex point P\"";
      char target2[] = "\"facevarying float[2] st\"";

      string obj_file = "../../Data/input.obj";
      string data_file = "../../Data/simulationData.txt";

      ifstream myfile ("../../Data/mountainsceneTemplate.rib");
      ifstream geometryfile ("../../Data/input.obj");
      ifstream datafile ("../../Data/simulationData.txt");
      ostringstream ss_newline;

//      DefaultDataLoader<MyMesh> dl = DefaultDataLoader<MyMesh>(obj_file,data_file,mesh);
      string newline ;
      newline = ss_newline.str();
      ostringstream ss_out_name_file;
      string out_name_file;
      ss_out_name_file << "../../Data/mountainsceneTemplate" << "Output.rib";
      out_name_file = ss_out_name_file.str();
      ofstream omyfile (out_name_file);
      size_t foundVertices,foundSimulationData;
      LoadGeometryData("../../Data/input.obj",mesh);
      AttachDataFromSimulationToEachVertex("../../Data/simulationData.txt",mesh);

      FindTaluses(mesh);

      if (myfile.is_open() && geometryfile.is_open() && datafile.is_open())
      {
        cout<<"files opened"<<endl;
        while ( getline(myfile, line))
        {
            foundVertices  = line.find(target);



            if (foundVertices != string::npos)
            {
                cout<<"found first targer\n";
                string templine = line;
                templine.erase(foundVertices,templine.length());
                omyfile<<templine;
                templine = line;
                line.erase(0,foundVertices);
                WriteVerticesDataOnRibFile(mesh,line,myfile,omyfile);

            }

            foundSimulationData  = line.find(target2);

            if (foundSimulationData != string::npos )
            {
                cout<<"found secondtarget\n";
                cout<<line<<endl;
                WriteSimulationData(line,myfile,omyfile,mesh);
            }
            else
            {
                  omyfile << line << '\n';
            }

         }

        geometryfile.close();
        myfile.close();
        omyfile.close();
        datafile.close();

      }
        else cout << "Unable to open file";


}




int main(int argc, char **argv)
{

  MyMesh mesh;
  //Checks();
  WriteOnRibFile();
  //LoadMesh(mesh);


  // If the file did not provide vertex normals, then calculate them
 //CalculateNormals();
/*
  // move all vertices one unit length along it's normal direction
 for (MyMesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
  {
    std::cout << "Vertex #" << v_it << ": " << mesh.point( v_it );
    mesh.set_point( v_it, mesh.point(v_it)+mesh.normal(v_it) );
    std::cout << " moved to " << mesh.point( v_it ) << std::endl;
  }
*/
  // don't need the normals anymore? Remove them!
  mesh.release_vertex_normals();

  // just check if it really works
  if (mesh.has_vertex_normals())
  {
    std::cerr << "Ouch! ERROR! Shouldn't have any vertex normals anymore!\n";
    return 1;
  }

  return 0;
}

