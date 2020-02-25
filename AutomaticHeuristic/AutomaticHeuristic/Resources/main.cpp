#include <iostream>
#include <sstream>
#include <fstream>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "aclassifier.h"
#include "riverclassifier.h"
#include "riverclassifiertester.h"
#include "screeclassifier.h"
#include "simulationdata.h"
#include "ribwriter.h"
#include "ShaderParameters.h"
#include "./Graphics/openglvisualizer.h"
#include "featuresfinder.h"

// --------------------OpenMesh----------------------------
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>

//-----------------------OpenGL----------------------------
#include <glm/glm.hpp>
#include <GL/glew.h>
#include <GL/glx.h>
#include <GLFW/glfw3.h>

typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;

using namespace std;
using namespace OpenMesh;
# define M_PI           3.14159265358979323846  /* pi */


#include <iostream>
#include <string>

#include <Field3D/DenseField.h>
#include <Field3D/InitIO.h>
#include <Field3D/Field3DFile.h>

//----------------------------------------------------------------------------//

using namespace std;

using namespace Field3D;




char* filetobuf(char *file)
{
    FILE *fptr;
    long length;
    char *buf;

    fptr = fopen(file, "rb"); /* Open file for reading */
    if (!fptr) /* Return NULL on failure */
        return NULL;
    fseek(fptr, 0, SEEK_END); /* Seek to the end of the file */
    length = ftell(fptr); /* Find out how many bytes into the file we are */
    buf = (char*)malloc(length+1); /* Allocate a buffer for the entire length of the file and a null terminator */
    fseek(fptr, 0, SEEK_SET); /* Go back to the beginning of the file */
    fread(buf, length, 1, fptr); /* Read the contents of the file in to the buffer */
    fclose(fptr); /* Close the file */
    buf[length] = 0; /* Null terminator */

    return buf; /* Return the buffer */
}

 void LoadGeometryData2(MyMesh mesh,GLfloat *mat ,GLfloat *colors )
{

   //GLfloat *colors ;
    int i=0;
  //  colors = new GLfloat[verticesNumber*3];
  //  mat = new GLfloat[verticesNumber*3];
    MyMesh::VertexIter v_it, v_end(mesh.vertices_end());
    MyMesh::Point p;
//   if(! IO::write_mesh(mesh,outputfile,IO::Options));
   cout<<"num vertices "<<mesh.n_vertices()<<endl;
   for (v_it = mesh.vertices_sbegin() ; v_it!= v_end;++v_it )
   {
       p = mesh.point(*v_it);
       colors[i] = 0.4;
       colors[i+1] = 0.4;
       colors[i+2] = 0.4;
       mat[i++] = p[0];
       mat[i++] = p[1];
       mat[i++] = p[2];
       cout<<i<<endl;

    }

}




void LoadGeometryDataIntoOpenGLBuffer(MyMesh mesh)
{
  GLuint vao, vbo[2];
    // GLfloat diamond[4][3] = {
    //{  0.0,  1.0 , 0.0  }, /* Top point */
    //{  1.0,  0.0 , 0.0  }, /* Right point */
    //{  0.0, -1.0 , 0.0  }, /* Bottom point */
    //{ -1.0,  0.0 , 0.0  } }; /* Left point */
    int verticesNumber = 4;
    vector<vector<GLfloat>> buffer;
    std::vector< std::vector<int> > a;

GLfloat *mat ;
    mat = new GLfloat[verticesNumber*3];
    //GLfloat **mat = (GLfloat **)malloc(verticesNumber * sizeof(GLfloat*));
    //for(int i = 0; i < verticesNumber; i++) mat[i] = (GLfloat *)malloc(3 * sizeof(GLfloat));

    for(int i = 0;i<verticesNumber*3;i+=3)
    {
        switch (i) {
        case 0:
            cout<<"case 0"<<endl;
            mat[i] = 0.0;
            mat[i+1] = 1.0;
            mat[i+2] = 0.0;
            break;
        case 3:
            cout<<"case 1"<<endl;
            mat[i]= 1.0;
            mat[i+1]= 0.0;
            mat[i+2]= 0.0;

            break;
        case 6:
            cout<<"case 2"<<endl;
            mat[i] = 0.0;
            mat[i+1] = -1.0;
            mat[i+2] = 0.0;

            break;
        case 9:
            cout<<"case 3"<<endl;
            mat[i] = -1.0;
            mat[i+1] = 0.0;
            mat[i+2] = 0.0;

            break;
        default:
            cout<<"err this point shouldnt be reached"<<endl;
            break;
        }
    }
    buffer.resize(verticesNumber);
    for(int i = 0;i<verticesNumber;i++)
    {
        buffer[i].resize(3);
        switch (i) {
        case 0:
            cout<<"case 0"<<endl;
            buffer[i][0] = 0.0;
            buffer[i][1] = 1.0;
            buffer[i][2] = 0.0;
            break;
        case 1:
            cout<<"case 1"<<endl;
            buffer[i][0] = 1.0;
            buffer[i][1] = 0.0;
            buffer[i][2] = 0.0;

            break;
        case 2:
            cout<<"case 2"<<endl;
            buffer[i][0] = 0.0;
            buffer[i][1] = -1.0;
            buffer[i][2] = 0.0;

            break;
        case 3:
            cout<<"case 3"<<endl;
            buffer[i][0] = -1.0;
            buffer[i][1] = 0.0;
            buffer[i][2] = 0.0;

            break;
        default:
            cout<<"err this point shouldnt be reached"<<endl;
            break;
        }
     }
  /*  for (int i = 0;i<verticesNumber;i++)
    {
        for(int j=0;j<3;j++)
        {
            cout << mat[i][j];
            cout<< " diamond "<<diamond[i][j]<< " / ";
        }
        cout << endl;
    }
*/
   // GLfloat *colors2;
   // GLfloat *mat ;
   // verticesNumber = mesh.n_vertices();
   // colors2 = new GLfloat[verticesNumber*3];
   // mat = new GLfloat[verticesNumber*3];
   //
   //  LoadGeometryData2(mesh,mat,colors2);

     const GLfloat colors[4][3] = {
    {  1.0,  0.0,  0.0  }, /* Red */
    {  0.0,  1.0,  0.0  }, /* Green */
    {  0.0,  0.0,  1.0  }, /* Blue */
    {  1.0,  1.0,  1.0  } }; /* White */



    for(int i = 0;i<verticesNumber;i+=3)
    {
        cout<<" x "<<mat[i]<< " y "<<mat[i+1]<<" z "<<mat[i+2]<<endl;
    }
       /* Allocate and assign a Vertex Array Object to our handle */
       glGenVertexArrays(1, &vao);

       /* Bind our Vertex Array Object as the current used object */
       glBindVertexArray(vao);

       /* Allocate and assign two Vertex Buffer Objects to our handle */
       glGenBuffers(2, vbo);

       /* Bind our first VBO as being the active buffer and storing vertex attributes (coordinates) */
       glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);
        verticesNumber = 4;
       /* Copy the vertex data from diamond to our buffer */
       /* 8 * sizeof(GLfloat) is the size of the diamond array, since it contains 8 GLfloat values */
       glBufferData(GL_ARRAY_BUFFER, verticesNumber*3*sizeof(GLfloat), mat, GL_STATIC_DRAW);
//8 * sizeof(GLfloat)
       GLenum err;
       while ((err = glGetError()) != GL_NO_ERROR) {
              cerr << "OpenGL error: " << err << endl;
          }

       /* Specify that our coordinate data is going into attribute index 0, and contains two floats per vertex */
       glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);

       /* Enable attribute index 0 as being used */
       glEnableVertexAttribArray(0);

       /* Bind our second VBO as being the active buffer and storing vertex attributes (colors) */
       glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);

       /* Copy the color data from colors to our buffer */
       /* 12 * sizeof(GLfloat) is the size of the colors array, since it contains 12 GLfloat values */
       glBufferData(GL_ARRAY_BUFFER, verticesNumber *3* sizeof(GLfloat), colors, GL_STATIC_DRAW);

       /* Specify that our color data is going into attribute index 1, and contains three floats per vertex */
       glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);

       /* Enable attribute index 1 as being used */
       glEnableVertexAttribArray(1);








    /*
 //const GLfloat diamond[4][2]
    //mesh.n_vertices()
    int verticesNumber = 3;
    vector<GLfloat> buffer = new vector<GLfloat>();
    buffer.resize(verticesNumber);
    for(int i = 0;i<verticesNumber;i++)
    {
        buffer[i].resize(3);
        switch (i) {
        case 0:
            buffer[i][0] = 0;
            buffer[i][1] = 0;
            buffer[i][2] = 0;
            break;
        case 1:
            buffer[i][0] = 1;
            buffer[i][1] = 0;
            buffer[i][2] = 0;
            break;
        case 2:
            buffer[i][0] = 0.5;
            buffer[i][1] = 1;
            buffer[i][2] = 0;
            break;
        default:
            break;
        }
     }
    buffer.

    const GLfloat diamond[4][2] = {
       {  0.0,  1.0  }, /* Top point */
  //     {  1.0,  0.0  }, /* Right point */
  //     {  0.0, -1.0  }, /* Bottom point */
//       { -1.0,  0.0  } };


   /* MyMesh::VertexIter vertex_iterator= mesh.fv_iter(face_handle);

    for(; face_vertex_circulator.is_valid(); ++face_vertex_circulator)
    {

        MyMesh::VertexHandle vertex_handle = face_vertex_circulator.handle();
        simulation_data[vertex_handle] = 3.14;
    }*/
}

int onWindowClose(GLFWwindow* window)
{
    if(glfwGetKey(window, GLFW_KEY_ESCAPE ) == GLFW_PRESS)
        return GLFW_TRUE;
    else
        return GLFW_FALSE;
}

void window_close_callback(GLFWwindow* window)
{

    glfwSetWindowShouldClose(window, onWindowClose(window));
 //   if (!time_to_close)
 //       glfwSetWindowShouldClose(window, GLFW_FALSE);
}


 GLFWwindow* OpenGLInit()
{
    // init GLFW
    if (!glfwInit())
    {
        std::cerr << "[GLFW] Error initialising GLFW" << std::endl;
        exit(1);
    }


    glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
    // Use OpenGL Core v3.2
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    GLFWwindow* window = glfwCreateWindow(300, 200, "BlankWindow", NULL, NULL);
    // Open an OpenGL window
   // if( !glfwCreateWindow( windowWidth,windowHeight, 8,8,8,8,8,8, GLX_WINDOW ) )
    if (window == NULL )
    {
        std::cerr << "[GLFW] Error opening window" << std::endl;
        glfwTerminate();
        exit(1);
    }

    int major, minor, rev;

    glfwGetVersion(&major, &minor, &rev);

    fprintf(stdout, "OpenGL version recieved: %d.%d.%d\n", major, minor, rev);
    // Init Glew (OpenGL Extension Loading)

#if defined(__APPLE__) || defined(__MACH__)
    // Do nothing
#else
    // http://stackoverflow.com/questions/8302625/segmentation-fault-at-glgenvertexarrays-1-vao
    glewExperimental = GL_TRUE;

    glfwMakeContextCurrent(window);
    GLenum err = glewInit();

    if (GLEW_OK != err)
    {
        std::cerr << "[GLEW] Init failed: " << glewGetErrorString(err) << std::endl;
        glfwTerminate();
        exit(1);
    }
#endif
    // Start Simulation ////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////

    glfwSetWindowCloseCallback(window, window_close_callback);
    return window;
}

void UpdateSimulationData(MyMesh& mesh, map<MyMesh::VertexHandle,ShaderParameters*> selected_vertices,int shaderID)
{
    auto shader_parameters_property = getOrMakeProperty<VertexHandle, ShaderParameters*>(mesh, "shader_parameters");
    for (auto const& x : selected_vertices)
    {
        MyMesh::VertexHandle vertex_handle = x.first;
        ShaderParameters* shader_parameter = x.second;

        shader_parameters_property[vertex_handle] = shader_parameter;

    }
}
void InitializerSimulationData(MyMesh& mesh)
{
auto shader_parameters_data_wrapper= getOrMakeProperty<VertexHandle, ShaderParameters*>(mesh, "shader_parameters");


    MyMesh::VertexIter vertex_iterator;
    MyMesh::VertexIter vertex_iterator_end(mesh.vertices_end());
    for(vertex_iterator=mesh.vertices_begin();vertex_iterator != vertex_iterator_end;++vertex_iterator)
    {
            ShaderParameters* shader_parameter = new ShaderParameters(0,10);
            shader_parameter->setValue(0,-1);
            shader_parameters_data_wrapper[vertex_iterator] = shader_parameter;
    }
}
void FindFeatures(MyMesh& mesh)
{

    //angle of repose is usually between 33-37 degreee depending on the rock type
    float angle = 10;
    float treshold = 3;
     map<MyMesh::VertexHandle,ShaderParameters*> selected_faces;
     InitializerSimulationData(mesh);
     AClassifier *sc = new ScreeClassifier(mesh,angle,treshold);
     selected_faces = sc->ClassifyVertices();
     UpdateSimulationData(mesh,selected_faces,1);
     selected_faces.clear();
     AClassifier *rc = new RiverClassifier(mesh,75,20,5,34,-35);
     selected_faces = rc->ClassifyVertices();
     UpdateSimulationData(mesh,selected_faces,2);
     RiverClassifierTester rct;
     rct.Test();

}


//std::ostream &operator<< (std::ostream &out, const glm::vec3 &vec) {
//    out << vec.x << " " << vec.y << " "<< vec.z << endl;
//
//    return out;
//}


void WriteSimulationData(MyMesh& mesh,ostream& outputfile)
{
    auto shader_parameters_data_wrapper = getOrMakeProperty<VertexHandle, ShaderParameters*>(mesh, "shader_parameters");
    //varying
    string newstring = " \"varying float simulation_data\"[";
    outputfile<<newstring;
    for (auto& vertex_handle : mesh.vertices())
    {
        ShaderParameters* const shader_param = shader_parameters_data_wrapper[vertex_handle];
        outputfile<< shader_param->getId()<<".0"<< endl;
    }
    outputfile<< " ]";
    for(int i= 0;i<1;i++)
    {
        string newstring = " \"vertex float shader_property_";
        outputfile<<newstring;
        outputfile<<i;
        outputfile << "\" [";

//        newstring += i + ;

        for (auto& vertex_handle : mesh.vertices())
        {

            ShaderParameters* shader_param = shader_parameters_data_wrapper[vertex_handle];
            outputfile<< shader_param->getValue(i)<< endl;
        }
        //how it was before -> outputfile<< " ] \" ";
        outputfile<< " ]  ";
    }

    newstring = " \"vertex float[3] shader_vector";
    outputfile<<newstring;
    outputfile << "\" [";

    for (auto& vertex_handle : mesh.vertices())
    {
        ShaderParameters* shader_param = shader_parameters_data_wrapper[vertex_handle];
        //outputfile<< shader_param->getVector()<< endl;
    }
    outputfile<< " ]  ";
}

bool WriteSimulationData(string line,ifstream& myfile,ostream& omyfile,MyMesh& mesh)
{
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

        WriteSimulationData(mesh,omyfile);
       /* while(getline(datafile, line))
        {
            newstring +=" " + line;
        }*/
        //newstring += " ] ";
        templine.erase(0,found +1);
        omyfile<<templine<<'\n';
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
    LoadMesh(mesh,mesh_file);
    mesh.request_vertex_normals();
    if (!mesh.has_vertex_normals())
    {
      std::cerr << "ERROR: Standard vertex property 'Normals' not available!\n";
    }
    CalculateNormals(mesh);
}



void AttachDataFromSimulationToEachVertex(string simulation_data_file,MyMesh &mesh)
{
    string line;
    int counter=0;
    vector<string> variablenames = {"vegetation","rivers","normalFlow"};
    ifstream inputfile(simulation_data_file);
    auto simulation_data = getOrMakeProperty<VertexHandle, SimulationData*>(mesh, "simulation_data");
    for (auto& vertex_handle : mesh.vertices())
    {
        getline(inputfile,line);
        if(line.empty())
        {
            cout<<"WARNING!! Empty line in the file "<<endl;
            continue;
        }
        counter++;
        try
        {
            SimulationData *sd =new SimulationData(variablenames,line);
            simulation_data[vertex_handle] = sd;
        }
        catch(const char* excp)
        {
            cout<<excp<<endl;
            exit(EXIT_FAILURE);
        }
    }
cout<<"There are "<<mesh.n_vertices()<< " counter is == "<< counter<<endl;
}

MyMesh LoadMesh(string obj_file,string data_file)
{
    MyMesh mesh;

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
    LoadGeometryData("../../Data/input.obj",mesh);
    AttachDataFromSimulationToEachVertex("../../Data/simulationData.txt",mesh);



return mesh;
}

void WriteOnRibFile(MyMesh mesh)
{
      string line;
      char target[] = " \"vertex point P\"";
      char target2[] = "\"facevarying float[2] st\"";
      size_t foundVertices,foundSimulationData;

      ifstream myfile ("../../Data/mountainsceneTemplate.rib");
      ifstream geometryfile ("../../Data/input.obj");
      ifstream datafile ("../../Data/simulationData.txt");
      ostringstream ss_newline;
      ostringstream ss_out_name_file;

      string out_name_file;
      ss_out_name_file << "../../Data/mountainsceneTemplate" << "Output.rib";
      out_name_file = ss_out_name_file.str();
      ofstream omyfile (out_name_file);

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


  string obj_file = "../../Data/input.obj";
  string data_file = "../../Data/simulationData.txt";


  MyMesh mesh;
  mesh = LoadMesh(obj_file,data_file);
  FeaturesFinder features_finder(mesh);
  mesh = features_finder.Find();
  GLFWwindow* window = OpenGLInit();
  OpenGlVisualizer visualizer(window, 300, 300,mesh,obj_file);
  visualizer.Initialize();
  visualizer.Visualize();

  RIBWriter writer(mesh,"../../Data/mountainsceneTemplateOutput.rib",visualizer.GetCamera(),features_finder.GetVertexEditTags());
  writer.Write();
  //WriteOnRibFile(mesh);



  // Call initIO() to initialize standard I/O methods and load plugins
  Field3D::initIO();

  DenseField<float>::Ptr field(new DenseField<float>);
  field->name = "hello";
  field->attribute = "world";
  field->setSize(V3i(50, 50, 50));
  field->clear(1.0f);
  field->metadata().setStrMetadata("my_attribute", "my_value");
    std::cout<<"helooooo";
 // Field3DOutputFile out;
 // out.create("field3d_file.f3d");
 // out.writeScalarLayer<float>(field);
  return 0;
}

