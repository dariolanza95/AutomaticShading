#include "openglvisualizer.h"
using namespace std;





OpenGlVisualizer::OpenGlVisualizer(GLFWwindow* window,int width,int height,MyMesh mesh,string obj_file):_cam(glm::vec3(0,0,0)),_mesh(mesh),_obj_file(obj_file)
{
    _grid_width = width;
    _grid_height = height;
    _window = window;
    glfwGetWindowSize(_window,&width,&height);

}

Camera OpenGlVisualizer::GetCamera(){return _cam;}


void OpenGlVisualizer::Initialize( )
{

//  InitializeOpengGL();
    InitializeBuffers();
}

void OpenGlVisualizer::Visualize()
{
    while((glfwGetKey(_window, GLFW_KEY_ESCAPE ) != GLFW_PRESS))
    {
    float dt = 1000.0/(60);
    glfwPollEvents();
    CameraMovement(dt);
    _shaderManager.Update();
    Render();
    }
}

void OpenGlVisualizer::Render()
{
    // Resize
    int w,h;
    bool resize = false;
    glfwGetWindowSize(_window,&w,&h);
    if (w != _width)
    {
        _width = w;
        resize = true;
    }
    if (h != _height)
    {
        _height = h;
        resize = true;
    }
    if (resize)
    {
        glViewport(0,0,_width,_height);
        _cam.SetAspectRatio(float(_width)/float(_height));
    }

    // clear buffer
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // bind shader
    _testShader->Bind();

    auto CameraviewMatrix = _cam.ViewMatrix();
   /* float aaa[16] =
    {
         0 , 0, 1, 0,
         0 ,  1 , 0, 0,
         1 , 0, 0, 0,
         0 , 0, 0, 1
    };
*/
    float aaa[16] =
    {
         -1 , 0, 0, 0,
         0 ,  0 , -1, 0,
         0 , -1, 0, 0,
         0 , 0, 0, 1
    };
    glm::quat qPitch = glm::angleAxis(0.0f,glm::vec3(1,0,0) );
    glm::quat qYaw = glm::angleAxis(0.0f, glm::vec3(0, 1,0));
    glm::quat qRoll = glm::angleAxis(glm::pi<float>(),glm::vec3(0,0,1));

    glm::quat orientation = qPitch * qYaw*qRoll;
    orientation = glm::normalize(orientation);

    float bbb[16] =
    {
         1 , 0, 0, 0,
         0 , 0, -1, 0,
         0 , -1, 0, 0,
         0 , 0, 0, 1
    };

    glm::mat4x4 correctiveMatrix = glm::make_mat4x4(aaa);
    glm::mat4x4 correctiveMatrix_s = glm::mat4_cast(orientation);
     glm::mat4x4 correctiveMatrix_3 = glm::make_mat4x4(bbb);
    glm::mat4x4 viewMatrix  =     (CameraviewMatrix) *  correctiveMatrix_s* correctiveMatrix ;//* correctiveMatrix_3 ;
    _testShader->SetUniform("uProjMatrix", _cam.ProjMatrix());
    _testShader->SetUniform("uViewMatrix",viewMatrix);
    _testShader->SetUniform("uViewMatrixNormal", transpose(inverse(viewMatrix)) );

    _testShader->SetUniform("uGridSize",_grid_width);

    // bind data
    _gridCoordBuffer.MapData(_testShader->AttributeLocation("inGridCoord"));
   // _terrainHeightBuffer.MapData(_testShader->AttributeLocation("inTerrainHeight"));
   // _waterHeightBuffer.MapData(_testShader->AttributeLocation("inWaterHeight"));
    _sedimentBuffer.MapData(_testShader->AttributeLocation("inSediment"));
    _simDataBuffer.MapData(_testShader->AttributeLocation("inSimData"));

    _normalBuffer.MapData(_testShader->AttributeLocation("inNormal"));

    _gridIndexBuffer.Bind();

    // render terrain
    _testShader->SetUniform("uColor", vec4(242.0/255.0,224.0/255.0,201.0/255.0,1));
    _testShader->SetUniform("uIsWater",false);

    glDrawElements(GL_TRIANGLES, _gridIndexBuffer.IndexCount(), _gridIndexBuffer.IndexType(),0);


    // unbind shader
    _testShader->UnBind();

    // finish
    glfwSwapBuffers(_window);
    glFinish();
}

void OpenGlVisualizer::ParseInputFile(std::vector<glm::vec3>& gridCoords,std::vector<uint>& gridIndices)
{

    glm::vec3 normalizationVector;
    glm::vec3 offset;
    //gridCoords->width = _mesh.n_vertices();
    //gridCoords->height = 1;
    //gridCoords->points.resize (gridCoords->width * gridCoords->height);
    gridCoords.resize( _mesh.n_vertices());
    string line;
    int width  = 300;
    int  height = 300;
    std::ifstream inputfile(_obj_file);
    std::getline(inputfile,line);
    std::istringstream iss_begin(line);

    //gridCoords.resize(width,height);
    gridIndices.reserve((width-1)*(height-1)*6);

    int i = 0;


    MyMesh::Point min_box = MyMesh::Point(INFINITY,INFINITY,INFINITY);
    MyMesh::Point max_box = MyMesh::Point(-INFINITY,-INFINITY,-INFINITY) ;

    MyMesh::VertexIter v_it, v_end(_mesh.vertices_end());




   //MyMesh::VertexIter v_it, v_end(_mesh.vertices_end());
   for (v_it = _mesh.vertices_sbegin() ; v_it!= v_end;++v_it )
   {
       MyMesh::Point point = _mesh.point(*v_it);
     /*  for(int k = 0;k<3;k++)
       point[k] += offset[k]/normalizationVector[k];

       gridCoords[i++] =glm::vec3(point[0],point[1],point[2] );
               //pcl::PointXYZ(point[0]/normalizationVector[0],point[1]/normalizationVector[1],point[2]/normalizationVector[2]);
       //gridCoords(i++) = glm::vec2();
*/

       min_box[0] = point[0] < min_box[0] ? point[0] : min_box[0];
       min_box[1] = point[1] < min_box[1] ? point[1] : min_box[1];
       min_box[2] = point[2] < min_box[2] ? point[2] : min_box[2];


       max_box[0] = point[0] > max_box[0] ? point[0] : max_box[0];
       max_box[1] = point[1] > max_box[1] ? point[1] : max_box[1];
       max_box[2] = point[2] > max_box[2] ? point[2] : max_box[2];






   }

    MyMesh::Point mesh_center = (min_box + max_box) / 2;
    MyMesh::Point normalized_cube_center(0.5,0.5,0.5);
    float longest_dimension = -INFINITY;
    for(int k = 0;k<3;k++)
    {
        offset[k] =  normalized_cube_center[k]-mesh_center[k];
        normalizationVector[k] = max_box[k]-min_box[k];
        if(normalizationVector[k]<1)
            normalizationVector[k] =1;
        longest_dimension = longest_dimension < normalizationVector[k]  ? normalizationVector[k] : longest_dimension;
    }
   for (v_it = _mesh.vertices_sbegin() ; v_it!= v_end;++v_it )
   {
       MyMesh::Point point = _mesh.point(*v_it);
       for(int k = 0;k<3;k++)
       point[k] = (point[k])/longest_dimension;

       gridCoords[i++] =glm::vec3(point[0],point[1],point[2] );



   }
    _cam.SetScalingCostant(longest_dimension);

   for (auto& face_handle : _mesh.faces())
   {
       OpenMesh::TriMesh_ArrayKernelT<>::FaceVertexIter face_vertex_circulator;
       for( face_vertex_circulator = _mesh.fv_iter(face_handle);face_vertex_circulator.is_valid();++face_vertex_circulator)
       {

           stringstream ss;
           ss<<*face_vertex_circulator;
           string tmp  = ss.str();
           int x = stof(tmp);
           gridIndices.push_back(x);
       }
   }

    /*while (std::getline(inputfile,line))
    {

        std::size_t pos = line.find('v');
        if( pos != std::string::npos )
        {
            float x,y,z;
            std::istringstream iss(line.substr(pos+1,line.length()));
            iss >> x >> y >> z;
            gridCoords(i++) = glm::vec2(x,y);
        }
        else
        {
            pos = line.find('f');
            if(pos != std::string::npos )
            {
                int a,b,c;
                std::istringstream iss(line.substr(pos+1,line.length()));
                iss >> a >> b >> c;
                gridIndices.push_back(--a);
                gridIndices.push_back(--b);
                gridIndices.push_back(--c);
            }
        }
    }*/

}

void OpenGlVisualizer::CameraMovement(float dt)
{
    float dtSeconds = dt/15000.0f;

    // camera control
    float camSpeed = 1.0f*dtSeconds;
    float rotSpeed = 60.0f*dtSeconds;

    vec3 yAxis(0,1,0);
    vec3 xAxis(1,0,0);
    vec3 zAxis(0,0,1);

    if (glfwGetKey(_window,'D')) _cam.TranslateLocal(vec3(camSpeed,0,0));
    if (glfwGetKey(_window,'A')) _cam.TranslateLocal(vec3(-camSpeed,0,0));
    if (glfwGetKey(_window,'R')) _cam.TranslateLocal(vec3(0,camSpeed,0));
    if (glfwGetKey(_window,'F')) _cam.TranslateLocal(vec3(0,-camSpeed,0));
    if (glfwGetKey(_window,'W')) _cam.TranslateLocal(vec3(0,0,-camSpeed));
    if (glfwGetKey(_window,'S')) _cam.TranslateLocal(vec3(0,0,camSpeed));
    if (glfwGetKey(_window,'Q')) {  _cam.LocalRotate(yAxis,-rotSpeed);   _cam.LocalRotateRIB(yAxis,rotSpeed);    }
    if (glfwGetKey(_window,'E')) {  _cam.LocalRotate(yAxis,rotSpeed);    _cam.LocalRotateRIB(yAxis, -rotSpeed);  }
    if (glfwGetKey(_window,'T')) {  _cam.LocalRotate(xAxis,-rotSpeed);   _cam.LocalRotateRIB(xAxis,rotSpeed);    }
    if (glfwGetKey(_window,'G')) {  _cam.LocalRotate(xAxis,rotSpeed);    _cam.LocalRotateRIB(xAxis, -rotSpeed);  }
    if (glfwGetKey(_window,'Z')) {  _cam.LocalRotate(zAxis,-rotSpeed);   _cam.LocalRotateRIB(zAxis, -rotSpeed);   }
    if (glfwGetKey(_window,'X')) {  _cam.LocalRotate(zAxis,rotSpeed);    _cam.LocalRotateRIB(zAxis, rotSpeed);  }





    if (glfwGetKey(_window,'1')) ShowSimulationDataInput();
    if (glfwGetKey(_window,'2')) ShowSelectedFaces();

}

int OpenGlVisualizer::ClampX(int x){ return glm::clamp(x,0,(int) _grid_width - 1);}
int OpenGlVisualizer::ClampY(int y){ return glm::clamp(y,0,(int) _grid_height - 1);}


void OpenGlVisualizer::InitializeBuffers()
{
    // Settings
#if defined(__APPLE__) || defined(__MACH__)
    std::string resourcePath = osx_GetBundleResourcesPath()+"/";
#else
    std::string resourcePath("../AutomaticHeuristic/Resources/");
#endif

    GLuint vao;
    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    // Create some data
    //Grid2D<vec3> vertexCoords;
    //MyCloudPtr vertexCoords (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<glm::vec3> vertexCoords;

    std::vector<uint> gridIndices;

    uint dimX = _grid_width;//_simulationState.terrain.width();
    uint dimY = _grid_height;//_simulationState.terrain.height();


    ParseInputFile(vertexCoords,gridIndices);

    // Send data to the GPU
    _gridIndexBuffer.SetData(gridIndices);
    _gridCoordBuffer.SetData(vertexCoords);

    _water.resize(_grid_width,_grid_height);
    _suspendedSediment.resize(_grid_width,_grid_height);
    _simData.resize(_grid_width,_grid_height);
    _terrain.resize(_grid_width,_grid_height);
    _surfaceNormals.resize(_grid_width,_grid_height);
    //PerlinNoise perlin;

    for (uint y=0; y<_water.height(); y++)
    {
        for (uint x=0; x<_water.width(); x++)
        {
            _water(y,x) = 0.0f;
            _suspendedSediment(y,x) = 0.0f;// 0.1*terrain(y,x);
            _simData(y,x) = 0.0f;

        }
    }


    auto simulation_data_wrapper = OpenMesh::getOrMakeProperty<MyMesh::VertexHandle,float>(_mesh, "simulation_data");
    ShowSimulationDataInput();



    // Load and configure shaders
    _testShader = _shaderManager.LoadShader(resourcePath+"lambert_v.glsl",resourcePath+"lambert_f.glsl");
    _testShader->MapAttribute("inGridCoord",0);
    //_testShader->MapAttribute("inTerrainHeight",1);
    //_testShader->MapAttribute("inWaterHeight",2);
    _testShader->MapAttribute("inSediment",3);
    //change the number
    (_testShader->MapAttribute("inSimData",4));

    _testShader->MapAttribute("inNormal",7);

    // position camera
   // float scale = 100;
   // float a = 0;
   // float b = 0;
   // float c = 0;
   // _cam.TranslateGlobal(vec3(a/scale,b/scale,c/scale));

    // OpenGL Settings
    glClearColor(0.4f,0.4f,0.4f,0.0f);
    //glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);



            for (int y=0; y<_grid_height; ++y)
             {
                 for (int x=0; x<_grid_width; ++x)
                 {
                     float r,l,t,b;
                     vec3 N;
                     r = _terrain(y,ClampX(x+1)) + _water(y,ClampX(x+1));
                     l = _terrain(y,ClampX(x-1)) + _water(y,ClampX(x-1));
                     t = _terrain(ClampY(y+1),x) + _water(ClampY(y+1),x);
                     b = _terrain(ClampY(y-1),x) + _water(ClampY(y-1),x);
                     N = vec3(l-r, t - b, 2 );
                     N = normalize(N);


                 }
             }

            MyMesh::VertexIter mesh_vertex_iterator;
            MyMesh::VertexIter vertex_iterator_end(_mesh.vertices_end());
int i = 0;
    for(mesh_vertex_iterator=_mesh.vertices_begin();mesh_vertex_iterator!= vertex_iterator_end;++mesh_vertex_iterator){
        MyMesh::Normal temp_normal = _mesh.normal(mesh_vertex_iterator);
        glm::vec3 normal(temp_normal[0],temp_normal[0],temp_normal[0]);
    _surfaceNormals(i) = normal;
i++;
}
_normalBuffer.SetData(_surfaceNormals);
}
void OpenGlVisualizer::ShowSelectedFaces()
{
    /*std::cout<<"showing selected faces"<<std::endl;
    int i = 0;
    auto shader_parameters_data_wrapper = OpenMesh::getOrMakeProperty<MyMesh::VertexHandle, ShadersWrapper*>(_mesh, "shader_parameters");

    for (auto& vertex_handle : _mesh.vertices())
        {
            MyMesh::Point point = _mesh.point(vertex_handle);
            ShadersWrapper* sp = shader_parameters_data_wrapper[vertex_handle];
            std::vector<ShaderParameters> list;
            sp->GetListOfShaders(list);

            //clear the previous data
            _water(i) = 0;
            _simData(i) = 0;
            _suspendedSediment(i) = 0;
            _terrain(i) = point[2];
            for(auto it = list.begin();it != list.end();++i){
            if(it->GetId() ==1)
                 _water(i) = 1;
            if(it->GetId()==3)
                _simData(i)= 5;
           if(it->GetId()== 0)
           {
               _water(i) = 0;
               _simData (i) = 0;
               _suspendedSediment(i) = 1.f;
           }

            }
        i++;

        }

    _terrainHeightBuffer.SetData(_terrain);
    _waterHeightBuffer.SetData(_water);
    _simDataBuffer.SetData(_simData);*/
}

void OpenGlVisualizer::ShowSimulationDataInput()
{
  std::cout<<"showing simulation input data"<<std::endl;
  auto simulation_data_wrapper = OpenMesh::getOrMakeProperty<MyMesh::VertexHandle,std::shared_ptr<SimulationData>>(_mesh, "simulation_data");
  int i = 0;
  for (auto& vertex_handle : _mesh.vertices())
  {
      MyMesh::Point point = _mesh.point(vertex_handle);
      //SimulationData* sd
      std::shared_ptr<SimulationData> sd = simulation_data_wrapper[vertex_handle];

      _terrain(i) = point[2];
      //clear the previous data
      _water(i) = 0;
      _simData(i) = 0;
      _suspendedSediment(i) = 0;
      float river;
      sd->getData(SimulationDataEnum::river,river);
      _water(i)=  glm::clamp( river,0.f,1.f );
      if(river == 0){
          float val = -1;
          sd->getData(SimulationDataEnum::vegetation,val);
          glm::clamp(val,0.f,1.f);
        }//_simData(i)= glm::clamp(sd->g map_of_floats[SimulationDataEnum::vegetation];,0.f,1.f);

    i++;
  }
  _sedimentBuffer.SetData(_suspendedSediment);
  _terrainHeightBuffer.SetData(_terrain);
  _waterHeightBuffer.SetData(_water);
  _simDataBuffer.SetData(_simData);


}
/*
void OpenGlVisualizer::InitializeOpengGL()
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

    GLFWwindow* window = glfwCreateWindow(800, 600, "BlankWindow", NULL, NULL);
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
    _window = window;

}
*/
