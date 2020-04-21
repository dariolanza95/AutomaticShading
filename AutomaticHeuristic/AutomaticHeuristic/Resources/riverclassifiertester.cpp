#include "riverclassifiertester.h"
/*
class selectRiverFrontierFunctorClass
{
    public:
    selectRiverFrontierFunctorClass(){}
    selectRiverFrontierFunctorClass (MyMesh mesh): _mesh(mesh){}
            int operator() (MyMesh::VertexHandle vertex_handle) {
            auto simulation_data_wrapper = getOrMakeProperty<VertexHandle,SimulationData*>(_mesh, "simulation_data");
            SimulationData* sd  = simulation_data_wrapper[vertex_handle];

            glm::vec3 vec;
            sd->getData(SimulationDataEnum::flow_normal,vec);
            float river;

            sd->getData(SimulationDataEnum::river,river);
            if(river<=0.0f)
                return 1;
            else
                return 0;
            }
    private:
            MyMesh _mesh;
           // PropertyManager<typename HandleToPropHandle<MyMesh::VertexHandle , SimulationData*>::type, MyMesh> simulation_data_wrapper;
};

void RiverClassifierTester::AttachMockUpSimulationDataToAllVertices()
{
    auto simulation_data = getOrMakeProperty < VertexHandle, SimulationData*>(mesh, "simulation_data");
    //Attach to all the vertices some simulation data (>0)
    for (auto& vertex_handle : mesh.vertices())
    {
        string line = {"river 1.0f vegetation 1.0f" };
        SimulationData *sd =new SimulationData( line);
        assert(sd!=nullptr);
        simulation_data[vertex_handle] = sd;
    }
}

void RiverClassifierTester::AttachMockUpSimulationToABox(int width,int height,float box_width,float box_height,float centerX,float centerY)
{
    auto simulation_data = getOrMakeProperty < VertexHandle, SimulationData*>(mesh, "simulation_data");
    //Attach to all the vertices some simulation data (>0)
    int i = 0;
    int inserted = 0;
    float x,y = 0;

    for (auto& vertex_handle : mesh.vertices())
    {
        x = i%width;
        y = floor(i/width);
        stringstream line;
        if(x >= centerX - box_width && x <= centerX+ box_width && y>= centerY - box_height && y <= centerY+box_height)
        {
                inserted++;
                line<<" river 1.0f" ;
        }
        else
        {
            line<<" river 0.0f" ;
        }
        SimulationData* sd = new SimulationData(line.str());
        simulation_data[vertex_handle] = sd;
         i++;
    }


    assert(inserted == ((box_width*2)+1)*((box_height*2)+1));

}

void RiverClassifierTester::attachMockUpSimulationDataToCorners(int width,int height)
{
    auto simulation_data = getOrMakeProperty < VertexHandle, SimulationData*>(mesh, "simulation_data");
    //Attach to all the vertices some simulation data (>0)

    int i = 0;
    for (auto& vertex_handle : mesh.vertices())
    {
        stringstream line("vegetations 0.0f ");

        if(i == 0 || i == width-1 || i== (width-1)*(height) || i == (height)*(width)-1)
        {
            line<<" rivers 1.0f";
        }
        else
        {  
            line<<" rivers 1.0f";
        }

        i++;
        SimulationData *sd =new SimulationData(line.str());
        assert(sd!=nullptr);
        simulation_data[vertex_handle] = sd;
    }
}

RiverClassifierTester::RiverClassifierTester()
{

}

void RiverClassifierTester::Test()
{
    TestSelectFrontier();
    TestSelectBySlope();


}
void RiverClassifierTester::TestSelectBySlope()
{
    int width = 10;
    int height = 10;
    float angle = 10;
    float treshold = 1;
    mesh = Grid(width,height,angle);
    map<MyMesh::VertexHandle,float> selected_vertices;
    MyMesh::VertexIter vertex_iterator,vertex_iterator_end(mesh.vertices_end());
    for(vertex_iterator=mesh.vertices_begin();vertex_iterator != vertex_iterator_end;++vertex_iterator)
    {
        selected_vertices.insert(make_pair(*vertex_iterator,1.0f));
    }
       rc = new RiverClassifier(mesh,angle,treshold,5,0,0);
       auto res = rc->SelectFacesBySlope(selected_vertices);
       cout<<"Test select by slope, selected "<< res.size()<<" faces ------>"<< (width)*(height);
       assert(res.size() == (width)*(height) );
       cout<<" passed"<<endl;
}

void RiverClassifierTester::TestSelectFrontier()
{
    int width = 10;
    int height = 10;
    mesh = Grid(width,height);
    cout<<"Attaching Data"<<endl;
    AttachMockUpSimulationDataToAllVertices();
    rc = new RiverClassifier(mesh,65,15,5,0,0);

    //Empty List the frontier should be empty as well
    map<MyMesh::VertexHandle,float> river_vertices;
    auto frontier = rc->selectFrontier(river_vertices);
    cout<<"List of selected faces is empty, frontier should be empty too---->";
    assert( frontier.empty() );
    cout<<" passed "<<endl;
    //Select all the vertices, frontier should be empty again
    frontier.clear();
    river_vertices.clear();
    MyMesh::VertexIter vertex_iterator,vertex_iterator_end(mesh.vertices_end());
    for(vertex_iterator=mesh.vertices_begin();vertex_iterator != vertex_iterator_end;++vertex_iterator)
    {
        river_vertices.insert(make_pair(*vertex_iterator,1.0f));
    }

    frontier = rc->selectFrontier(river_vertices);
    cout<<"List of selected faces is full, frontier should be empty ---->";
    assert(frontier.empty());
    cout<<" passed"<<endl;
    selectGridExternalCorners_testSelectFrontierFunction(width,height);
    int box_half_width = 1;
    int box_half_height = 1;
    int centerX = ceil(width/2);
    int centerY = ceil(height/2);
    selectBox_testSelectFrontier(width,height,box_half_width,box_half_height,centerX,centerY);


}
void RiverClassifierTester::selectBox_testSelectFrontier(int width,int height,int box_half_width,int box_half_height,int centerX,int centerY)
{
    //Create a central line, the frontier should be equal to (box_half_height*2+1)*2 +(box_half_width*2+1-2)*2 ,
    //regardeless how width the line is
    // iff( the line has a width >1)
    int x(0),y = 0;
    AttachMockUpSimulationToABox(width,height,box_half_width,box_half_height,centerX,centerY);
    rc = new RiverClassifier(mesh,65,15,5,0,0);
    map<MyMesh::VertexHandle,float> river_vertices;

    int i = 0;
    int inserted = 0;
    MyMesh::VertexIter vertex_iterator;
    MyMesh::VertexIter vertex_iterator_end(mesh.vertices_end());

    for(vertex_iterator=mesh.vertices_begin();vertex_iterator != vertex_iterator_end;++vertex_iterator)
    {
        x = i%width;
        y = floor(i/width);
        if(x>= centerX - box_half_width && x <= centerX + box_half_width)
        {
            if(y>= centerY - box_half_height && y <= centerY+ box_half_height)
            {
                inserted++;
                river_vertices.insert(make_pair(*vertex_iterator,1.0f));
            }
        }

        i++;
    }
    assert(inserted == ((box_half_width*2)+1)*((box_half_height*2)+1));
    auto frontier = rc->selectFrontier(river_vertices);

    cout<<inserted<< " vertices have been selected, frontier should be equal to "<<(box_half_height*2+1)*2 +(box_half_width*2+1-2)*2<<" ------>";
    assert((int)frontier.size()==(box_half_height*2+1)*2 +(box_half_width*2+1-2)*2);
    cout << frontier.size();
    cout<<" passed"<<endl;
    frontierMadeFromBox_testBFSFunction(box_half_width,box_half_height,frontier);
}

void RiverClassifierTester::frontierMadeFromBox_testBFSFunction(int box_half_width,int box_half_height, map<MyMesh::VertexHandle,float> frontier)
{
    rc = new RiverClassifier(mesh,65,15,5,0,0);
    map<MyMesh::VertexHandle,float> river_vertices;
    int max_depth = 3;
    int result_1=0;
    int result_2=0;
    int expected_result= 0;
    for(int i =0;i<=max_depth;i++)
    {
        if(i==0)
        {
            result_1 = (box_half_width*2+1)*2;
            result_2 = (box_half_height*2+1-2)*2;
        }

        else
        {
            result_1 += 3*2;
        }
        expected_result += result_1+result_2;
    }
    selectRiverFrontierFunctorClass functor(mesh);
    river_vertices = rc->BFS(max_depth,frontier,functor);

    cout<<"border should be equal to "<< expected_result << "------->"<<river_vertices.size();
    assert((int)river_vertices.size()==expected_result);
    cout<<"passed"<<endl;
}


void RiverClassifierTester::frontierMadeFromGridExternalCorners_testBFSFunction( map<MyMesh::VertexHandle,float> frontier)
{
    rc = new RiverClassifier(mesh,65,15,5,0,0);
    map<MyMesh::VertexHandle,float> river_vertices;
    int expected_result = 0;
    int max_depth = 3;
    int level = 0;
    int result_1=0;
    int result_2=0;

    //From how the grid is formed 2 cornerns will have 2 neighbours meanwhile the others 2 will have 3 neighbours
    for(int i = 0;i<=max_depth;i++)
    {
        if(i==0)
            level = 1;
        else
        {
            level+=2;
        }
        result_1 += level;
    }

    for(level = 0;level<=max_depth;level++)
    {
        result_2+=level+1;
    }

    expected_result = result_1*2+result_2*2;
      selectRiverFrontierFunctorClass functor(mesh);
    river_vertices = rc->BFS(max_depth,frontier,functor);
    cout<<"border should be equal to "<< expected_result << "------->"<<river_vertices.size();
    assert((int)river_vertices.size()==expected_result);
    cout<<"passed"<<endl;

}



void RiverClassifierTester::selectGridExternalCorners_testSelectFrontierFunction(int width,int height)
{
    int i = 0;
    //Select the 4 corners,frontier should equal to 4*2=8
    attachMockUpSimulationDataToCorners(width,height);
    rc = new RiverClassifier(mesh,65,15,5,0,0);
    map<MyMesh::VertexHandle,float> river_vertices;

    MyMesh::VertexIter vertex_iterator;
    MyMesh::VertexIter vertex_iterator_end(mesh.vertices_end());

    for(vertex_iterator=mesh.vertices_begin();vertex_iterator != vertex_iterator_end;++vertex_iterator)
    {
        if(i == 0 || i == width-1 || i== (width-1)*(height) || i == (height)*(width)-1)
        {
            river_vertices.insert(make_pair(*vertex_iterator,1.0f));
        }
        i++;
    }
    auto frontier = rc->selectFrontier(river_vertices);
    cout<<"4 corners are selected frontier should be equal to 4 ------>";
    assert(frontier.size()==4);
    cout<<" passed"<<endl;
    frontierMadeFromGridExternalCorners_testBFSFunction(frontier);

}
MyMesh RiverClassifierTester::Cube()
{
    MyMesh mesh;

    // generate vertices
    MyMesh::VertexHandle vhandle[8];
    vhandle[0] = mesh.add_vertex(MyMesh::Point(-1, -1,  1));
    vhandle[1] = mesh.add_vertex(MyMesh::Point( 1, -1,  1));
    vhandle[2] = mesh.add_vertex(MyMesh::Point( 1,  1,  1));
    vhandle[3] = mesh.add_vertex(MyMesh::Point(-1,  1,  1));
    vhandle[4] = mesh.add_vertex(MyMesh::Point(-1, -1, -1));
    vhandle[5] = mesh.add_vertex(MyMesh::Point( 1, -1, -1));
    vhandle[6] = mesh.add_vertex(MyMesh::Point( 1,  1, -1));
    vhandle[7] = mesh.add_vertex(MyMesh::Point(-1,  1, -1));
    // generate (quadrilateral) faces
    std::vector<MyMesh::VertexHandle>  face_vhandles;
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[0]);
    face_vhandles.push_back(vhandle[1]);
    face_vhandles.push_back(vhandle[2]);
    face_vhandles.push_back(vhandle[3]);
    mesh.add_face(face_vhandles);

    face_vhandles.clear();
    face_vhandles.push_back(vhandle[7]);
    face_vhandles.push_back(vhandle[6]);
    face_vhandles.push_back(vhandle[5]);
    face_vhandles.push_back(vhandle[4]);
    mesh.add_face(face_vhandles);
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[1]);
    face_vhandles.push_back(vhandle[0]);
    face_vhandles.push_back(vhandle[4]);
    face_vhandles.push_back(vhandle[5]);
    mesh.add_face(face_vhandles);
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[2]);
    face_vhandles.push_back(vhandle[1]);
    face_vhandles.push_back(vhandle[5]);
    face_vhandles.push_back(vhandle[6]);
    mesh.add_face(face_vhandles);
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[3]);
    face_vhandles.push_back(vhandle[2]);
    face_vhandles.push_back(vhandle[6]);
    face_vhandles.push_back(vhandle[7]);
    mesh.add_face(face_vhandles);
    face_vhandles.clear();
    face_vhandles.push_back(vhandle[0]);
    face_vhandles.push_back(vhandle[3]);
    face_vhandles.push_back(vhandle[7]);
    face_vhandles.push_back(vhandle[4]);
    mesh.add_face(face_vhandles);

    return mesh;
}
MyMesh RiverClassifierTester::Grid(int width,int height,float angle)
{
    MyMesh mesh;
    int faces = 0;
    int vertices = width*height;
    int i = 0;
    //add vertices
    MyMesh::VertexHandle vhandle[vertices];
    for (int y=0; y<height; y++)
    {
        for (int x=0; x<width; x++)
        {
            MyMesh::Point p(x,y,x * tan(M_PI*angle/180) );
           vhandle[i++]= mesh.add_vertex( p);
        }
    }
    assert(i == vertices);

    //add faces
    std::vector<MyMesh::VertexHandle>  face_vhandles;
    for (int y=0; y<height-1; y++)
    {
        for (int x=0; x<width-1; x++)
        {
            VertexHandle p00 = vhandle[y*width +x];
            VertexHandle p10 = vhandle[y*width +x+1];
            VertexHandle p01 = vhandle[y*width +x+width];
            VertexHandle p11 = vhandle[y*width +x+1+width];


            // add the two faces
            face_vhandles.clear();
            face_vhandles.push_back(p00);
            face_vhandles.push_back(p11);
            face_vhandles.push_back(p10);
            mesh.add_face(face_vhandles);
            faces++;
            face_vhandles.clear();
            face_vhandles.push_back(p00);
            face_vhandles.push_back(p01);
            face_vhandles.push_back(p11);
            mesh.add_face(face_vhandles);
            faces++;
        }
    }
    assert(faces==(width-1)*(height-1)*2);
    assert(mesh.n_vertices()== (uint) vertices);

    mesh.request_face_normals();

    mesh.update_normals();

    return mesh;


}



MyMesh RiverClassifierTester::Grid(int width,int height)
{
    MyMesh mesh;
    int faces = 0;
    int vertices = width*height;
    int i = 0;
    //add vertices
    MyMesh::VertexHandle vhandle[vertices];
    for (int y=0; y<height; y++)
    {
        for (int x=0; x<width; x++)
        {
           vhandle[i++]= mesh.add_vertex(MyMesh::Point(float(x)/(width-1),float(y)/(height-1),0));
        }
    }
    assert(i == vertices);

    //add faces
    std::vector<MyMesh::VertexHandle>  face_vhandles;
    for (int y=0; y<height-1; y++)
    {
        for (int x=0; x<width-1; x++)
        {
            VertexHandle p00 = vhandle[y*width +x];
            VertexHandle p10 = vhandle[y*width +x+1];
            VertexHandle p01 = vhandle[y*width +x+width];
            VertexHandle p11 = vhandle[y*width +x+1+width];


            // add the two faces
            face_vhandles.clear();
            face_vhandles.push_back(p00);
            face_vhandles.push_back(p11);
            face_vhandles.push_back(p10);
            mesh.add_face(face_vhandles);
            faces++;
            face_vhandles.clear();
            face_vhandles.push_back(p00);
            face_vhandles.push_back(p01);
            face_vhandles.push_back(p11);
            mesh.add_face(face_vhandles);
            faces++;
        }
    }
    assert(faces==(width-1)*(height-1)*2);
    assert(mesh.n_vertices()== (uint) vertices);

    return mesh;
}
void RiverClassifierTester::IterateThroughMesh()
{
float max = 0;
float min = std::numeric_limits<float>::max();
    MyMesh::VertexIter vertex_iterator;
MyMesh::VertexIter vertex_iterator_end(mesh.vertices_end());
for(vertex_iterator=mesh.vertices_begin();vertex_iterator != vertex_iterator_end;++vertex_iterator)
{
    cout<<"Actual vertex is "<<mesh.point( *vertex_iterator ) <<endl;
    //circulate over the vertex
    for (MyMesh::VertexVertexIter vertex_vertex_iterator = mesh.vv_iter(*vertex_iterator); vertex_vertex_iterator.is_valid(); ++vertex_vertex_iterator)
    {
        cout<<"circulating ";
        MyMesh::Point p = mesh.point( *vertex_vertex_iterator) ;
        cout<<p<<endl;
        if(p[2]<min)
            min = p[2];
        if(p[2]>max)
            max = p[2];
    }

}
 cout<<"MAX is "<< max<<" but min is "<< min <<endl;

}
*/
