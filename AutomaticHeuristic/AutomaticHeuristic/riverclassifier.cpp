#include "riverclassifier.h"
#include <unordered_set>
#include <list>

    RiverClassifier::RiverClassifier(MyMesh mesh)
    {
        _mesh = mesh;
         simulation_data_wrapper = getOrMakeProperty<VertexHandle,SimulationData*>(_mesh, "simulation_data");

    }


    map<MyMesh::VertexHandle,float> RiverClassifier::selectFrontier(map<MyMesh::VertexHandle,float> river_vertices)
    {
        cout<<"Inside SelectFrontier" <<endl;
        map<MyMesh::VertexHandle,float> frontier;
        float river;
        for (auto const& entry : river_vertices)
        {
            VertexHandle vertex_handle = entry.first;
            //for each selected vertices iterate over its neighborhood, if a vertex that doesnt have the required property
            //is encountered then add the actual vertex in the frontier.

        //      MyMesh::VertexVertexIter vv_it = _mesh.vv_iter(vertex_handle);
              for (MyMesh::VertexVertexIter vertex_vertex_iterator = _mesh.vv_iter(vertex_handle); vertex_vertex_iterator.is_valid(); ++vertex_vertex_iterator)
              {
                  auto simulation_data_wrapper = getOrMakeProperty<VertexHandle,SimulationData*>(_mesh, "simulation_data");
                  SimulationData* sd = simulation_data_wrapper[*vertex_vertex_iterator];
                  river = sd->_map.at("rivers");
                  if( river == 0)
                  {
                      //insert in the map the actual vertex
                      frontier.insert(make_pair(vertex_handle,simulation_data_wrapper[*vertex_vertex_iterator]->_map.at("rivers")));
                      break;
                  }
              }
        }
        return frontier;
    }

    map<MyMesh::FaceHandle,float> RiverClassifier::ClassifyVertices()
    {
        MyMesh::Vertex _vertex;
        MyMesh::VertexIter vertex_iterator,vertex_iterator_end(_mesh.vertices_end());
        map<MyMesh::FaceHandle,float> selected_faces;
        map<MyMesh::VertexHandle,float> river_vertices;
        int counter = 0;
        for(vertex_iterator=_mesh.vertices_begin();vertex_iterator != vertex_iterator_end;++vertex_iterator)
        {
            counter++;
            _vertex = _mesh.vertex(*vertex_iterator);
            //MyMesh::VertexHandle vertex_handle = vertex_iterator.handle();
            //PropertyManager<typename HandleToPropHandle<VertexHandle, SimulationData*>::type, MyMesh> simulation_data_wrapper = getOrMakeProperty<VertexHandle,SimulationData*>(_mesh, "simulation_data");
            SimulationData* sd = simulation_data_wrapper[*vertex_iterator];

            if(sd->_map.at("rivers")>0.0f)
            {
                river_vertices.insert(make_pair(*vertex_iterator,sd->_map.at("rivers")));
                cout<<"rivers is "<< sd->_map.at("rivers")<<endl;
            }
            else
            {
                cout<<"river is 0"<<endl;
            }
        }
        auto frontier = selectFrontier(river_vertices);
        selected_faces = SelectFacesBySlope(frontier);
        return selected_faces;
    }

    struct BFS_cell
    {
        MyMesh::VertexHandle vertex_handle;
        float val;
        int depth;
        bool operator==(const struct BFS_cell& c) const
            {
                return (this->vertex_handle.idx() == c.vertex_handle.idx());
            }
    };

    class BFS_CellHashFunction {
    public:

        size_t operator()(const BFS_cell& cell) const
        {
            VertexHandle vh = cell.vertex_handle;
            return vh.idx();
        }
    };

    map<MyMesh::VertexHandle,float> RiverClassifier::BFS(int max_depth,map<MyMesh::VertexHandle,float> frontier_map)
    {
        cout<<"insideBFS"<<endl;
        unordered_set<BFS_cell,BFS_CellHashFunction> visitedNodes;
        map<MyMesh::VertexHandle,float>  selected_nodes;
        list<struct BFS_cell> queue;
        selected_nodes = frontier_map;
        cout<<"corners"<<endl;
        for (auto const& entry : frontier_map )
        {
            struct BFS_cell cell = {entry.first,entry.second,0};
            visitedNodes.insert(cell);
            queue.push_back(cell);
            MyMesh::VertexHandle vh = entry.first;
            cout<<_mesh.point( vh) <<endl;
        }
        cout<<"END"<<endl;
        while(!queue.empty())
        {
            BFS_cell cell = queue.front();
            queue.pop_front();
            if( cell.depth < max_depth)
            {
                cout<<"Actual vertex "<< _mesh.point(cell.vertex_handle)<<endl;
//                MyMesh::VertexVertexIter vv_it = _mesh.vv_iter(cell.vertex_handle);
                //circulate over the vertex
                for (MyMesh::VertexVertexIter vertex_vertex_iterator = _mesh.vv_iter(cell.vertex_handle); vertex_vertex_iterator.is_valid(); ++vertex_vertex_iterator)
                {

                    cout<<_mesh.point( *vertex_vertex_iterator ) <<endl;

                    SimulationData* sd  = simulation_data_wrapper[*vertex_vertex_iterator];
                    //takes only vertices that are out of the set (since it's given that we're using the frontier vertices)
                    if(sd->_map.at("rivers")<=0.0f)
                    {
                        BFS_cell new_cell = {*vertex_vertex_iterator,0,cell.depth+1};
                        if(visitedNodes.count(new_cell)<=0)
                        {
                         visitedNodes.insert(new_cell);
                         selected_nodes.insert(make_pair<VertexHandle,float>(*vertex_vertex_iterator,0));
                         queue.push_back(new_cell);
                        }

                    }


                }

                visitedNodes.insert(cell);

            }
            else
            {
                continue;
            }

        }

   return selected_nodes;
    }


    map<MyMesh::FaceHandle,float> RiverClassifier::SelectFacesBySlope(map<MyMesh::VertexHandle,float> rivers_boundaries)
    {
        //along the river boundaries (i.e. the rivers frontier with some treshold)
        //We select those faces which are still rather steep.
        //Steep faces might indicates the fact that the river eroded those faces very quickly, hence with a great strength
        //The rivers flow might still be visible
        MyMesh::Face face;

        MyMesh::Normal up_direction = Vec3f(1,0,0);
        MyMesh::FaceIter face_iterator,face_iterator_end(_mesh.faces_end());
        map<MyMesh::FaceHandle,float> selected_faces ;
        int counter = 0;
        MyMesh::VertexFaceIter vertex_face_circulator;
        for(auto &entry : rivers_boundaries)
        {
            MyMesh::VertexHandle vertex_handle = entry.first;
            for( vertex_face_circulator = _mesh.vf_iter(vertex_handle);vertex_face_circulator.is_valid();++vertex_face_circulator)

//            for (MyMesh::VertexVertexIter vertex_vertex_iterator = mesh.vv_iter(*vertex_iterator); vertex_vertex_iterator.is_valid(); ++vertex_vertex_iterator)
//
//            for(face_iterator=_mesh.faces_begin();face_iterator != face_iterator_end;++face_iterator)
            {
                face = _mesh.face(*vertex_face_circulator);
                MyMesh::Normal mynormal = _mesh.normal(*vertex_face_circulator);

                float dot_result = dot(mynormal,up_direction);
                float resulting_angle_in_radians = acos(dot_result);
                float resulting_angle_in_degree = resulting_angle_in_radians* (180.0/  M_PI);
                resulting_angle_in_degree = 90 - resulting_angle_in_degree;
                resulting_angle_in_degree = resulting_angle_in_degree > 0 ? resulting_angle_in_degree : 0;

                if(resulting_angle_in_degree <= _slope + _treshold && resulting_angle_in_degree >= _slope - _treshold)
                {
                      selected_faces.insert(pair<MyMesh::FaceHandle,float>(*face_iterator,1));
                      counter++;
                }

            }
        }
    }

  /*  // Program to print BFS traversal from a given
    // source vertex. BFS(int s) traverses vertices
    // reachable from s.
    #include<iostream>
    #include <list>


    // This class represents a directed graph using
    // adjacency list representation
    class Graph
    {
        int V; // No. of vertices

        // Pointer to an array containing adjacency
        // lists
        list<int> *adj;
    public:
        Graph(int V); // Constructor

        // function to add an edge to graph
        void addEdge(int v, int w);

        // prints BFS traversal from a given source s
        void BFS(int s);
    };

    Graph::Graph(int V)
    {
        this->V = V;
        adj = new list<int>[V];
    }

    void Graph::addEdge(int v, int w)
    {
        adj[v].push_back(w); // Add w to v’s list.
    }

    void Graph::BFS(int s)
    {
        // Mark all the vertices as not visited
        bool *visited = new bool[V];
        for(int i = 0; i < V; i++)
            visited[i] = false;

        // Create a queue for BFS
        list<int> queue;

        // Mark the current node as visited and enqueue it
        visited[s] = true;
        queue.push_back(s);

        // 'i' will be used to get all adjacent
        // vertices of a vertex
        list<int>::iterator i;

        while(!queue.empty())
        {
            // Dequeue a vertex from queue and print it
            s = queue.front();
            cout << s << " ";
            queue.pop_front();

            // Get all adjacent vertices of the dequeued
            // vertex s. If a adjacent has not been visited,
            // then mark it visited and enqueue it
            for (i = adj[s].begin(); i != adj[s].end(); ++i)
            {
                if (!visited[*i])
                {
                    visited[*i] = true;
                    queue.push_back(*i);
                }
            }
        }
    }

    // Driver program to test methods of graph class
    int main()
    {
        // Create a graph given in the above diagram
        Graph g(4);
        g.addEdge(0, 1);
        g.addEdge(0, 2);
        g.addEdge(1, 2);
        g.addEdge(2, 0);
        g.addEdge(2, 3);
        g.addEdge(3, 3);

        cout << "Following is Breadth First Traversal "
            << "(starting from vertex 2) \n";
        g.BFS(2);

        return 0;
    }
*/
