#include "riverclassifier.h"
#include <unordered_set>
#include <list>

    RiverClassifier::RiverClassifier(MyMesh mesh,float slope,float treshold,float border_width,float max_height,float min_height):AClassifier()
    {
        _mesh = mesh;
        _slope = slope;
        _treshold = treshold;
        _border_width = border_width;
         simulation_data_wrapper = getOrMakeProperty<VertexHandle,SimulationData*>(_mesh, "simulation_data");
        _max_height = max_height;
        _min_height = min_height;
    }

    map<MyMesh::VertexHandle,ShaderParameters*> RiverClassifier::ClassifyVertices()
    {
        map<MyMesh::VertexHandle,float> river_vertices = SelectRiverVertices();
        auto frontier = selectFrontier(river_vertices);
        frontier = BFS(_border_width,frontier);
        auto selected_faces = SelectFacesBySlope(frontier);
        return selected_faces;
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

     map<MyMesh::VertexHandle,float> RiverClassifier::SelectRiverVertices()
     {
         MyMesh::VertexIter vertex_iterator,vertex_iterator_end(_mesh.vertices_end());
         map<MyMesh::VertexHandle,float> river_vertices;
          for(vertex_iterator=_mesh.vertices_begin();vertex_iterator != vertex_iterator_end;++vertex_iterator)
         {
             SimulationData* sd = simulation_data_wrapper[*vertex_iterator];
             if(sd->_map.at("rivers")>0.0f)
             {
                 river_vertices.insert(make_pair(*vertex_iterator,sd->_map.at("rivers")));
             }
         }
     return river_vertices;
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
        unordered_set<BFS_cell,BFS_CellHashFunction> visitedNodes;
        map<MyMesh::VertexHandle,float>  selected_nodes;
        list<struct BFS_cell> queue;
        selected_nodes = frontier_map;
        for (auto const& entry : frontier_map )
        {
            struct BFS_cell cell = {entry.first,entry.second,0};
            visitedNodes.insert(cell);
            queue.push_back(cell);
        }
        while(!queue.empty())
        {
            BFS_cell cell = queue.front();
            queue.pop_front();
            if( cell.depth < max_depth)
            {
                //circulate over the vertex
                for (MyMesh::VertexVertexIter vertex_vertex_iterator = _mesh.vv_iter(cell.vertex_handle); vertex_vertex_iterator.is_valid(); ++vertex_vertex_iterator)
                {
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

        }

   return selected_nodes;
    }


    map<MyMesh::VertexHandle,ShaderParameters*> RiverClassifier::SelectFacesBySlope(map<MyMesh::VertexHandle,float> rivers_boundaries)
    {
        //along the river boundaries (i.e. the rivers frontier with some treshold)
        //We select those faces which are still rather steep.
        //Steep faces might indicates the fact that the river eroded those faces very quickly, hence with a great strength
        //The rivers flow might still be visible
        MyMesh::Normal up_direction = Vec3f(0,0,1);
        map<MyMesh::VertexHandle,ShaderParameters*> selected_faces ;
        MyMesh::VertexFaceIter vertex_face_circulator;
        float bins = 10;

        for(auto &entry : rivers_boundaries)
        {
            MyMesh::VertexHandle vertex_handle = entry.first;
            for( vertex_face_circulator = _mesh.vf_iter(vertex_handle);vertex_face_circulator.is_valid();++vertex_face_circulator)
            {
                    MyMesh::Normal mynormal = -1*_mesh.normal(*vertex_face_circulator);

                    float dot_result = dot(mynormal,up_direction);
                    float resulting_angle_in_radians = acos(dot_result);
                    float resulting_angle_in_degree = resulting_angle_in_radians* (180.0/  M_PI);
                  //  resulting_angle_in_degree = 90 - resulting_angle_in_degree;
                    if(resulting_angle_in_degree <= _slope + _treshold && resulting_angle_in_degree >= _slope - _treshold)
                    {
                        MyMesh::Point point = _mesh.point(vertex_handle);
                        ShaderParameters* shader_parameters = new ShaderParameters(_id,10);
                        //roundf should work
                        cout<<"point[2]"<<point[2]<<" vs " << bins*roundf(point[2]/bins)<< endl;
                        shader_parameters->setValue(0, bins*roundf(point[2]/bins));
                          selected_faces.insert(pair<MyMesh::VertexHandle,ShaderParameters*>(vertex_handle,shader_parameters));
                    }

            }
        }
        return selected_faces;
    }

