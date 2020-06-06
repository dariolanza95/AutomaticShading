#include "riverclassifier.h"
#include <unordered_set>
#include <list>

/*
class selectRiverFrontierFunctorClass
{
    public:
    float getValue(VertexHandle vertex_handle) {return 0;}
    selectRiverFrontierFunctorClass(){}
    selectRiverFrontierFunctorClass (MyMesh mesh): _mesh(mesh){}
            int operator() (MyMesh::VertexHandle vertex_handle) {
            auto simulation_data_wrapper = getOrMakeProperty<VertexHandle,SimulationData*>(_mesh, "simulation_data");
            SimulationData* sd  = simulation_data_wrapper[vertex_handle];
            float river;
            sd->getData(SimulationDataEnum::river,river);
            if(river<=0.0f)
                return 1;
            else
                return 0;
            }

    private:
            MyMesh _mesh;

};


class splitInGroupsFunctorClass
{

    public:
    ShaderParameters* getValue(MyMesh::VertexHandle vertex_handle) {
        if(_map.count(vertex_handle) == 1 )
        {
            return _map[vertex_handle];
        }
        else
        {
            return  new ShaderParameters(_id);
        }

    }
    splitInGroupsFunctorClass(map<MyMesh::VertexHandle,ShaderParameters*> map,int id): _map(map),_id(id){}
            int operator() (MyMesh::VertexHandle vertex_handle) {
            if(_map.count(vertex_handle) == 1 )
                return 1;
            else
                return 0;
            }
    private:
            map<MyMesh::VertexHandle,ShaderParameters*> _map;
            int _id;

};


void RiverClassifier::FindMeshExtremes()
{
float max = std::numeric_limits<float>::min();
float min = std::numeric_limits<float>::max();
    MyMesh::VertexIter vertex_iterator;
MyMesh::VertexIter vertex_iterator_end(_mesh.vertices_end());

for(vertex_iterator=_mesh.vertices_begin();vertex_iterator != vertex_iterator_end;++vertex_iterator)
{
        MyMesh::Point p = _mesh.point( *vertex_iterator) ;
        min = p[2]<min ? p[2] : min;
        max = p[2]>max ? p[2] : max;

}
_min_height = min;
_max_height = max;
 cout<<"MAX is "<< max<<" but min is "<< min <<endl;

}


    RiverClassifier::RiverClassifier(MyMesh mesh,float slope,float treshold,float border_width,float max_height,float min_height):AClassifier(mesh)
    {
        _slope = slope;
        _treshold = treshold;
        _border_width = border_width;
         simulation_data_wrapper = getOrMakeProperty<VertexHandle,SimulationData*>(_mesh, "simulation_data");
        _max_height = max_height;
        _min_height = min_height;
        _shader_parameter_size = 10;
    }



    map<MyMesh::VertexHandle,AShader> RiverClassifier::ClassifyVertices()
    {
        map<MyMesh::VertexHandle,float> river_vertices = SelectRiverVertices();
        FindMeshExtremes();
        auto river_frontier = selectFrontier(river_vertices);
        selectRiverFrontierFunctorClass functor(_mesh);
        river_frontier = BFS(_border_width,river_frontier,functor);
        auto selected_faces = SelectFacesBySlope(river_frontier);
        auto groups= DivideInGroups(selected_faces);
        groups = FindLocalMinimumValue(groups);
        selected_faces = CollectGroups(groups);
        return selected_faces;
    }
    map<MyMesh::VertexHandle,ShaderParameters*> RiverClassifier::CollectGroups(vector<map <MyMesh::VertexHandle, ShaderParameters* > > groups)
    {
        map<MyMesh::VertexHandle,ShaderParameters*> result;
        for(int i = 0;i<groups.size();i++)
        {
            for(auto const &entry : groups[i])
            result.insert(entry);
        }
        return result;
    }
    map<MyMesh::VertexHandle,float> RiverClassifier::selectFrontier(map<MyMesh::VertexHandle,float> river_vertices)
    {
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
                  float river;
                  sd->getData(SimulationDataEnum::river,river);
                  if( river == 0)
                  {
                      //insert in the map the actual vertex
                      frontier.insert(make_pair(vertex_handle,river));
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
             float river;
             sd->getData(SimulationDataEnum::river,river);
             if(river>0.0f)
             {
                 river_vertices.insert(make_pair(*vertex_iterator,river));
             }
         }
     return river_vertices;
     }
    template <typename T>
    struct BFS_cell
    {
        MyMesh::VertexHandle vertex_handle;
        T val;
        int depth;
        bool operator==(const struct BFS_cell& c) const
            {
                return (this->vertex_handle.idx() == c.vertex_handle.idx());
            }
    };

    class BFS_CellHashFunction {
    public:
        template <typename T>
        size_t operator()(const BFS_cell<T> & cell) const
        {
            VertexHandle vh = cell.vertex_handle;
            return vh.idx();
        }
    };
    template <typename T,typename FuncType>
    map<MyMesh::VertexHandle,T> RiverClassifier::BFS(int max_depth,map<MyMesh::VertexHandle,T> frontier_map,FuncType pred)
    {
        unordered_set<BFS_cell<T>,BFS_CellHashFunction> visitedNodes;
        map<MyMesh::VertexHandle,T>  selected_nodes;
        list<struct BFS_cell<T>> queue;
        selected_nodes = frontier_map;
        for (auto const entry : frontier_map )
        {
            struct BFS_cell<T> cell = {entry.first,entry.second,0};
            visitedNodes.insert(cell);
            queue.push_back(cell);
        }
        while(!queue.empty())
        {
            BFS_cell<T> cell = queue.front();
            queue.pop_front();
            if( cell.depth < max_depth)
            {
                //circulate over the vertex
                for (MyMesh::VertexVertexIter vertex_vertex_iterator = _mesh.vv_iter(cell.vertex_handle); vertex_vertex_iterator.is_valid(); ++vertex_vertex_iterator)
                {
           //         SimulationData* sd  = simulation_data_wrapper[*vertex_vertex_iterator];
                    //takes only vertices that are out of the set (since it's given that we're using the frontier vertices)
                  //  if(sd->_map.at("rivers")<=0.0f)
                    if(pred(*vertex_vertex_iterator))
                    {
                        BFS_cell<T> new_cell = {*vertex_vertex_iterator,cell.val,cell.depth+1};
                        if(visitedNodes.count(new_cell)<=0)
                        {
                         visitedNodes.insert(new_cell);
                         selected_nodes.insert(make_pair<MyMesh::VertexHandle,T>(*vertex_vertex_iterator,pred.getValue(vertex_vertex_iterator)));
                         queue.push_back(new_cell);
                        }
                    }
                }
                visitedNodes.insert(cell);
            }

        }

   return selected_nodes;
    }


    struct VertexHandleWrapper
    {
        MyMesh::VertexHandle vertex_handle;
        bool operator==(const struct VertexHandleWrapper& c) const
            {
                return (this->vertex_handle.idx() == c.vertex_handle.idx());
            }
    };

    class VectorHandle_wrapperHashFunction {
    public:
        size_t operator()(const VertexHandleWrapper & wrapper) const
        {
            VertexHandle vh = wrapper.vertex_handle;
            return vh.idx();
        }
    };



    typedef std::pair<MyMesh::VertexHandle,ShaderParameters*> MyPairType;
    struct CompareSecond
    {
        bool operator()(const pair<MyMesh::VertexHandle,ShaderParameters*>& left, const pair<MyMesh::VertexHandle,ShaderParameters*>& right) const
        {
            return left.second < right.second;
        }
    };



vector<map<MyMesh::VertexHandle, ShaderParameters *>> RiverClassifier::FindLocalMinimumValue( vector<map<MyMesh::VertexHandle, ShaderParameters *>> vector_of_groups)
    {
        for(int i=0 ;i<vector_of_groups.size();i++)
        {
            map<MyMesh::VertexHandle,ShaderParameters*>& single_group = vector_of_groups[i];
            float min = std::numeric_limits<float>::max() ;
            for(auto entry : single_group)
            {
                float value = _mesh.point( entry.first)[2];
                if(value<min)
                {
                    min = value;
                }
            }
            //min = (min /(_max_height-_min_height))*2;
            for(auto &entry : single_group)
            {
                ShaderParameters* shader_parameters = new ShaderParameters(_id);
                shader_parameters->AddParameter(ShaderParametersEnum::river,min);
                //shader_parameters->setValue(0,min);
                entry.second = shader_parameters;
            }
             vector_of_groups[i] = single_group;

        }
        return vector_of_groups;

    }

    vector<map<MyMesh::VertexHandle, ShaderParameters *>> RiverClassifier::DivideInGroups(map<MyMesh::VertexHandle,ShaderParameters*>& points_to_be_grouped)
    {
        vector<map<MyMesh::VertexHandle,ShaderParameters*>>  result;
        unordered_set<VertexHandleWrapper,VectorHandle_wrapperHashFunction> visited_vertices;

        for(pair<MyMesh::VertexHandle,ShaderParameters*> entry : points_to_be_grouped)
        {
            VertexHandleWrapper wrapper = {entry.first};

            if( visited_vertices.count(wrapper)!=1)
            {
            map<MyMesh::VertexHandle,ShaderParameters*> initial_point;
            initial_point.insert(entry);
            splitInGroupsFunctorClass functor(points_to_be_grouped,_id);
            map<MyMesh::VertexHandle,ShaderParameters*> group = BFS(1000,initial_point,functor);
            for(auto group_element : group)
            {
                VertexHandleWrapper wrapper = {group_element.first};
                visited_vertices.insert(wrapper);
            }
            if(group.size()>1)
            result.push_back(group);
            }
        }
        return result;
    }


    map<MyMesh::VertexHandle,AShader*> RiverClassifier::SelectFacesBySlope(map<MyMesh::VertexHandle,float> rivers_boundaries)
    {
        //along the river boundaries (i.e. the rivers frontier with some treshold)
        //We select those faces which are still rather steep.
        //Steep faces might indicates the fact that the river eroded those faces very quickly, hence with a great strength
        //The rivers flow might still be visible
        MyMesh::Normal up_direction = Vec3f(0,0,1);
        map<MyMesh::VertexHandle,ShaderParameters*> selected_faces ;
        MyMesh::VertexFaceIter vertex_face_circulator;
        float bins = 2;

        for(auto &entry : rivers_boundaries)
        {
            MyMesh::VertexHandle vertex_handle = entry.first;
            for( vertex_face_circulator = _mesh.vf_iter(vertex_handle);vertex_face_circulator.is_valid();++vertex_face_circulator)
            {
                    MyMesh::Normal mynormal = _mesh.normal(*vertex_face_circulator);

                    float dot_result = dot(mynormal,up_direction);
                    float resulting_angle_in_radians = acos(dot_result);
                    float resulting_angle_in_degree = resulting_angle_in_radians* (180.0/  M_PI);
                  //  resulting_angle_in_degree = 90 - resulting_angle_in_degree;
                    if(resulting_angle_in_degree <= _slope + _treshold && resulting_angle_in_degree >= _slope - _treshold)
                    {
                        MyMesh::Point point = _mesh.point(vertex_handle);
                        ShaderParameters* shader_parameters = new ShaderParameters(_id,10);
                        shader_parameters->AddParameter(ShaderParametersEnum::river,bins*floorf(point[2]/bins));
                        selected_faces.insert(pair<MyMesh::VertexHandle,ShaderParameters*>(vertex_handle,shader_parameters));
                    }
            }
        }
        return selected_faces;
    }*/
