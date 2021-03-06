#include "flowclassifier.h"
typedef std::mt19937 RANDOM;
using namespace  OpenMesh;
#define MULTCONSTANT 1000000


FlowClassifier::FlowClassifier(MyMesh mesh, SimulationDataMap simulation_data_map,int subdivision_level,float box_length, float scale, float step_size) : AClassifier(mesh),
    simulation_data_map(simulation_data_map),subdiv_levels(subdivision_level), scale(scale),step_size(step_size),box_length(box_length)
{
//simulation_data_wrapper = OpenMesh::getOrMakeProperty<MyMesh::VertexHandle,std::shared_ptr<SimulationData>>(_mesh, "simulation_data");
     _shader = std::shared_ptr<AShader>(new FlowShader(_id));
}




class splitInGroupsFunctorClass
{
    public:
    std::shared_ptr<FlowShader> getValue(MyMesh::VertexHandle vertex_handle,float depth,std::shared_ptr<FlowShader> val) {
        if(_map.count(vertex_handle) == 1 )
        {
            return _map[vertex_handle];
        }
        else
        {
            float confidence = ((float) _max_depth-depth)/(float) _max_depth;
            return  std::shared_ptr<FlowShader>(new FlowShader(_id,confidence,val->GetFlowNormal()));
        }

    }
    splitInGroupsFunctorClass(map<MyMesh::VertexHandle,std::shared_ptr<FlowShader>> map,int id,int max_depth): _map(map),_id(id),_max_depth(max_depth){}
            int operator() (MyMesh::VertexHandle vertex_handle) {
            if(_map.count(vertex_handle) == 1 )
                return 0;
            else
                return 1;
            }
    private:
            map<MyMesh::VertexHandle,std::shared_ptr<FlowShader>> _map;
            int _id;
            int _max_depth;

};

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
map<MyMesh::VertexHandle,T> FlowClassifier::BFS(int max_depth,map<MyMesh::VertexHandle,T> frontier_map,FuncType pred)
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
                     selected_nodes.insert(make_pair<MyMesh::VertexHandle,T>(*vertex_vertex_iterator,pred.getValue(vertex_vertex_iterator,new_cell.depth,(std::shared_ptr<FlowShader>)new_cell.val)));
                     queue.push_back(new_cell);
                    }
                }
            }
            visitedNodes.insert(cell);
        }

    }

return selected_nodes;
}




void FlowClassifier::selectFrontier(map<MyMesh::VertexHandle,std::shared_ptr<FlowShader>>& selected_vertices)
{


    map<MyMesh::VertexHandle,std::shared_ptr<FlowShader>> frontier;
    //float river;
    for (pair<MyMesh::VertexHandle,std::shared_ptr<FlowShader>> const& entry : selected_vertices)
    {
        VertexHandle vertex_handle = entry.first;
        //for each selected vertices iterate over its neighborhood, if a vertex that doesnt have the required property
        //is encountered then add the actual vertex in the frontier.
         for (MyMesh::VertexVertexIter vertex_vertex_iterator = _mesh.vv_iter(vertex_handle); vertex_vertex_iterator.is_valid(); ++vertex_vertex_iterator)
          {
             if(selected_vertices.count(*vertex_vertex_iterator)>0)
                 continue;
             else{
                 frontier.insert(entry);
                //entry.second->SetConfidence(0.3);
             }
//              auto simulation_data_wrapper = getOrMakeProperty<VertexHandle,SimulationData*>(_mesh, "simulation_data");
//              SimulationData* sd = simulation_data_wrapper[*vertex_vertex_iterator];
//              float river;
//              sd->getData(SimulationDataEnum::river,river);
//              if( river == 0)
//              {
//                  //insert in the map the actual vertex
//                  frontier.insert(make_pair(vertex_handle,river));
//                  break;
//              }
          }
    }
    int max_depth = 3;
    splitInGroupsFunctorClass functor(selected_vertices,_id,max_depth);
    auto temp_nodes = BFS(max_depth,frontier,functor);
    for(auto const entry : temp_nodes){
        selected_vertices.insert(entry);
    }
}

/*
 VertexEditTag FlowClassifier::GetVertexEditTag()
 {
     return _vertex_edit_tag;
 }
*/
map<MyMesh::VertexHandle,glm::vec3> FlowClassifier::selectFlowVertices(glm::vec3& min_bb,glm::vec3& max_bb)
 {


     MyMesh::VertexIter vertex_iterator,vertex_iterator_end(_mesh.vertices_end());
     for(int k=0;k<3;k++){
         min_bb[k] = INFINITY;
         max_bb[k] = -INFINITY;
     }

     map<MyMesh::VertexHandle,glm::vec3> flow_vertices;
     for(vertex_iterator=_mesh.vertices_begin();vertex_iterator != vertex_iterator_end;++vertex_iterator)
     {
    std::shared_ptr<SimulationData> sd;// = simulation_data_wrapper[*vertex_iterator];
    if(simulation_data_map.count(*vertex_iterator)>0){
        sd = simulation_data_map.at(*vertex_iterator);
         glm::vec3 sim_data_normal;
         sd->getData(SimulationDataEnum::flow_normal,sim_data_normal);
         sim_data_normal = glm::normalize(sim_data_normal);
         //invert part of the normals
         float tmp = sim_data_normal[0];
         sim_data_normal[0] = sim_data_normal[1];
         sim_data_normal[1] = tmp;

         if(glm::any(glm::isnan(sim_data_normal)))
                 continue;

         glm::vec3 up_normal(0,0,1);
         MyMesh::Normal openMesh_normal = _mesh.normal(vertex_iterator);
         glm::vec3 normal= glm::vec3(openMesh_normal[0],openMesh_normal [1],openMesh_normal[2]);

           float res =   (glm::dot(normal,up_normal));
         if(res> 0.85 ){//0.75
             continue;
         }
       //  if(normal[0]!=0 || normal[2]!=0 || normal[1]!=0)


             MyMesh::Point temp =_mesh.point(*vertex_iterator);
             for(int k = 0;k<3;k++){
                 if(temp[k]<min_bb[k]){
                     if(min_bb[k]<-100000)
                         std::cout<<"incoming error"<<std::endl;
                     min_bb[k] = temp[k];
                 }
                 if(temp[k]>max_bb[k]){
                     max_bb[k] = temp[k];
                     if(max_bb[k]>100000)
                         std::cout<<"incoming error"<<std::endl;
                 }
             }
             flow_vertices.insert(make_pair(*vertex_iterator,sim_data_normal));
        }
     }
     return flow_vertices;
 }

 void FlowClassifier::TemporaryUpdate(map<MyMesh::VertexHandle,std::shared_ptr<FlowShader>> selected_vertices)
 {
     auto shader_parameters_property = getOrMakeProperty<VertexHandle, std::shared_ptr<FlowShader>>(_mesh, "flow_shader_temp_propery");
     for (auto const& x : selected_vertices)
     {
         MyMesh::VertexHandle vertex_handle = x.first;
         std::shared_ptr<FlowShader> shader = x.second;
         shader_parameters_property[vertex_handle] = shader;
         //ShadersWrapper* shaders_wrapper = shader_parameters_property[vertex_handle];
         //shaders_wrapper->AddShaderParameters(shader);
         //shader_parameters_property[vertex_handle] = shader_parameter;

     }
 }

 std::shared_ptr<AShader> FlowClassifier::GetShader(){return _shader;}

void FlowClassifier::ClassifyVertices(std::vector<glm::vec3>& list_of_points, std::vector<std::shared_ptr<AShader>>& list_of_data, float& details,const pcl::PointCloud<pcl::PointXYZL>::Ptr point_cloud,
                                      const std::vector<std::shared_ptr<ShadersWrapper>>   list_of_shaders_wrappers)
{

    map<MyMesh::VertexHandle,std::shared_ptr<AShader>> selected_vertices;
    map<MyMesh::VertexHandle,std::shared_ptr<FlowShader>> temporary_selected_vertices;
    glm::vec3 min_bb;
    glm::vec3 max_bb;
    map<MyMesh::VertexHandle,glm::vec3> flow_vertices;


    for(int i=0;i<subdiv_levels;i++){

        auto temp_flow_vertices = selectFlowVertices(min_bb,max_bb);
         temporary_selected_vertices = ComputeShaderParameters(temp_flow_vertices);
        if(temporary_selected_vertices.size()==0)
            break;
        set<MyMesh::FaceHandle> set_of_faces;
       if(i==0){
        pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud1 ( new pcl::PointCloud<pcl::PointXYZLNormal>);
        cloud_input = cloud1;
        MyMesh::Point mesh_point;
        int j = 0;
              cloud_input->width = temp_flow_vertices.size();//numpoints
              cloud_input->height = 1;
              cloud_input->points.resize (cloud_input->width * cloud_input->height);
                for(auto const entry : temp_flow_vertices){
                    mesh_point =  _mesh.point((entry.first));
                    cloud_input->points[j].x = mesh_point[0];
                    cloud_input->points[j].y = mesh_point[1];
                    cloud_input->points[j].z = mesh_point[2];
                    glm::vec3 vec;
                    vec = entry.second;
                    cloud_input->points[j].normal_x =vec[0];// shader_param->getValue(0);
                    cloud_input->points[j].normal_y =vec[1];// shader_param->getValue(1);
                    cloud_input->points[j].normal_z =vec[2];// shader_param->getValue(2);
                    j++;

                }

    kdtree_input.setInputCloud (cloud_input);
        }


        for(auto const entry : temporary_selected_vertices){
            MyMesh::VertexFaceIter vertex_face_circulator;
                  vertex_face_circulator = _mesh.vf_iter(entry.first);
                  for( ;vertex_face_circulator.is_valid(); ++vertex_face_circulator){
                        set_of_faces.insert(*vertex_face_circulator);
                  }
        }
        //std::cout<<"ERR no subdiv is applied"<<std::endl;//dilit dis
        SubdividerAndInterpolator<MyMesh> catmull(set_of_faces,simulation_data_map
                                                  );
            catmull.attach(_mesh);
            catmull(1);
            catmull.detach();
//simulation_data_wrapper = OpenMesh::getOrMakeProperty<MyMesh::VertexHandle,std::shared_ptr<SimulationData>>(_mesh, "simulation_data");

    }
    if(temporary_selected_vertices.size()==0){
         std::cout<<"No point found FlowClassifier will stop."<<std::endl;
         return;
    }
     ulong total_dist = 0;
     ulong i = 0;
     for (MyMesh::EdgeIter e_it=_mesh.edges_begin(); e_it!=_mesh.edges_end(); ++e_it) {
          i++;
          HalfedgeHandle heh     = _mesh.halfedge_handle(*e_it, 0);
          HalfedgeHandle opp_heh = _mesh.halfedge_handle(*e_it, 1);

          VertexHandle   vh1( _mesh.to_vertex_handle(heh));
          VertexHandle   vh2( _mesh.to_vertex_handle(opp_heh));

          MyMesh::Point p1_mesh = _mesh.point(vh1);
          MyMesh::Point p2_mesh = _mesh.point(vh2);
          glm::vec3 p1(p1_mesh[0],p1_mesh[1],p1_mesh[2]);
          glm::vec3 p2(p2_mesh[0],p2_mesh[1],p2_mesh[2]);
            //total_dist += OpenMesh::VectorT
          float res = 0;
          glm::vec3 offset = p2-p1;
          res = dot(offset,offset);
          total_dist += roundf(res*100);
     }


    float avg = total_dist/(float)i;

    //auto flow_vertices = selectFlowVertices(min_bb,max_bb);
    flow_vertices = selectFlowVertices(min_bb,max_bb);
    temporary_selected_vertices = ComputeShaderParameters(flow_vertices);
    selectFrontier(temporary_selected_vertices);
    //float scale= subdiv_levels>0?  subdiv_levels:1;
    std::cout<<"secondary points"<<temporary_selected_vertices.size();
    selected_vertices = LIC(temporary_selected_vertices,min_bb,max_bb);

    for(pair<MyMesh::VertexHandle,std::shared_ptr<AShader>> entry : selected_vertices){
          MyMesh::Point point = _mesh.point(entry.first);
          list_of_points.push_back(glm::vec3(point[0],point[1],point[2]));
          list_of_data.push_back(entry.second);
    }
    details = 0.01;
    //return selected_vertices;
}



//
 map<MyMesh::VertexHandle,std::shared_ptr<FlowShader>> FlowClassifier::ComputeShaderParameters(map<MyMesh::VertexHandle,glm::vec3>  flow_vertices)
{
  //  map<MyMesh::VertexHandle,std::shared_ptr<FlowShader>> map;
    //glm::vec3 up(0,1,0);
     glm::vec3 up(0,0,1);
    map<MyMesh::VertexHandle,std::shared_ptr<FlowShader>> map;
   // auto shader_parameters_property = getOrMakeProperty<VertexHandle, std::shared_ptr<FlowShader>>(_mesh, "flow_shader_temp_propery");
    for(pair<MyMesh::VertexHandle,glm::vec3> entry : flow_vertices)
    {
        if(!glm::any(glm::isnan(entry.second)))
        {
            std::shared_ptr<FlowShader> shader = std::shared_ptr<FlowShader>(new FlowShader(_id,1.0f,entry.second));
          map.insert(std::make_pair(entry.first,shader));
        }
    }
return map;
}



void FlowClassifier::ContrastEnhancement(map<MyMesh::VertexHandle,std::shared_ptr<AShader>>& map,std::vector<int> Pdf,int z)
{
    int L = Pdf.size();
    float value_counter= 0;
    float a = 0;
    float b = 1;
    float c = 0;
    float d = 1;
    int f;
    if(z>0){
        f = 2;
    }else{
        f=1;
    }



    int num_vertices;
    if(z==0){
        num_vertices= cloud->points.size();
    }else{
        num_vertices= map.size();
    }
    for(int i = 0;i<L;i++)
    {
        value_counter += Pdf[i];
        if ( value_counter < num_vertices *  f /100  )
            if(i!=0)
                c =(float) (i-1)/L;
            else
                c= (float) i/L;
        if ( value_counter > num_vertices* (100-f) /100  )
        {
            if(i!=num_vertices)
                d =(float) (i+1)/L;
            else
                d= (float) i/L;
            break;
        }
    }

    int out_of_range_vertices= 0;
    std::shared_ptr<FlowShader> shader;
int i = 0;
std::cout<<"c "<< c << " d "<< d << std::endl;

if(z==1){
    for (auto const& x : map)
    {
        i++;
      if(x.second != nullptr)
          shader = std::static_pointer_cast<FlowShader> (x.second);
      else
          map[x.first] = shader;
      float val = shader->GetLicValue();
      val = (val - c)*((b-a)/(d-c)) + a;
      if(val>1 || val<0){
          out_of_range_vertices++;
      }
      shader->SetLicValue(val);
    }
}else{

    for(int j = 0;j<cloud->points.size();j++ ){

        float val = cloud->points[j].label/(float)MULTCONSTANT;
        val = (val - c)*((b-a)/(d-c)) + a;
        if(val>1 || val<0)
          out_of_range_vertices++;
        val = val > 1 ? 1 : val;
        val = val < 0 ? 0 : val;
        cloud->points[j].label = val*MULTCONSTANT;
    }
}
std::cout<<"Contrast Enhanchement"<<std::endl;
    std::cout<<" "<<out_of_range_vertices << " vertices were out of range over "<< num_vertices<<" which is "<< out_of_range_vertices/(float)num_vertices*100<<" %"<<std::endl;

}

glm::vec3 FlowClassifier::InterpolateFlowNormal( MyMesh::Point actual_point){
    int K = 3;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    glm::vec3 flow_vector(0,0,0);
    pcl::PointXYZLNormal new_point;
    pcl::PointXYZLNormal temp_point;
pcl::PointXYZLNormal searchPoint_tmp;
    searchPoint_tmp.x = actual_point[0];
    searchPoint_tmp.y = actual_point[1];
    searchPoint_tmp.z = actual_point[2];
  searchPoint_tmp.x = searchPoint_tmp.x<0.00001 ? 0 : searchPoint_tmp.x;
  searchPoint_tmp.y = searchPoint_tmp.y<0.00001 ? 0 : searchPoint_tmp.y;
  searchPoint_tmp.z = searchPoint_tmp.z<0.00001 ? 0 : searchPoint_tmp.z;
  new_point.normal_x = 0;
  new_point.normal_y = 0;
  new_point.normal_z = 0;
    if ( kdtree_input.nearestKSearch (searchPoint_tmp, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
        for(int b = 0 ;b<pointIdxNKNSearch.size();b++){
            temp_point =  cloud_input->points[ pointIdxNKNSearch[b]];
            new_point.normal_x += temp_point.normal_x;
            new_point.normal_y += temp_point.normal_y;
            new_point.normal_z += temp_point.normal_z;
            new_point.x += temp_point.x;
            new_point.y += temp_point.y;
            new_point.z += temp_point.z;
        }
        new_point.normal_x /= pointIdxNKNSearch.size();
        new_point.normal_y /= pointIdxNKNSearch.size();
        new_point.normal_z /= pointIdxNKNSearch.size();
        new_point.x /= pointIdxNKNSearch.size();
        new_point.y /= pointIdxNKNSearch.size();
        new_point.z /= pointIdxNKNSearch.size();
        flow_vector[0] = new_point.normal_x;
        flow_vector[1] = new_point.normal_y;
        flow_vector[2] = new_point.normal_z;
    }
    return flow_vector;
}


map<MyMesh::VertexHandle,std::shared_ptr<AShader>> FlowClassifier:: LIC(map<MyMesh::VertexHandle,std::shared_ptr<FlowShader>>const map,glm::vec3 min_bb,glm::vec3 max_bb)
{

    float longest_dimension = -INFINITY;
    for(int k=0;k<3;k++){
        float dim = max_bb[k]-min_bb[k];
        if(dim>longest_dimension){
            longest_dimension = dim;
        }
    }

   box_length = longest_dimension/4;

    //float step_size = 0.15;//0.26;0.5;//scale;
    float frequency = scale;//200;//longest_dimension/2;//scale*2;
 //  box_length = step_size;// 20;//150;
    FastNoise noise;
    std::map<MyMesh::VertexHandle,std::shared_ptr<AShader>> output_map;
   std::map<MyMesh::VertexHandle,std::shared_ptr<AShader>> intermediate_map;
   //float prob = 0.001;
   //int max_rnd = 100;
   //std::uniform_int_distribution<int> rnd_int(0,max_rnd);

    std::cout<<"InterpolationDone"<<std::endl;
    MyMesh::Point actual_point;
    MyMesh::Point circulator_point;
    MyMesh::Point offset;
    glm::vec3 flow_vector(0,0,0);
    MyMesh::VertexVertexIter vvit;
    MyMesh::VertexVertexIter next_point;
    ShadersWrapper*  shader_wrapper;
    std::shared_ptr<FlowShader> flow_shader;
   // auto shader_parameters_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, std::shared_ptr<FlowShader>>(_mesh, "flow_shader_temp_propery");

    float squared_dist;
    float min_dist = INFINITY;
    int k = 0;
    int L = 5000;
    uint u=0;
    int quantizied_level;
    std::vector<int> Pdf(L,0) ;

        RANDOM rnd_seed;
 //       std::uniform_real_distribution<float> rndFloat(0,3);


    float total_value = 0;

    int num_vertices = map.size();
    int percentual_part = num_vertices/100;
    //MyMesh::VertexIter mesh_vertex_iterator;
    MyMesh::VertexHandle mesh_vertex_handle;


    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud1 ( new pcl::PointCloud<pcl::PointXYZLNormal>);
      cloud = cloud1;
int j = 0;
    cloud->width = map.size();//numpoints
    cloud->height = 1;
    cloud->points.resize (cloud->width * cloud->height);




int f = 0;
for(int z = 0;z<2;z++){
    Pdf = std::vector<int>(L,0);

    if(z==1)
        box_length /= 2;//step_size;// 20*scale;

    std::cout<<"iteration "<<z<<std::endl;
    k = 0;
    for(pair<MyMesh::VertexHandle,std::shared_ptr<FlowShader>> entry : map)
    {
        //mesh_vertex_iterator = entry.first;

        mesh_vertex_handle = entry.first;
     if(k%percentual_part == 0)
         std::cout<< "LIC map "<<k / percentual_part<< " %  completed"<< std::endl;
     k++;
     float val = 0;
     int w = 0;
    flow_shader = entry.second;//shader_parameters_data_wrapper[mesh_vertex_handle];
    if(flow_shader==nullptr)
         continue;
    else{
    for(int j=0;j<2;j++)
    {
        float samepoint_iter = 1;
        actual_point = _mesh.point(mesh_vertex_handle);


            flow_shader = map.at(mesh_vertex_handle);// [mesh_vertex_handle];// shader_parameters_data_wrapper[mesh_vertex_handle];
            pcl::PointXYZLNormal new_point;
            pcl::PointXYZLNormal temp_point;
        pcl::PointXYZLNormal searchPoint_tmp;
        searchPoint_tmp.x = actual_point[0];
        searchPoint_tmp.y = actual_point[1];
        searchPoint_tmp.z = actual_point[2];

        int K = 3;//or 27



        flow_vector = flow_shader->GetFlowNormal();
        if(glm::any(glm::isnan(flow_vector)))
        {
            std::cout<<"Error flow_vect is empty AT THE BEGGINING"<<std::endl;
        }
        vvit = _mesh.vv_iter(mesh_vertex_handle);

         for(float i = 0;i<box_length;i+= step_size)
             {
             f++;
             float n;



             if(z>0){

                pcl::PointXYZLNormal searchPoint;
                                          searchPoint.x = actual_point[0];
                                          searchPoint.y = actual_point[1];
                                          searchPoint.z = actual_point[2];
                                    pcl::PointXYZLNormal new_point;
                                          int K = 3;//or 27
                                          std::vector<int> pointIdxNKNSearch(K);
                                          std::vector<float> pointNKNSquaredDistance(K);
                                          if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
                                             float tmp_val = 0;
                                              for(uint b = 0 ;b<pointIdxNKNSearch.size();b++){
                                                  new_point =  cloud->points[ pointIdxNKNSearch[b]];
                                                  tmp_val += new_point.label/(float )MULTCONSTANT;
                                              }
                                              n = tmp_val/(float) pointIdxNKNSearch.size();
n = n>1? 0 : n;
n = n<0? 0 : n;
                                           }
             }else{
                 n = noise.GetNoise(actual_point[0] * frequency ,
                                          actual_point[1] * frequency,
                                          actual_point[2] * frequency);
             }
                 if(j==1)
                     flow_vector = -flow_vector;

                  actual_point[0] += flow_vector[0]*step_size;
                  actual_point[1] += flow_vector[1]*step_size;
                  actual_point[2] += flow_vector[2]*step_size;
                  squared_dist = 0;
                  min_dist = INFINITY;

                  val+=n;
                  w++;

                      glm::vec3 next_flow_vector ;
                      std::shared_ptr<FlowShader> temp_shader = nullptr;
              flow_vector = InterpolateFlowNormal(actual_point);



             }
     }
  val = val/ w;

  //float a  = 0;
  //float b = 1;
  //float c = 0.43;
  //float d = 0.57;
  //
  //
  if(z==0)
    val = 0.5 + 0.5 * val ;
  //val = (val - c)*((b-a)/(d-c)) + a;
  val = val > 1 ? 1 : val;
  val = val < 0 ? 0 : val;
  total_value += val;
  quantizied_level =  roundf(val*L);
  Pdf[quantizied_level]++;
  actual_point = _mesh.point(mesh_vertex_handle);
  std::shared_ptr<FlowShader> old_flow_shader ;
//  if(z==1){
//    old_flow_shader = (std::shared_ptr<FlowShader>) map[mesh_vertex_iterator];
//  }else
   old_flow_shader =  entry.second;// shader_parameters_data_wrapper[mesh_vertex_handle];
  //glm::vec3 victor = old_flow_shader->GetFlowNormal();
//std::pair<MyMesh::VertexHandle,std::shared_ptr<AShader>> pair= std::make_pair(mesh_vertex_iterator,new_flow_shader);
  //if(z==1){
  //    map1.insert(std::make_pair(mesh_vertex_iterator,new_flow_shader));
  //
  //}else{
  //if(intermediate_map.count(mesh_vertex_handle)>0)
   if(z>0){


    glm::vec3 new_flow_normal = InterpolateFlowNormal(actual_point);

      std::shared_ptr<AShader> new_flow_shader =std::shared_ptr<AShader> (new FlowShader(_id,old_flow_shader->GetConfidence(),new_flow_normal,val));
      output_map.insert(std::make_pair(mesh_vertex_handle,new_flow_shader));

//      ((std::shared_ptr<FlowShader>) map_output[mesh_vertex_handle])->SetLicValue(val);
    }
  else{
      if(old_flow_shader!=nullptr && !glm::any(glm::isnan(old_flow_shader->GetFlowNormal()))){
//      std::shared_ptr<AShader> new_flow_shader = new FlowShader(_id,old_flow_shader->GetConfidence(),old_flow_shader->GetFlowNormal(),val);
//          intermediate_map.insert(std::make_pair(mesh_vertex_handle,new_flow_shader));
//



      cloud->points[u].x = actual_point[0];
      cloud->points[u].y = actual_point[1];
      cloud->points[u].z = actual_point[2];
      glm::vec3 vec;
      vec = old_flow_shader->GetFlowNormal();
      cloud->points[u].normal_x =vec[0];// shader_param->getValue(0);
      cloud->points[u].normal_y =vec[1];// shader_param->getValue(1);
      cloud->points[u].normal_z =vec[2];// shader_param->getValue(2);
      int tmp = val*MULTCONSTANT;
      cloud->points[u].label = tmp;
    u++;

//          std::shared_ptr<AShader> new_flow_shader = new FlowShader(_id,old_flow_shader->GetConfidence(),old_flow_shader->GetFlowNormal(),val);
    }
  }

  }
}
    if(z==0){
        kdtree.setInputCloud (cloud);
        ContrastEnhancement(intermediate_map,Pdf,z);
    }
   if(z==1)
    ContrastEnhancement(output_map,Pdf,z);
    if(output_map.size()==0)
        output_map = intermediate_map;

}

std::cout<<"f is "<<f<<std::endl;
return output_map;


}
/*
void FlowClassifier::ComputeOutputs()
{

    FastNoise noise;
    SubdividerAndInterpolator<MyMesh> catmull;
    if(_subdiv_levels>0)
    {
        catmull.attach(_mesh);
        catmull(_subdiv_levels);
        catmull.detach();

    }

auto shader_parameters_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, ShadersWrapper*>(_mesh, "shader_parameters");


std::cout<<"InterpolationDone"<<std::endl;
MyMesh::Point actual_point;
MyMesh::Point circulator_point;
MyMesh::Point offset;
glm::vec3 flow_vector(0,0,0);
MyMesh::VertexVertexIter vvit;
MyMesh::VertexVertexIter next_point;
ShaderParameters  shader_param;
ShadersWrapper*  shader_wrapper;

float dist;
float min_dist = INFINITY;
int k = 0;
int L = 1000;
int quantizied_level;
std::vector<int> Pdf(L,0) ;
MyMesh::VertexIter mesh_vertex_iterator;
MyMesh::VertexIter vertex_iterator_end(_mesh.vertices_end());


int num_vertices = _mesh.n_vertices();
int percentual_part = num_vertices/100;
k = 0;
float total_value = 0;
mesh_vertex_iterator=_mesh.vertices_begin();
   for(;mesh_vertex_iterator!= vertex_iterator_end;++mesh_vertex_iterator,k++)
   {
    if(k%percentual_part == 0)
        std::cout<< "LIC map "<<k / percentual_part<< " %  completed"<< std::endl;

       //fw advection
float val = 0;
int w = 0;

 for(int j=0;j<2;j++)
 {
     float samepoint_iter = 1;
     actual_point = _mesh.point(*mesh_vertex_iterator);

      shader_wrapper = shader_parameters_data_wrapper[mesh_vertex_iterator];
      std::vector<ShaderParameters> list;
      shader_wrapper->GetListOfShaders(list);
      for(ShaderParameters sp : list)
      {
          sp.GetParameter(ShaderParametersEnum::flow_normal,flow_vector);

      }
     // if(shader_param == nullptr)
     //     std::cout<<"Err shader param is empty"<<std::endl;
     // else
     //   flow_vector = glm::vec3( shader_param->getValue(1),shader_param->getValue(2),shader_param->getValue(3));

     vvit = _mesh.vv_iter(mesh_vertex_iterator);
     // = _mesh.vv_iter(*mesh_vertex_iterator);

        for(float i = 0;i<box_length;i++)
            {
                if(j==1)
                    flow_vector = -flow_vector;
               //  MyMesh::VertexVertexIter vvit = _mesh.vv_iter(next_point);
                 actual_point[0] += flow_vector[0]*step_size;
                 actual_point[1] += flow_vector[1]*step_size;
                 actual_point[2] += flow_vector[2]*step_size;

                 dist = 0;
                 min_dist = INFINITY;
                 if(i!=0 && samepoint_iter == 1)
                     vvit = _mesh.vv_iter(vvit);
                 else
                      vvit = _mesh.vv_iter(mesh_vertex_iterator);
                 for ( ;vvit.is_valid(); ++vvit)
                 {
                     circulator_point = _mesh.point(*vvit);
                    offset = actual_point - circulator_point;
                     dist = dot(offset,offset);
                    if(dist<min_dist)
                    {
                        min_dist = dist;
                        next_point = vvit;
                    }
                 }
                 float n = noise.GetPerlin(actual_point[0] * frequency ,
                                           actual_point[1] * frequency,
                                           actual_point[2] * frequency);


//                 if( next_point.is_valid())
//                    actual_point = _mesh.point(*next_point);

                 val+=n;
                 w++;
                 if( next_point.is_valid())
                 {
                     glm::vec3 next_flow_vector ;
                     shader_wrapper = shader_parameters_data_wrapper[mesh_vertex_iterator];
                     shader_wrapper->GetListOfShaders(list);
                     for(ShaderParameters sp : list)
                     {
                         sp.GetParameter(ShaderParametersEnum::flow_normal,next_flow_vector);

                     }


                  //   shader_param = shader_parameters_data_wrapper[next_point];
                  //   glm::vec3 next_flow_vector = glm::vec3( shader_param->getValue(1),shader_param->getValue(2),shader_param->getValue(3));
//                     flow_vector = flow_vector* (1-1/(min_dist+1)) + next_flow_vector*(1-min_dist/(min_dist+1));
                     flow_vector = next_flow_vector;
                     //vvit  = _mesh.vv_iter(next_point);
                     vvit = next_point;
                    samepoint_iter = 1;
                 }
                 else
                 {
                    samepoint_iter++;
                 }
            }
    }
 val = val/ w;

 //float a  = 0;
 //float b = 1;
 //float c = 0.43;
 //float d = 0.57;
 //
 //
 val = 0.5 + 0.5 * val ;
 //val = (val - c)*((b-a)/(d-c)) + a;
 val = val > 1 ? 1 : val;
 total_value += val;
 quantizied_level =  roundf(val*L);
Pdf[quantizied_level]++;
 actual_point = _mesh.point(*mesh_vertex_iterator);
 shader_param = shader_parameters_data_wrapper[mesh_vertex_iterator];
 shader_param.AddParameter(ShaderParametersEnum::LIC,val);

 //std::cout<<"val "<<shader_param->getValue(5)<<std::endl;
 shader_parameters_data_wrapper[mesh_vertex_iterator] = shader_param;


 new_point_cloud->points[k].x = actual_point[0] ;
 new_point_cloud->points[k].y = actual_point[1] ;
 new_point_cloud->points[k].z = actual_point[2] ;
 new_point_cloud->points[k].intensity = val;

   }


   //Contrast Enhanchement

   float a  = 0;
   float b = 1;
   float c=0; //= 0.43;
   float d=1; //= 0.57;
   k = 0;
   float value_counter = 0;

   for(int i = 0;i<L;i++)
   {
       value_counter += Pdf[i];
       if ( value_counter < num_vertices * 5 /100  )
           if(i!=0)
               c =(float) (i-1)/L;
           else
               c= (float) i/L;
       if ( value_counter > num_vertices* 95 /100  )
       {
           if(i!=num_vertices)
               d =(float) (i+1)/L;
           else
               d= (float) i/L;
           break;
       }
   }

   mesh_vertex_iterator=_mesh.vertices_begin();
   shader_parameters_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, ShaderParameters*>(_mesh, "shader_parameters");



      for(;mesh_vertex_iterator!= vertex_iterator_end;++mesh_vertex_iterator)
      {
          //shader_param = shader_parameters_data_wrapper[mesh_vertex_iterator];

          shader_wrapper = shader_parameters_data_wrapper[mesh_vertex_iterator];
          std::vector<ShaderParameters> list;
          shader_wrapper->GetListOfShaders(list);
           float val;
          for(ShaderParameters sp : list)
          {
                shader_param.GetParameter(ShaderParametersEnum::LIC,val);
          }


        val = (val - c)*((b-a)/(d-c)) + a;

        shader_param->setValue(5,val);
        shader_parameters_data_wrapper[mesh_vertex_iterator] = shader_param;
        new_point_cloud->points[k].intensity = val;
      }

      shader_parameters_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, ShaderParameters*>(_mesh, "shader_parameters");


std::cout<<" c " << c << " d "<< d << std::endl;
box_length /= 2 ;
std::fill(Pdf.begin(), Pdf.end(), 0);
   //second pass
  std::cout<<"------SECOND PASS---------"<<std::endl;
    mesh_vertex_iterator=_mesh.vertices_begin();
    k = 0;
   for(;mesh_vertex_iterator!= vertex_iterator_end;++mesh_vertex_iterator,k++)
   {
    if(k%percentual_part == 0)
        std::cout<< "LIC map "<<k / percentual_part<< " %  completed"<< std::endl;

       //fw advection
float val = 0;
int w = 0;



 for(int j=0;j<2;j++)
 {
     float samepoint_iter = 1;
    actual_point = _mesh.point(*mesh_vertex_iterator);

//      shader_param = shader_parameters_data_wrapper[mesh_vertex_iterator];
      shader_wrapper = shader_parameters_data_wrapper[mesh_vertex_iterator];
      std::vector<ShaderParameters> list;
      shader_wrapper->GetListOfShaders(list);
      for(ShaderParameters sp : list)
      {
          sp.GetParameter(ShaderParametersEnum::flow_normal,flow_vector);

      }
      //previously
      //if(shader_param == nullptr)
      //    std::cout<<"Err shader param is empty"<<std::endl;
      //else
      //  flow_vector = glm::vec3( shader_param->getValue(1),shader_param->getValue(2),shader_param->getValue(3));

     vvit = _mesh.vv_iter(mesh_vertex_iterator);
     // = _mesh.vv_iter(*mesh_vertex_iterator);

        for(float i = 0;i<box_length;i++)
            {
                if(j==1)
                    flow_vector = -flow_vector;
               //  MyMesh::VertexVertexIter vvit = _mesh.vv_iter(next_point);
                 actual_point[0] += flow_vector[0]*step_size;
                 actual_point[1] += flow_vector[1]*step_size;
                 actual_point[2] += flow_vector[2]*step_size;

                 dist = 0;
                 min_dist = INFINITY;
                 if(i!=0 && samepoint_iter == 1)
                     vvit = _mesh.vv_iter(vvit);
                 else
                      vvit = _mesh.vv_iter(mesh_vertex_iterator);
                 for ( ;vvit.is_valid(); ++vvit)
                 {
                     circulator_point = _mesh.point(*vvit);
                    offset = actual_point - circulator_point;
                     dist = dot(offset,offset);
                    if(dist<min_dist)
                    {
                        min_dist = dist;
                        next_point = vvit;
                    }
                 }
//                 float n =0;// shader_param->getValue(5);
                 float n = noise.GetPerlin(actual_point[0] * frequency ,
                                           actual_point[1] * frequency,
                                           actual_point[2] * frequency);
           //     if(n<0 || n>1)
           //         std::cout<<"Err in getVal"<< n <<std::endl;
//                 if( next_point.is_valid())
//                    actual_point = _mesh.point(*next_point);

                 val+=n;
                 w++;
                 if( next_point.is_valid())
                 {
                     //shader_param = shader_parameters_data_wrapper[next_point];
                     shader_wrapper = shader_parameters_data_wrapper[next_point];
                     std::vector<ShaderParameters> list;
                     shader_wrapper->GetListOfShaders(list);
                     glm::vec3 next_flow_vector;
                      for(ShaderParameters sp : list)
                     {
                           shader_param.GetParameter(ShaderParametersEnum::flow_normal,next_flow_vector);
                     }
                     // = glm::vec3( shader_param->getValue(1),shader_param->getValue(2),shader_param->getValue(3));
                     //flow_vector = flow_vector* (1-1/(min_dist+1)) + next_flow_vector*(1-min_dist/(min_dist+1));
                     flow_vector = next_flow_vector;
                     //vvit  = _mesh.vv_iter(next_point);
                     vvit = next_point;
                     samepoint_iter = 1;
                  }
                  else
                  {
                     samepoint_iter++;
                  }

            }
    }

 val = val/ w;
 val = 0.5 + 0.5 * val ;
 if(val<0 || val>1)
     std::cout<<"Err in Val"<< val <<std::endl;
 val = val > 1 ? 1 : val;
 quantizied_level =  roundf(val*L);
Pdf[quantizied_level]++;
 actual_point = _mesh.point(*mesh_vertex_iterator);
 shader_wrapper = shader_parameters_data_wrapper[mesh_vertex_iterator];
 shader_param->setValue(5,val);

//shader_parameters_data_wrapper[mesh_vertex_iterator] = shader_param;

   }
   value_counter = 0;

   for(int i = 0;i<L;i++)
   {
       value_counter += Pdf[i];
       if ( value_counter < num_vertices * 2 /100  )
           if(i!=0)
               c =(float) (i-1)/L;
           else
               c= (float) i/L;
       if ( value_counter > num_vertices* 98 /100  )
       {
           if(i!=num_vertices)
               d =(float) (i+1)/L;
           else
               d= (float) i/L;
           break;
       }
   }

   mesh_vertex_iterator=_mesh.vertices_begin();
   k = 0;
     for(;mesh_vertex_iterator!= vertex_iterator_end;++mesh_vertex_iterator,k++)
      {
          shader_param = shader_parameters_data_wrapper[mesh_vertex_iterator];
        float val = shader_param->getValue(5);
        val = (val - c)*((b-a)/(d-c)) + a;
        shader_param->setValue(5,val);

      }



   Outputcloud = new_point_cloud;
   kdtree_output.setInputCloud (Outputcloud);

}
*/

/*
void FlowClassifier::LIC2(float box_length,float frequency,float step_size,MyMesh mesh)
{
    _mesh = mesh;

    FastNoise noise;
   pcl::PointCloud<pcl::PointXYZI>::Ptr new_point_cloud ( new pcl::PointCloud<pcl::PointXYZI>);

SubdividerAndInterpolator<MyMesh> catmull;


if(_subdiv_levels>0)
{
    catmull.attach(_mesh);
    catmull(_subdiv_levels);
    catmull.detach();

}

auto shader_parameters_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, ShadersWrapper*>(_mesh, "shader_parameters");


std::cout<<"InterpolationDone"<<std::endl;
 new_point_cloud->width = _mesh.n_vertices();//numpoints
 new_point_cloud->height = 1;
 new_point_cloud->points.resize (new_point_cloud->width * new_point_cloud->height);
MyMesh::Point actual_point;
MyMesh::Point circulator_point;
MyMesh::Point offset;
glm::vec3 flow_vector(0,0,0);
MyMesh::VertexVertexIter vvit;
MyMesh::VertexVertexIter next_point;
ShaderParameters  shader_param;
ShadersWrapper*  shader_wrapper;

float dist;
float min_dist = INFINITY;
int k = 0;
int L = 1000;
int quantizied_level;
std::vector<int> Pdf(L,0) ;
std::vector<int> Cdf(L,0) ;
//InterpolateData(_mesh);
float cdf_min = INFINITY;
MyMesh::VertexIter mesh_vertex_iterator;
MyMesh::VertexIter vertex_iterator_end(_mesh.vertices_end());


int num_vertices = _mesh.n_vertices();
int percentual_part = num_vertices/100;
k = 0;
float total_value = 0;
mesh_vertex_iterator=_mesh.vertices_begin();
   for(;mesh_vertex_iterator!= vertex_iterator_end;++mesh_vertex_iterator,k++)
   {
    if(k%percentual_part == 0)
        std::cout<< "LIC map "<<k / percentual_part<< " %  completed"<< std::endl;

       //fw advection
float val = 0;
int w = 0;

 for(int j=0;j<2;j++)
 {
     float samepoint_iter = 1;
     actual_point = _mesh.point(*mesh_vertex_iterator);

      shader_wrapper = shader_parameters_data_wrapper[mesh_vertex_iterator];
      std::vector<ShaderParameters> list;
      shader_wrapper->GetListOfShaders(list);
      for(ShaderParameters sp : list)
      {
          sp.GetParameter(ShaderParametersEnum::flow_normal,flow_vector);

      }
     // if(shader_param == nullptr)
     //     std::cout<<"Err shader param is empty"<<std::endl;
     // else
     //   flow_vector = glm::vec3( shader_param->getValue(1),shader_param->getValue(2),shader_param->getValue(3));

     vvit = _mesh.vv_iter(mesh_vertex_iterator);
     // = _mesh.vv_iter(*mesh_vertex_iterator);

        for(float i = 0;i<box_length;i++)
            {
                if(j==1)
                    flow_vector = -flow_vector;
               //  MyMesh::VertexVertexIter vvit = _mesh.vv_iter(next_point);
                 actual_point[0] += flow_vector[0]*step_size;
                 actual_point[1] += flow_vector[1]*step_size;
                 actual_point[2] += flow_vector[2]*step_size;

                 dist = 0;
                 min_dist = INFINITY;
                 if(i!=0 && samepoint_iter == 1)
                     vvit = _mesh.vv_iter(vvit);
                 else
                      vvit = _mesh.vv_iter(mesh_vertex_iterator);
                 for ( ;vvit.is_valid(); ++vvit)
                 {
                     circulator_point = _mesh.point(*vvit);
                    offset = actual_point - circulator_point;
                     dist = dot(offset,offset);
                    if(dist<min_dist)
                    {
                        min_dist = dist;
                        next_point = vvit;
                    }
                 }
                 float n = noise.GetPerlin(actual_point[0] * frequency ,
                                           actual_point[1] * frequency,
                                           actual_point[2] * frequency);


//                 if( next_point.is_valid())
//                    actual_point = _mesh.point(*next_point);

                 val+=n;
                 w++;
                 if( next_point.is_valid())
                 {
                     glm::vec3 next_flow_vector ;
                     shader_wrapper = shader_parameters_data_wrapper[mesh_vertex_iterator];
                     shader_wrapper->GetListOfShaders(list);
                     for(ShaderParameters sp : list)
                     {
                         sp.GetParameter(ShaderParametersEnum::flow_normal,next_flow_vector);

                     }


                  //   shader_param = shader_parameters_data_wrapper[next_point];
                  //   glm::vec3 next_flow_vector = glm::vec3( shader_param->getValue(1),shader_param->getValue(2),shader_param->getValue(3));
//                     flow_vector = flow_vector* (1-1/(min_dist+1)) + next_flow_vector*(1-min_dist/(min_dist+1));
                     flow_vector = next_flow_vector;
                     //vvit  = _mesh.vv_iter(next_point);
                     vvit = next_point;
                    samepoint_iter = 1;
                 }
                 else
                 {
                    samepoint_iter++;
                 }
            }
    }
 val = val/ w;

 //float a  = 0;
 //float b = 1;
 //float c = 0.43;
 //float d = 0.57;
 //
 //
 val = 0.5 + 0.5 * val ;
 //val = (val - c)*((b-a)/(d-c)) + a;
 val = val > 1 ? 1 : val;
 total_value += val;
 quantizied_level =  roundf(val*L);
Pdf[quantizied_level]++;
 actual_point = _mesh.point(*mesh_vertex_iterator);
 shader_param = shader_parameters_data_wrapper[mesh_vertex_iterator];
 shader_param.AddParameter(ShaderParametersEnum::LIC,val);

 //std::cout<<"val "<<shader_param->getValue(5)<<std::endl;
 shader_parameters_data_wrapper[mesh_vertex_iterator] = shader_param;


 new_point_cloud->points[k].x = actual_point[0] ;
 new_point_cloud->points[k].y = actual_point[1] ;
 new_point_cloud->points[k].z = actual_point[2] ;
 new_point_cloud->points[k].intensity = val;
}}

*/
