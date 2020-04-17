#include "LICMap.h"


LICMap::LICMap(MyMesh mesh,int subdiv_level) : _subdiv_levels(subdiv_level)
{



    // kd-tree used for finding neighbours
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdt;

      pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud1 ( new pcl::PointCloud<pcl::PointXYZLNormal>);
      cloud = cloud1;
      // Generate pointcloud data
      /*
      cloud->width = mesh.n_vertices();//numpoints
      cloud->height = 1;
      cloud->points.resize (cloud->width * cloud->height);


      auto shader_parameters_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, ShaderParameters*>(mesh, "shader_parameters");
      int i=0;
      float point[3], normal[3];
      float radius = 0.02;
      MyMesh::Point mesh_point;

      MyMesh::VertexIter vertex_handle;
      MyMesh::VertexIter vertex_iterator_end(mesh.vertices_end());

      /*
       * To Refactor this part is important!!! Here it works because we already know
       * in which position the direction of the flow is stored
        */
      /*
      int j = 0;



      for(vertex_handle= mesh.vertices_begin();vertex_handle != vertex_iterator_end;++vertex_handle)
      {
          ShaderParameters* const shader_param = shader_parameters_data_wrapper[vertex_handle];

          mesh_point =  mesh.point(*vertex_handle);
          cloud->points[j].x = mesh_point[0];
          cloud->points[j].y = mesh_point[1];
          cloud->points[j].z = mesh_point[2];
          glm::vec3 vec;
          vec[0] = shader_param->getValue(1);
          vec[1] = shader_param->getValue(2);
          vec[2] = shader_param->getValue(3);
          if(vec[0] != 0 && vec[1] != 0 && vec[2] != 0)
            vec = glm::normalize (vec);
            else
          {
              vec[0] = 0;
              vec[1] = 0;
              vec[2] = 0;
          }
          cloud->points[j].normal_x =vec[0];// shader_param->getValue(0);
          cloud->points[j].normal_y =vec[1];// shader_param->getValue(1);
          cloud->points[j].normal_z =vec[2];// shader_param->getValue(2);
          j++;
      }
      //OpenMesh::Subdivider::Uniform::CatmullClarkT<MyMesh> catmull;
    SubdividerAndInterpolator<MyMesh> catmull;

     // OpenMesh::Subdivider::Uniform::SubdividerAndInterpolator<MyMesh> catmull;
      // Execute 1 subdivision steps
         /* kdtree.setInputCloud (cloud);
          int num_iterations = 1;
      //iterate for number of desired subdivisions
      catmull.attach(mesh);
      catmull( subdiv_level );
      catmull.detach();
      pcl::PointXYZLNormal new_point;
      cloud->width = mesh.n_vertices();//numpoints
      cloud->height = 1;
      cloud->points.resize (cloud->width * cloud->height);
      for(vertex_handle= mesh.vertices_begin();vertex_handle!= vertex_iterator_end;++vertex_handle)
      {
          mesh_point =  mesh.point(*vertex_handle);
          pcl::PointXYZLNormal searchPoint;
          searchPoint.x = mesh_point[0];
          searchPoint.y = mesh_point[1];
          searchPoint.z = mesh_point[2];

          int K = 1;//or 27
          std::vector<int> pointIdxNKNSearch(K);
          std::vector<float> pointNKNSquaredDistance(K);
          if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
          {
            // NN as interpolation
            new_point =  cloud->points[ pointIdxNKNSearch[0]];
            //glm::vec3 flow_dir;
            //flow_dir[0] = point.normal_x;
            //flow_dir[1] = point.normal_y;
            //flow_dir[2] = point.normal_z;

          }

          cloud->points[j].x = mesh_point[0];
          cloud->points[j].y = mesh_point[1];
          cloud->points[j].z = mesh_point[2];
          cloud->points[j].normal_x = new_point.normal_x;
          cloud->points[j].normal_y = new_point.normal_y;
          cloud->points[j].normal_z = new_point.normal_z;
          j++;
      }


      kdtree.setInputCloud (cloud);
*/



  /*    pcl::PointXYZRGB searchPoint;

      searchPoint.x = 1024.0f * rand () / (RAND_MAX + 1.0f);
      searchPoint.y = 1024.0f * rand () / (RAND_MAX + 1.0f);
      searchPoint.z = 1024.0f * rand () / (RAND_MAX + 1.0f);

      // K nearest neighbor search

      int K = 10;

      std::vector<int> pointIdxNKNSearch(K);
      std::vector<float> pointNKNSquaredDistance(K);

      std::cout << "K nearest neighbor search at (" << searchPoint.x
                << " " << searchPoint.y
                << " " << searchPoint.z
                << ") with K=" << K << std::endl;

      if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
      {
        for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
          std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x
                    << " " << cloud->points[ pointIdxNKNSearch[i] ].y
                    << " " << cloud->points[ pointIdxNKNSearch[i] ].z
                    << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
      }

      // Neighbors within radius search

      std::vector<int> pointIdxRadiusSearch;
      std::vector<float> pointRadiusSquaredDistance;

      float radius = 256.0f * rand () / (RAND_MAX + 1.0f);

      std::cout << "Neighbors within radius search at (" << searchPoint.x
                << " " << searchPoint.y
                << " " << searchPoint.z
                << ") with radius=" << radius << std::endl;


      if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
      {
        for (std::size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
          std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x
                    << " " << cloud->points[ pointIdxRadiusSearch[i] ].y
                    << " " << cloud->points[ pointIdxRadiusSearch[i] ].z
                    << " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
      }

*/
}

void LICMap::LIC2(float box_length,float frequency,float step_size,MyMesh mesh)
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

auto shader_parameters_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, ShaderParameters*>(_mesh, "shader_parameters");


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
ShaderParameters*  shader_param;
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

      shader_param = shader_parameters_data_wrapper[mesh_vertex_iterator];
      if(shader_param == nullptr)
          std::cout<<"Err shader param is empty"<<std::endl;
      else
        flow_vector = glm::vec3( shader_param->getValue(1),shader_param->getValue(2),shader_param->getValue(3));

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
                     shader_param = shader_parameters_data_wrapper[next_point];
                     glm::vec3 next_flow_vector = glm::vec3( shader_param->getValue(1),shader_param->getValue(2),shader_param->getValue(3));
                     flow_vector = flow_vector* (1-1/(min_dist+1)) + next_flow_vector*(1-min_dist/(min_dist+1));
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
 shader_param->setValue(5,val);
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
          shader_param = shader_parameters_data_wrapper[mesh_vertex_iterator];
        float val = shader_param->getValue(5);
        float temp = shader_param->getValue(2);
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

      shader_param = shader_parameters_data_wrapper[mesh_vertex_iterator];
      if(shader_param == nullptr)
          std::cout<<"Err shader param is empty"<<std::endl;
      else
        flow_vector = glm::vec3( shader_param->getValue(1),shader_param->getValue(2),shader_param->getValue(3));

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
                     shader_param = shader_parameters_data_wrapper[next_point];
                     glm::vec3 next_flow_vector = glm::vec3( shader_param->getValue(1),shader_param->getValue(2),shader_param->getValue(3));
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
 shader_param = shader_parameters_data_wrapper[mesh_vertex_iterator];
 shader_param->setValue(5,val);

//shader_parameters_data_wrapper[mesh_vertex_iterator] = shader_param;
new_point_cloud->points[k].x = actual_point[0] ;
new_point_cloud->points[k].y = actual_point[1] ;
new_point_cloud->points[k].z = actual_point[2] ;
new_point_cloud->points[k].intensity = val;

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

        new_point_cloud->points[k].intensity = val;
      }



   Outputcloud = new_point_cloud;
   kdtree_output.setInputCloud (Outputcloud);

//ReIterationLIC(box_length,frequency,step_size,_mesh);
}


/*void LICMap::HistogramEqualization()
{
    int L = 300;
    auto shader_parameters_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, ShaderParameters*>(_mesh, "shader_parameters");

    MyMesh::VertexIter mesh_vertex_iterator;
    MyMesh::VertexIter vertex_iterator_end(_mesh.vertices_end());

}


void LICMap::ReIterationLIC(float box_length,float frequency,float step_size,MyMesh mesh)
{

}*/

/*
void LICMap::LIC(float box_length,float frequency,float step_size,MyMesh mesh)
{
     FastNoise noise;
      OpenMesh::Subdivider::Uniform::CatmullClarkT<MyMesh> catmull;
    pcl::PointCloud<pcl::PointXYZI>::Ptr new_point_cloud ( new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZLNormal>::iterator cloud_iterator;
  catmull.attach(mesh);
  catmull( 1 );

  new_point_cloud->width = mesh.n_vertices();//numpoints
  new_point_cloud->height = 1;
  new_point_cloud->points.resize (new_point_cloud->width * new_point_cloud->height);

    pcl::PointXYZLNormal actual_point;
    pcl::PointXYZLNormal found_point;

//num_threads(4)

//pcl::CloudIterator::
//pcl::PointCloud<pcl::PointXYZLNormal>::iterator cloud_iterator
//#pragma omp parallel for
for(int j = 0;j<cloud->points.size();j++)
{
//    std::cout<<"hola"<<std::endl;
//  //geom::vectors::find_normal, pt_cl, cloud_it, kdt, radius, max_neighbs);
//
//
//   #pragma openmp parallel for num_threads(4)
//    for(cloud_iterator= cloud->begin();cloud_iterator!=cloud->end();cloud_iterator++)
  //  {
    std::cout<<" j is "<< j << " over " << mesh.n_vertices()<<std::endl;

         pcl::PointXYZLNormal pt = cloud->points[j];

                  actual_point = pt;
         found_point = pt;
         float val = 0;
         float w = 1;
         int K =10 ;
   //      std::cout<<"--------------NEW ITERATION pt.x "<<pt.x<<" pt.y "<<pt.y<<std::endl;
         //  std::cout<<"ahoj "<<x++<< " num threads "<<omp_get_num_threads()<<" over "<< omp_get_max_threads()<<std::endl;

    //     std::cout<< " j "<< j << " over "<<cloud->points.size()<<std::endl;
         for(float i = 0;i<box_length;i++)
         {

             actual_point.x += found_point.normal_x*step_size;
             actual_point.y += found_point.normal_y*step_size;
             actual_point.z += found_point.normal_z*step_size;

             std::vector<int> pointIdxNKNSearch(K);
             std::vector<float> pointNKNSquaredDistance(K);
             if ( kdtree.nearestKSearch (actual_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
             {
                 found_point = cloud->points[pointIdxNKNSearch[0]];
             }
             else
             {
                 continue;
             }

            float n = noise.GetPerlin(actual_point.x * frequency ,
                                    actual_point.y * frequency,
                                    actual_point.z * frequency);

            val+=n;
             w++;
         }

         actual_point = pt;
         found_point = pt;
         for(float i = 0;i<box_length;i++)
         {

             actual_point.x -= found_point.normal_x*step_size;
             actual_point.y -= found_point.normal_y*step_size;
             actual_point.z -= found_point.normal_z*step_size;

             std::vector<int> pointIdxNKNSearch(K);
             std::vector<float> pointNKNSquaredDistance(K);
             if ( kdtree.nearestKSearch (actual_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
             {

                 found_point = cloud->points[pointIdxNKNSearch[0]];
              //  i+= pointNKNSquaredDistance[0];
             }
             else
             {
                // i++;
                 continue;
             }
             float n = noise.GetPerlin(actual_point.x * frequency ,
                                    actual_point.y * frequency,
                                    actual_point.z * frequency);
    //         n = n >= 0 ? 1 : 0;
            val+=n;

             w++;
         }
         val = val/w;
      //   std::cout<<"Total VAL "<< val <<" pt.x" <<pt.x << "actual_point.x "<<actual_point.x
      //           <<" pt.y" <<pt.y << "actual_point.y "<<actual_point.y<< std::endl;


         new_point_cloud->points[j].x = pt.x ;
         new_point_cloud->points[j].y = pt.y ;
         new_point_cloud->points[j].z = pt.z ;
         new_point_cloud->points[j].intensity = val;

    }
    catmull.detach();

    Outputcloud = new_point_cloud;
  kdtree_output.setInputCloud (Outputcloud);
}
*/

float LICMap::GetPoint(float point[3])
{
    pcl::PointXYZI searchPoint;
    searchPoint.x = point[0];
    searchPoint.y = point[1];
    searchPoint.z = point[2];

    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    if ( kdtree_output.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
      pcl::PointXYZI point= Outputcloud->points[ pointIdxNKNSearch[0]];

    return point.intensity;
    }
    return 0;
}


