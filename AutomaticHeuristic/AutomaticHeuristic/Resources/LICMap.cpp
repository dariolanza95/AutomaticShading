#include "LICMap.h"


LICMap::LICMap(MyMesh mesh)
{


    // kd-tree used for finding neighbours
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdt;





      pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud1 ( new pcl::PointCloud<pcl::PointXYZLNormal>);
cloud = cloud1;
      // Generate pointcloud data
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
      OpenMesh::Subdivider::Uniform::CatmullClarkT<MyMesh> catmull;
      // Execute 1 subdivision steps
          kdtree.setInputCloud (cloud);
      //iterate for number of desired subdivisions
    /*  catmull.attach(mesh);
      catmull( 1 );
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

               /*     if(pointNKNSquaredDistance[0] < 10e-2)
                    {
                        //The actual point was already present in the cloud so we stop here, since interpolation was done at the previous step
                       break;
                    }
                    else
                    {
                        InterpolateData(pointIdxNKNSearch, pointNKNSquaredDistance);
                    }
              InterpolateData();*//*
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

*/
      kdtree.setInputCloud (cloud);




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
  new_point_cloud->points.resize (cloud->width * cloud->height);

int j = 0;
    pcl::PointXYZLNormal actual_point;
    pcl::PointXYZLNormal found_point;
int x = 0;
//num_threads(4)

//pcl::CloudIterator::
//pcl::PointCloud<pcl::PointXYZLNormal>::iterator cloud_iterator
//#pragma omp parallel for
for(int j = 0;j<cloud->points.size();j++)
    //    for (std::vector<int>::const_iterator cloud_iterator = cloud->points.begin();
    //cloud_iterator < cloud->end();
    //++cloud_iterator )
{
//    std::cout<<"hola"<<std::endl;
//  //geom::vectors::find_normal, pt_cl, cloud_it, kdt, radius, max_neighbs);
//
//
//   #pragma openmp parallel for num_threads(4)
//    for(cloud_iterator= cloud->begin();cloud_iterator!=cloud->end();cloud_iterator++)
  //  {
         pcl::PointXYZLNormal pt = cloud->points[j];

                  actual_point = pt;
         found_point = pt;
         float val = 0;
         float w = 1;
         int K =1 ;
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


