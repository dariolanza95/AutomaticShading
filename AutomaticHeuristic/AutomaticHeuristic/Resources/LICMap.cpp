#include "LICMap.h"


LICMap::LICMap(MyMesh mesh)
{


      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

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
      for(vertex_handle= mesh.vertices_begin();vertex_handle!= vertex_iterator_end;++vertex_handle)
      {
          ShaderParameters* const shader_param = shader_parameters_data_wrapper[vertex_handle];
          glm::vec3 vec;
          for(int i = 0;i<3;i++)
          {
              vec[i] = shader_param->getValue(i);
          }
          mesh_point =  mesh.point(*vertex_handle);
          cloud->points[j].x = mesh_point[0];
          cloud->points[j].y = mesh_point[1];
          cloud->points[j].z = mesh_point[2];
          cloud->points[j].r = vec[0];
          cloud->points[j].g = vec[1];
          cloud->points[j].b = vec[2];
          j++;
      }

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

pcl::PointXYZRGB LICMap::GetPoint(glm::vec3 point)
{
    pcl::PointXYZRGB searchPoint;
    searchPoint.x = point[0];
    searchPoint.y = point[1];
    searchPoint.z = point[2];
    int K = 1;
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    if ( kdtree.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
/*      for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
        std::cout << "    "  <<   cloud->points[ pointIdxNKNSearch[i] ].x
                  << " " << cloud->points[ pointIdxNKNSearch[i] ].y
                  << " " << cloud->points[ pointIdxNKNSearch[i] ].z
                  << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
*/
      return cloud->points[ pointIdxNKNSearch[0]];
    }
}


