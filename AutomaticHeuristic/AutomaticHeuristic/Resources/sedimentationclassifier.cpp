#include "sedimentationclassifier.h"
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using namespace boost::numeric::ublas;

double SedimentationClassifier::RBFInterp2(glm::vec3 actual_point,std::vector<glm::vec3> selected_points){
int number_of_samples = selected_points.size();

    Eigen::MatrixXd  X(2, number_of_samples);
    Eigen::VectorXd  y(number_of_samples);
    if(selected_points.size()<=0){
        return -1;
    }


    for (int index = 0;index<number_of_samples;index++)
    {
        glm::vec3 temp_vec = glm::vec3(selected_points[index]);
        X.col(index) = Vector2d(temp_vec[0],temp_vec[1]);//,temp_vec[2]);
        y(index)     = temp_vec[2];
    }

    // Define interpolation settings
    const auto     kernel             = mathtoolbox::ThinPlateSplineRbfKernel();
    const auto     kernel_2           = mathtoolbox::InverseQuadraticRbfKernel();
    constexpr bool use_regularization = false;

    // Instantiate an interpolator
    //mathtoolbox::RbfInterpolator rbf_interpolator(kernel);
    mathtoolbox::RbfInterpolator rbf_interpolator(kernel_2);

    // Set data
    rbf_interpolator.SetData(X, y);

    // Calculate internal weights with or without regularization
    rbf_interpolator.CalcWeights(use_regularization);

    // Calculate and print interpolated values on randomly sampled points in CSV format
//    constexpr int number_of_test_samples = 1;
//    std::cout << "x(0),x(1),y" << std::endl;
  //  for (int i = 0; i < number_of_test_samples; ++i)
  //  {
        const Vector2d x(actual_point[0],actual_point[1]);//,actual_point[2]);
        const double   res = rbf_interpolator.CalcValue(x);
        return res;
     //   std::cout << x(0) << "," << x(1) << "," << y << std::endl;
    //}
}



float SedimentationClassifier::RBFInterp(glm::vec3 actual_point,std::vector<glm::vec3> selected_points,std::vector<float> list_of_materials){
int number_of_samples = selected_points.size();
    constexpr double noise_intensity   = 0.1;

    Eigen::MatrixXd  X(3, number_of_samples);
    Eigen::VectorXd  y(number_of_samples);
    if(selected_points.size()<=0){
        return -1;
    }

    for (int index = 0;index<number_of_samples;index++)
    {
        glm::vec3 temp_vec = glm::vec3(selected_points[index]);
        X.col(index) = Vector3d(temp_vec[0],temp_vec[1],temp_vec[2]);
        y(index)     = list_of_materials[index];
    }

    // Define interpolation settings
    const auto     kernel             = mathtoolbox::RbfInterpolator();
    constexpr bool use_regularization = true;

    // Instantiate an interpolator
    mathtoolbox::RbfInterpolator rbf_interpolator(kernel);

    // Set data
    rbf_interpolator.SetData(X, y);

    // Calculate internal weights with or without regularization
    rbf_interpolator.CalcWeights();

    // Calculate and print interpolated values on randomly sampled points in CSV format
//    constexpr int number_of_test_samples = 1;
//    std::cout << "x(0),x(1),y" << std::endl;
  //  for (int i = 0; i < number_of_test_samples; ++i)
  //  {
        const Vector3d x(actual_point[0],actual_point[1],actual_point[2]);
        const double   res = rbf_interpolator.CalcValue(x);
        return res;
     //   std::cout << x(0) << "," << x(1) << "," << y << std::endl;
    //}
}
double tps_base_func(double r)
{
  if ( r == 0.0 )
    return 0.0;
  else
    return r*r * log(r);
}

float SedimentationClassifier::CalculateTPS(std::vector<glm::vec3> control_points)
    {
    float regularization = 0.2f;
      // You We need at least 3 points to define a plane
      if ( control_points.size() < 3 )
        return INFINITY;

      unsigned p = control_points.size();

      // Allocate the matrix and vector
      //for 3D case le't put +4
      //matrix<double> mtx_l(p+3, p+3);
      //matrix<double> mtx_v(p+3, 1);
      matrix<double> mtx_l(p+3, p+3);
      matrix<double> mtx_v(p+3, 1);
      matrix<double> mtx_orig_k(p, p);

      // Fill K (p x p, upper left of L) and calculate
      // mean edge length from control points
      //
      // K is symmetrical so we really have to
      // calculate only about half of the coefficients.
      double a = 0.0;
      for ( unsigned i=0; i<p; ++i )
      {
        for ( unsigned j=i+1; j<p; ++j )
        {
          glm::vec3 pt_i = control_points[i];
          glm::vec3 pt_j = control_points[j];
          pt_i[2] = pt_j[2] = 0;
          double elen = sqrtf(dot(pt_i-pt_j,pt_i-pt_j));
          mtx_l(i,j) = mtx_l(j,i) =
            mtx_orig_k(i,j) = mtx_orig_k(j,i) =
              tps_base_func(elen);
          a += elen * 2; // same for upper & lower tri
        }
      }
      a /= (double)(p*p);

      // Fill the rest of L
      for ( unsigned i=0; i<p; ++i )
      {
        // diagonal: reqularization parameters (lambda * a^2)
        mtx_l(i,i) = mtx_orig_k(i,i) =
          regularization * (a*a);

        // P (p x 3, upper right)
        mtx_l(i, p+0) = 1.0;
        mtx_l(i, p+1) = control_points[i][0];
        mtx_l(i, p+2) = control_points[i][1];
    //    mtx_l(i, p+3) = control_points[i][2];

        // P transposed (3 x p, bottom left)
        mtx_l(p+0, i) = 1.0;
        mtx_l(p+1, i) = control_points[i][0];//.x;
        mtx_l(p+2, i) = control_points[i][1];//.z;
      //  mtx_l(p+3, i) = control_points[i][2];
      }
      // O (4 x 4, lower right)
      for ( unsigned i=p; i<p+3; ++i )
        for ( unsigned j=p; j<p+3; ++j )
          mtx_l(i,j) = 0.0;


      // Fill the right hand vector V
      for ( unsigned i=0; i<p; ++i )
        mtx_v(i,0) = control_points[i][2];//.y;
      //mtx_v(i,0) = 1;
          mtx_v(p+0, 0) = mtx_v(p+1, 0) = mtx_v(p+2, 0) = 0.0;

      // Solve the linear system "inplace"
      if (0 != LU_Solve(mtx_l, mtx_v))
      {
          std::cout<<"singular matrix"<<std::endl;
        return INFINITY;
          //puts( "Singular matrix! Aborting." );
        //exit(1);
      }

    /*  // Interpolate grid heights
      for ( int x=-GRID_W/2; x<GRID_W/2; ++x )
      {
        for ( int z=-GRID_H/2; z<GRID_H/2; ++z )
        {
          double h = mtx_v(p+0, 0) + mtx_v(p+1, 0)*x + mtx_v(p+2, 0)*z;
          Vec pt_i, pt_cur(x,0,z);
          for ( unsigned i=0; i<p; ++i )
          {
            pt_i = control_points[i];
            pt_i.y = 0;
            h += mtx_v(i,0) * tps_base_func( ( pt_i - pt_cur ).len());
          }
          grid[x+GRID_W/2][z+GRID_H/2] = h;
        }
      }
*/
      // Calc bending energy
      matrix<double> w( p, 1 );
      for ( int i=0; i<p; ++i )
        w(i,0) = mtx_v(i,0);
      matrix<double> be = prod( prod<matrix<double> >( trans(w), mtx_orig_k ), w );
     float bending_energy = be(0,0);
return bending_energy;
}

SedimentationClassifier::SedimentationClassifier(MyMesh mesh, SimulationDataMap simulation_data_map) : AClassifier(mesh)
,simulation_data_map(simulation_data_map)
{_shader = std::shared_ptr<AShader>(new SedimentationShader(_id)); }

SedimentationClassifier::~SedimentationClassifier(){
    std::cout<<"Sed class dtor"<<std::endl;
}
void SedimentationClassifier::AverageData(){
    std::vector<std::shared_ptr<AShader>>  temporary_list_of_shaders(list_of_sedimentation_points.size());
    std::vector<glm::vec3> temporary_list_of_sedimentation_points(list_of_sedimentation_points.size());
    std::vector<float> new_list_of_materials;
    std::vector<glm::vec3> new_list_of_points;
   //  temporary_list_of_sedimentation_points = list_of_sedimentation_points;
   // temporary_list_of_shaders = list_of_shaders  ;
    CreatePointCloud();
    for(uint i = 0;i<list_of_sedimentation_points.size();i++){
        pcl::PointXYZLNormal new_point;
        glm::vec3 actual_point = list_of_sedimentation_points[i];
        std::cout<<i<<" over" << list_of_sedimentation_points.size()<<std::endl;
        new_point.x = actual_point[0];
        new_point.y = actual_point[1];
        new_point.z = actual_point[2];
    int id = 0;
  pcl::PointXYZLNormal temp_point;
        int K = 30;//or 27
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<SedimentationShader*> sedimentation_materials_list(K);
        std::vector<float> dists_list(K);
        std::vector<float> ids_list(K);
        float total_dist=0;
        float total= 0;
        std::vector<float> pointNKNSquaredDistance(K);
        if ( kdtree_input.nearestKSearch (new_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
            //SedimentationShader* tmp_sed = temporary_list_of_shaders[pointIdxNKNSearch[0]];
            std::vector<std::vector<float>> list_of_lists(pointIdxNKNSearch.size());
            for(uint j = 0;j<pointIdxNKNSearch.size();j++){
                list_of_lists[j] = std::static_pointer_cast<SedimentationShader>( list_of_shaders[pointIdxNKNSearch[j]])->getListOfIntermediateSedimentationMaterials();
            }
            std::shared_ptr<SedimentationShader> tmp_sed = std::static_pointer_cast<SedimentationShader> (list_of_shaders[pointIdxNKNSearch[0]]);
            float tot_size = 0;
            for(uint j  = 0;j<list_of_lists.size();j++){
                tot_size += list_of_lists[j].size();
            }
            tot_size /= list_of_lists.size();
           //new_list_of_materials = tmp_sed->getListOfIntermediateSedimentationMaterials();
            new_list_of_points = tmp_sed->getListOfIntermediateSedimentationPoints();
           int new_size = roundf(tot_size);
            new_list_of_materials.resize(new_size);
            for(uint j = 0;j<new_size;j++){
                float total_val = 0;
                for(uint z = 0;z<list_of_lists.size();z++){
                    if(list_of_lists[z].size()>j)
                        total_val+=list_of_lists[z][j];
                }
                float val_res  = roundf(total_val/(float)list_of_lists.size());
new_list_of_materials[j] = val_res;
            }

            glm::vec3 initial_point = new_list_of_points[0];
            glm::vec3 actual_point = new_list_of_points.back();
            new_list_of_points.resize(new_size);
            glm::vec3 distance_vector =   actual_point - initial_point;
            int num_divisions = new_size -1 ;//- 2 + 1;
            float mod = sqrtf(dot(distance_vector,distance_vector));
            float step_length = mod / (float) num_divisions;
            glm::vec3 normalized_distance_vector = distance_vector/mod;
            glm::vec3 new_sedimentation_point;
            for(uint i = 0 ;i<new_size;i++){
                new_sedimentation_point=  initial_point + normalized_distance_vector * step_length * (float )i ;
                new_list_of_points.push_back(new_sedimentation_point);
         }


          //  temporary_list_of_shaders[i] = std::shared_ptr<SedimentationShader> (new SedimentationShader(_id,1.0f,new_list_of_points,new_list_of_materials)) ;
            new_list_of_materials.clear();
            new_list_of_points.clear();
    }

}
    list_of_shaders = temporary_list_of_shaders;
}

void SedimentationClassifier::AssignSedimentationParameters2(map<MyMesh::VertexHandle,sedimentationData> selected_vertices)
{
    std::vector<std::shared_ptr<AShader>>  temporary_list_of_shaders;
    std::vector<glm::vec3> temporary_list_of_sedimentation_points;
    //uint i = 0;i<selected_vertices.size();i++

int i = 0;
    for(auto const entry : list_of_shaders){
       i++;
        std::vector<float> temp_mat;
       std::vector<glm::vec3> temp_pts;
        std::shared_ptr<SedimentationShader> sd =std::static_pointer_cast<SedimentationShader>(entry);
       temp_mat = sd->getListOfIntermediateSedimentationMaterials();
       temp_pts = sd->getListOfIntermediateSedimentationPoints();
       int l = temp_mat.size();
       for(uint i = 0;i<temp_mat.size();i++){

        //if(i==l-1){
            temporary_list_of_shaders.push_back(std::shared_ptr<SedimentationShader> (new SedimentationShader(_id,1.0f,temp_mat[i])));
            temporary_list_of_sedimentation_points.push_back(temp_pts[i]);
      //  }
       }
    }
list_of_sedimentation_points= temporary_list_of_sedimentation_points;
list_of_shaders = temporary_list_of_shaders;
}

std::vector<int> SedimentationClassifier::SelectNeighbours(glm::vec3 actual_point){
    std::vector<int> list_of_indices;
    pcl::PointXYZLNormal new_point;
    pcl::PointXYZLNormal Front_vector;
    pcl::PointXYZLNormal Right_vector;
    pcl::PointXYZLNormal Up_vector;
    pcl::PointXYZLNormal probe;
    new_point.x = actual_point[0];
    new_point.y = actual_point[1];
    new_point.z = actual_point[2];

    Front_vector.x = 1;
    Front_vector.y = 0;
    Front_vector.z = 0;

    Right_vector.x = 0;
    Right_vector.y = 1;
    Right_vector.z = 0;

    Up_vector.x = 0;
    Up_vector.y = 0;
    Up_vector.z = 1;
    std::set<int>::iterator it;
     std::pair<std::set<int>::iterator,bool> ret;
    std::set<int> set_of_indices;
    int K = 2;//or 27
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    std::vector<glm::vec3> interp_points;
    std::vector<float> list_of_materials;
    if ( kdtree_input.nearestKSearch (new_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
        //list_of_indices.push_back(pointIdxNKNSearch[0]);
        int val = pointIdxNKNSearch[0];
        set_of_indices.insert(val);
        K = 3;
        float scale=2.0f;
        probe = new_point;
        probe.z+= scale;
        if ( kdtree_input.nearestKSearch (probe, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
            for(uint j=0;j<pointIdxNKNSearch.size();j++){
                if(set_of_indices.count(pointIdxNKNSearch[j])==0){
                    int val = pointIdxNKNSearch[j];
                    ret = set_of_indices.insert(val);
                 //   std::cout<< *(ret.first)<<std::endl;
                }
            }
        }

        pointIdxNKNSearch.clear();
        pointNKNSquaredDistance.clear();
        probe = new_point;
        probe.z-= scale;
        if ( kdtree_input.nearestKSearch (probe, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
            for(uint j=0;j<pointIdxNKNSearch.size();j++){
                if(set_of_indices.count(pointIdxNKNSearch[j])==0){
                    int val = pointIdxNKNSearch[j];
                    ret = set_of_indices.insert(val);
                //    std::cout<< *(ret.first)<<std::endl;
                }
        }}

        probe = new_point;
        probe.x+= scale;
        if ( kdtree_input.nearestKSearch (probe, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
            for(uint j=0;j<pointIdxNKNSearch.size();j++){
                if(set_of_indices.count(pointIdxNKNSearch[j])==0){
                    int val = pointIdxNKNSearch[j];
                    set_of_indices.insert(val);                }
        }}
        probe = new_point;
        probe.x-= scale;
        if ( kdtree_input.nearestKSearch (probe, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
            for(uint j=0;j<pointIdxNKNSearch.size();j++){
                if(set_of_indices.count(pointIdxNKNSearch[j])==0){
                    int val = pointIdxNKNSearch[j];
                    set_of_indices.insert(val);             }
        }}
        probe = new_point;
        probe.y+= scale;
        if ( kdtree_input.nearestKSearch (probe, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
            for(uint j=0;j<pointIdxNKNSearch.size();j++){
                if(set_of_indices.count(pointIdxNKNSearch[j])==0){
                    int val = pointIdxNKNSearch[j];
                    set_of_indices.insert(val);                }
        }}

        probe = new_point;
        probe.y-= scale;
        if ( kdtree_input.nearestKSearch (probe, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
            for(uint j=0;j<pointIdxNKNSearch.size();j++){
                if(set_of_indices.count(pointIdxNKNSearch[j])==0){
                    int val = pointIdxNKNSearch[j];
                    set_of_indices.insert(val);                }
        }
    }
        int f = 0;
        for(int index: set_of_indices){

            if(f==0){
                f++;
                continue;
            }

            list_of_indices.push_back(index);
        }}
return list_of_indices;
        }

bool IsFrontierPoint(glm::vec3 pp){
    if(  roundf(pp[0])== 0 || ceilf(pp[0]) ==299 || roundf(pp[1])==0 || ceilf(pp[1]==299)  )
        return true;
    else
        return false;
}

float CalculateVariance(std::vector<glm::vec3> input_points){
    float average = 0;
    glm::vec3 centroid(0,0,0);
    int counter = input_points.size();
    for(glm::vec3 point : input_points){
        float height = point[2];
        centroid += point;
        average+=height;
    }
    average = average/(float)counter;
    centroid[0] /= (float )counter;
    centroid[1] /= (float )counter;
    centroid[2] /= (float )counter;
    float value = 0;
    glm::vec3 value_p(0,0,0);
    for(glm::vec3 point : input_points){
        float height = point[2];
        value += (height -average)*(height -average);
//        value += dot(point-centroid,point-centroid);
    }
    value = sqrtf(value/(float)counter);

    return value;
}


void SedimentationClassifier::TPSSecondApproach(map<MyMesh::VertexHandle,sedimentationData> selected_vertices){
    std::vector<std::shared_ptr<AShader>>  temporary_list_of_shaders;
    std::vector<glm::vec3> temporary_list_of_sedimentation_points;
    int f = 0;
    int percentil_part = selected_vertices.size()/100;
    for(auto const entry : selected_vertices){
        if(f%percentil_part == 0){
            std::cout<<f/percentil_part <<" %"<<std::endl;
        }
        f++;

        //if(f/percentil_part<55 ){
        //    continue;
        //}
        pcl::PointXYZLNormal new_point;
        MyMesh::Point actual_point = _mesh.point(entry.first);
        int num_materials = 4;
        new_point.x = actual_point[0];
        new_point.y = actual_point[1];
        new_point.z = actual_point[2];
        int K = 1;//or 27
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        int final_id = 0;
        float val_1,val_2;
        int id_candidate = 0;
        int expected_res = 0;
        std::vector<glm::vec3> min_points,max_points;
        if ( kdtree_input.nearestKSearch (new_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
            glm::vec3 pp(actual_point[0],actual_point[1],actual_point[2]);
            float offset = std::sin(pp[1]);
            offset = 0;
            int z_height =ceilf(pp[2]+offset);
           expected_res  = z_height%num_materials;
            //expected_res+=2;
            if(expected_res==0)
                expected_res = num_materials;
            if(pp[0]==171.5 && pp[1] == 145.5){
                std::cout<<"roi"<<std::endl;
            }
            float min_dist = INFINITY;
            float min_dist_2 = INFINITY;


        for(int i=1;i<6;i++){
           std::vector<int> list = SelectNeighbours(pp);
           float min_height = INFINITY;
           float max_height = -INFINITY;
           float min_centroid = 0;
           float max_centroid = 0;
     //      std::map<int,std::pair<std::vector<glm::vec3>,std::vector<glm::vec3>>> map_of_stack_id_to_material_id;
           std::shared_ptr<SedimentationShader>  shad;
           glm::vec3 min_point;
           glm::vec3 max_point;
           min_centroid = INFINITY;
           max_centroid = -INFINITY;
           for(int j:list){

            shad = std::static_pointer_cast<SedimentationShader>(list_of_shaders[j]);
            if(shad==nullptr)
                continue;
            int stack_id = -1;

                if(shad->GetLineId2(i,pp,stack_id,min_point,max_point)){
                    min_points.push_back(min_point);
                    max_points.push_back(max_point);
                    if(min_point[2]<min_centroid){
                        min_centroid = min_point[2];
                    }
                    if(max_point[2]>max_centroid){
                        max_centroid = max_point[2];
                    }
                   /* if(map_of_stack_id_to_material_id.count(stack_id)>0){
                        std::pair<std::vector<glm::vec3>,std::vector<glm::vec3>> &element = map_of_stack_id_to_material_id[stack_id];
                        element.first.push_back(min_point);
                        element.second.push_back(max_point);
                    }else{
                        std::vector<glm::vec3> min_points;
                        std::vector<glm::vec3> max_points;
                        min_points.push_back(min_point);
                        max_points.push_back(max_point);
                        map_of_stack_id_to_material_id.insert(std::make_pair(stack_id,std::make_pair(min_points,max_points)));
                    }*/
                }
           }
      //     std::vector<glm::vec3> min_points;
      //     std::vector<glm::vec3> max_points;
           //if(map_of_stack_id_to_material_id.size()>1){
           //    std::cout<<"interesting"<<std::endl;
           //}
         /* for(auto const map_entry:map_of_stack_id_to_material_id){
            std::pair<std::vector<glm::vec3>,std::vector<glm::vec3>> couple = map_entry.second;
            min_points = couple.first;
            max_points = couple.second;
            min_centroid = 0;
            max_centroid = 0;
            min_height = INFINITY;
            max_height = -INFINITY;
            int counter = 0;
            std::vector<glm::vec3>::iterator it_max = max_points.begin();
            for(std::vector<glm::vec3>::iterator it = min_points.begin(); it != min_points.end(); ++it,++it_max){
                glm::vec3 min_point = *it;
                glm::vec3 max_point = *it_max;
                counter++;
                min_centroid += min_point[2];//s[k][2];
                max_centroid += max_point[2];//s[k][2];
                if(min_point[2]<min_height)
                    min_height = min_point[2];
                if( max_point[2]> max_height)
                    max_height = max_point[2];
          //  if(std::abs(min_point[2]-pp[2])>4 || std::abs(max_point[2]-pp[2])>4){
          //      min_points.erase(it);
          //      max_points.erase(it_max);
            }
            }*/
        //min_centroid = min_centroid/(float) counter;
        //max_centroid = max_centroid/(float) counter;

        if(min_points.size()==0){

            min_height = INFINITY;
            max_height = -INFINITY;
           min_points.clear();
           max_points.clear();
           continue;
        }
          //  for(int k = 0;k<min_points.size();k++){
          //      min_centroid += min_points[k][2];
          //      max_centroid += max_points[k][2];
          //
          // }
          //  min_centroid = min_centroid/(float) min_points.size();
          //  max_centroid = max_centroid/(float) min_points.size();
            float eps = 0.01;
            float dist;


            if(min_points.size()>=3){
                val_1 = RBFInterp2(pp,min_points);
                val_2 = RBFInterp2(pp,max_points);
                 float var_1=CalculateVariance(min_points);
                 //float var_2=CalculateVariance(max_points);
                float dist = std::abs(val_1-pp[2]) + std::abs(val_2-pp[2]);
                float err1 = std::abs(val_1 - min_centroid);
                float err2 = std::abs(val_2 - max_centroid);
                dist = dist + var_1  ;
                float dist_2 = dist+ err1*err1+err2*err2;

              //  dist = x-val_1 + x-val_2;
      // float         dist = std::abs(min_centroid-pp[2]) + std::abs(max_cent"roi"d-pp[2]);
                if(dist_2<min_dist){
                    min_dist = dist_2;
                    final_id = i;
                }

                 /*   if(pp[2]>=val_1-eps)  {

                        if(pp[2]<=val_2+eps && (min_height<=val_1 && val_2<=max_height) ){
                         //   float dist;// = max_height-min_height;
                          //  dist = min_centroid-pp[2];
                            float dist = std::abs(pp[2]-val_2+eps);
                            dist += std::abs(pp[2]-val_1-eps);
                           // dist =  val_2-val_1;//CalculateVariance(min_points);
                            if(dist>min_succesful_dist){
                                min_succesful_dist = dist;
                                final_id = i;
                                stack_id = map_entry.first;
                            }
                           // if(z_height%6!=1){
                           //     std::cout<<"false positive"<<std::endl;
                           // }
                        }else
                        {
                            float range = val_2-val_1;
                            float dist = std::abs(pp[2]-val_2+eps);
                            dist += std::abs(pp[2]-val_1-eps);

            //    dist = std::abs(pp[2]-max_centroid) +std::abs(pp[2]-min_centroid) ;

                            if(dist<min_dist //&& min_height<=pp[2] && max_height>=pp[2]
){
                                min_dist = dist;
                                id_candidate = i;
                                stack_id_candidate = map_entry.first;
                            }
                        }
                    }
                    else{
                        float range = val_2-val_1;
                        float dist = std::abs(pp[2]-val_2+eps);
                        dist += std::abs(pp[2]-val_1-eps);
                     //   dist = pp[2]-min_height + max_height - pp[2];//+1/range;
                   //     dist = std::abs(pp[2]-max_centroid) +std::abs(pp[2]-min_centroid) ;
                        if(dist<min_dist) //&& min_height<=pp[2] && max_height>=pp[2])
                        {
                            min_dist = dist;
                            id_candidate = i;
                            stack_id_candidate = map_entry.first;
                        }
                    }*/

            }
            min_points.clear();
            max_points.clear();
}

      //  min_height = INFINITY;
      //  max_height = -INFINITY;
       min_points.clear();
       max_points.clear();

     //   }

                   //   id_candidate = 0;


            if(final_id==0)
            {
                final_id = id_candidate;
              //  stack_id = stack_id_candidate;
            }

            float err_treshold = pp[2]-floorf(pp[2]);

       //  if(final_id != expected_res && final_id!=0 && pp[2]>1 && err_treshold>0.02  ){
       //     std::cout<<"mistake: id is "<<final_id << " expected "<<expected_res << " err "<<err_treshold<< " pp.x "<<pp[0]<<" pp.y "<<pp[1]<<std::endl;
       // }
            if(final_id!=0){
              temporary_list_of_sedimentation_points.push_back(pp);
              std::shared_ptr<AShader> shad  = std::shared_ptr<SedimentationShader>(new SedimentationShader(_id,1.0f,final_id));
              temporary_list_of_shaders.push_back(shad);
            }


    }
}
    list_of_sedimentation_points = temporary_list_of_sedimentation_points;
    list_of_shaders = temporary_list_of_shaders;
}

void SedimentationClassifier::TPSFirstApprach(map<MyMesh::VertexHandle,sedimentationData> selected_vertices)
{
    std::vector<std::shared_ptr<AShader>>  temporary_list_of_shaders;
    std::vector<glm::vec3> temporary_list_of_sedimentation_points;
    //uint i = 0;i<selected_vertices.size();i++
    int f = 0;
    int percentil_part = selected_vertices.size()/100;
    for(auto const entry : selected_vertices){
        if(f%percentil_part == 0){
            std::cout<<f/percentil_part <<" %"<<std::endl;
        }
        f++;
        pcl::PointXYZLNormal new_point;
        MyMesh::Point actual_point = _mesh.point(entry.first);

        new_point.x = actual_point[0];
        new_point.y = actual_point[1];
        new_point.z = actual_point[2];
    int id = 0;
        pcl::PointXYZLNormal temp_point;
        int K = 30;//or 27
        std::vector<int> pointIdxNKNSearch(K);
      //  std::vector<SedimentationShader*> sedimentation_materials_list(K);
        std::vector<float> dists_list(K);
        std::vector<float> ids_list(K);
        float total_dist=0;
        float total= 0;
        std::vector<glm::vec3> interp_points;
        std::vector<float> list_of_materials;
        std::vector<float> pointNKNSquaredDistance(K);
        if ( kdtree_input.nearestKSearch (new_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
            //temporary_list_of_sedimentation_points.push_back(glm::vec3(actual_point[0],actual_point[1],actual_point[2]));
            float min_energy = INFINITY;
            int final_id = 0;
            glm::vec3 pp = glm::vec3 (actual_point[0],actual_point[1],actual_point[2]);
            for(int i = 1;i<4;i++){
        interp_points.clear();
            for(int j=0;j<pointIdxNKNSearch.size();j++){
                std::shared_ptr<SedimentationShader> shad = std::static_pointer_cast<SedimentationShader>(list_of_shaders[pointIdxNKNSearch[j]]);
               glm::vec3 closest_point;

               bool result = shad->getClosestPointWithSameId(pp,i,closest_point);
               if(result){
                   interp_points.push_back(closest_point);

               }else{
                   continue;
               }

            }
            interp_points.push_back(pp);
            //float interp_height = RBFInterp2(pp,interp_points);
            //if(interp_height ==-1)
            //    continue;
            float bending_energy = CalculateTPS(interp_points);
            //bending_energy = abs(interp_height-actual_point[2]);
            if(bending_energy<=min_energy){
                final_id = i;
                min_energy = bending_energy;
            }
        }
            temporary_list_of_sedimentation_points.push_back(pp);
            std::shared_ptr<AShader> shad  = std::shared_ptr<SedimentationShader>(new SedimentationShader(_id,1.0f,final_id));
            temporary_list_of_shaders.push_back(shad);

        }

    }
     list_of_sedimentation_points = temporary_list_of_sedimentation_points;
     list_of_shaders = temporary_list_of_shaders;
}



void SedimentationClassifier::AssignSedimentationParameters3(map<MyMesh::VertexHandle,sedimentationData> selected_vertices) {

    std::vector<std::shared_ptr<AShader>>  temporary_list_of_shaders;
    std::vector<glm::vec3> temporary_list_of_sedimentation_points;
    //uint i = 0;i<selected_vertices.size();i++
    for(auto const entry : selected_vertices){
        pcl::PointXYZLNormal new_point;
        MyMesh::Point actual_point = _mesh.point(entry.first);

        new_point.x = actual_point[0];
        new_point.y = actual_point[1];
        new_point.z = actual_point[2];
    int id = 0;
        pcl::PointXYZLNormal temp_point;
        int K = 5;//or 27
        std::vector<int> pointIdxNKNSearch(K);
      //  std::vector<SedimentationShader*> sedimentation_materials_list(K);
        std::vector<float> dists_list(K);
        std::vector<float> ids_list(K);
        float total_dist=0;
        float total= 0;
        std::vector<glm::vec3> interp_points;
        std::vector<float> list_of_materials;
        std::vector<float> pointNKNSquaredDistance(K);
        if ( kdtree_input.nearestKSearch (new_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
            //temporary_list_of_sedimentation_points.push_back(glm::vec3(actual_point[0],actual_point[1],actual_point[2]));
            for(int j=0;j<pointIdxNKNSearch.size();j++){
                std::shared_ptr<SedimentationShader> shad = std::static_pointer_cast<SedimentationShader>(list_of_shaders[pointIdxNKNSearch[0]]);
               list_of_materials.push_back(shad->GetMaterialId());
               interp_points.push_back(list_of_sedimentation_points[pointIdxNKNSearch[j]]);
            }
            glm::vec3 pp = glm::vec3 (actual_point[0],actual_point[1],actual_point[2]);
            temporary_list_of_sedimentation_points.push_back(pp);
            float val = RBFInterp(pp,interp_points, list_of_materials);
            std::shared_ptr<AShader> shad  = std::shared_ptr<SedimentationShader>(new SedimentationShader(_id,1.0f,roundf(val)));
            temporary_list_of_shaders.push_back(shad);

        }

    }
     list_of_sedimentation_points = temporary_list_of_sedimentation_points;
     list_of_shaders = temporary_list_of_shaders;
}


void SedimentationClassifier::AssignSedimentationParameters(map<MyMesh::VertexHandle,sedimentationData> selected_vertices) {
    std::vector<std::shared_ptr<AShader>>  temporary_list_of_shaders;
    std::vector<glm::vec3> temporary_list_of_sedimentation_points;
    //uint i = 0;i<selected_vertices.size();i++
    int percentual_part = selected_vertices.size()/100;
    int step = 0;




    for(auto const entry : selected_vertices){
        if((step%percentual_part)==0)
        std::cout<<step/percentual_part<<" % completed"<<std::endl;
        pcl::PointXYZLNormal new_point;
        MyMesh::Point actual_point = _mesh.point(entry.first);
    step++;
        new_point.x = actual_point[0];
        new_point.y = actual_point[1];
        new_point.z = actual_point[2];


    int id = 0;
        pcl::PointXYZLNormal temp_point;
        int K = 25;//or 27
        std::vector<int> pointIdxNKNSearch(K);
      //  std::vector<SedimentationShader*> sedimentation_materials_list(K);
        std::vector<float> dists_list(K);
        std::vector<float> ids_list(K);
        float total_dist=0;
        float total= 0;
        std::vector<float> pointNKNSquaredDistance(K);
        if ( kdtree_input.nearestKSearch (new_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
            temporary_list_of_sedimentation_points.push_back(glm::vec3(actual_point[0],actual_point[1],actual_point[2]));
            glm::vec3 pp(actual_point[0],actual_point[1],actual_point[2]);
            float system_energy = 0;
            int z_height =ceilf(pp[2]);
            int expected_id = z_height%4;
            if(pp[0]==150 && pp[1]==5)
                std::cout<<"point of interest"<<std::endl;
            if(expected_id==0)
                expected_id = 4;

            int first_id = -1;
            int second_id = -1;
      //      std::shared_ptr<SedimentationShader> shad2 = std::static_pointer_cast<SedimentationShader>(list_of_shaders[pointIdxNKNSearch[0]]);
          //  uint index = shad2->getClosestPointIndex(pp);
    //        uint min_index = 0;
            int final_id =-1;
            float min_energy = INFINITY;
            float dist ;
         /*   for(uint j = 0;j<pointNKNSquaredDistance.size();j++){
                    total_dist+= (pointNKNSquaredDistance[j]);
            }
        float normalization = 0;
            for(uint j = 0;j<pointNKNSquaredDistance.size();j++){
                    pointNKNSquaredDistance[j] = 1- (pointNKNSquaredDistance[j]/total_dist);
                    normalization+= (pointNKNSquaredDistance[j]);
            }
total_dist = normalization;
            std::vector<float> list = shad2->getListOfIntermediateSedimentationMaterials();*/
            //for(int i = index - 1 ; i <index+1;i++){
            //    if(i< 0 || i >= list.size()){
            //        continue;
            //    }
                float energy = 0;
                float min_dist = INFINITY;
                float min_second_dist = INFINITY;
                system_energy = 0;
                int temp_id = 0;
                float min_local_energy=INFINITY;

                for(uint j = 0;j< pointIdxNKNSearch.size();j++){
                    std::shared_ptr<SedimentationShader> shad = std::static_pointer_cast<SedimentationShader>(list_of_shaders[pointIdxNKNSearch[j]]);

                    //float weight = 1/(K-1);
                  float weight = pointNKNSquaredDistance[j]/total_dist;

//                    if(j!=0){
//                    weight = pointNKNSquaredDistance[j]/total_dist;
//                    //    weight = ((total_dist- pointNKNSquaredDistance[j])/total_dist);
//                    }
                    //float weight = 1;//-1/pointNKNSquaredDistance[j];
                //  system_energy += shad->utilityFunct(pp,list[i]);//*weight;

                    temp_id= shad->getClosestPointMatId(pp,dist);
                    if(dist<min_dist){
                        min_dist = dist;
                        first_id = temp_id;
                        /*if(temp_id!= first_id && first_id != -1){
                            min_second_dist = min_dist;
                            second_id = first_id;

                        }*/

                    }
                    /*else{
                        if(dist<min_second_dist && temp_id!= first_id){
                            min_second_dist = dist;
                            second_id = temp_id;
                        }
                    }*/
                }
                float weight;
                energy = 0;
        /*for(int i = 1;i<7;i++){
            system_energy = 0;
            for(uint j = 0;j< pointIdxNKNSearch.size();j++){
                std::shared_ptr<SedimentationShader> shad = std::static_pointer_cast<SedimentationShader>(list_of_shaders[pointIdxNKNSearch[j]]);
                energy = shad->utilityFunct(pp,i,weight);
                if(weight!=0)
                    system_energy += energy*1/(weight*weight);//*weight;
               else
                    system_energy += energy;
             }

            if(system_energy<min_energy){
                min_energy = system_energy;
                final_id = i;
            }
        }*/

        id = first_id;
    //    if(id==-1)
    //        std::cout<<"err"<<std::endl;
        //id = final_id;
        /*min_energy = INFINITY;
        if(first_id!= final_id){
             for(int i = 1;i<6;i++){
                 system_energy = 0;
                 for(uint j = 0;j< pointIdxNKNSearch.size();j++){
                     std::shared_ptr<SedimentationShader> shad = std::static_pointer_cast<SedimentationShader>(list_of_shaders[pointIdxNKNSearch[j]]);
                     temp_id= shad->getClosestPointMatId(pp,dist);
                     energy = shad->utilityFunct(pp,i,weight);
                     if(weight!=0)
                     system_energy += energy*1/(weight*weight);//*weight;
                    else
                         system_energy += energy;
                  }


                 if(system_energy<min_energy){
                     min_energy = system_energy;
                     final_id = i;
                 }
             }
        }

        /*
                if(first_id!=-1 && second_id!=-1){
                for(uint j = 0;j< pointIdxNKNSearch.size();j++){
                    std::shared_ptr<SedimentationShader> shad = std::static_pointer_cast<SedimentationShader>(list_of_shaders[pointIdxNKNSearch[j]]);
                    system_energy += shad->utilityFunct(pp,first_id);//*weight;
                 }
                }
                if(system_energy<min_energy){
                    min_energy = system_energy;
                    final_id = first_id;
                }
                system_energy = 0;
                    for(uint j = 0;j< pointIdxNKNSearch.size();j++){
                        std::shared_ptr<SedimentationShader> shad = std::static_pointer_cast<SedimentationShader>(list_of_shaders[pointIdxNKNSearch[j]]);
                        system_energy += shad->utilityFunct(pp,second_id);//*weight;
                     }
                if(system_energy<min_energy){
                        min_energy = system_energy;
                        final_id = second_id;
                }else{
                    final_id = first_id;
                }
*/


            /*if(pointIdxNKNSearch.size()>=2){
                id = sedimentation_materials_list[0]->GetMaterialId( *sedimentation_materials_list[1],glm::vec3(actual_point[0],actual_point[1],actual_point[2]));

            }*/
//if(dist!=0){
//    //total_dist = 1/total_dist;
//    float min_dist = INFINITY;
//    for(uint j = 1;j<pointIdxNKNSearch.size();j++){
//        if(dists_list[j]< min_dist){
//            min_dist = dists_list[j];
//            total = ids_list[j];
//        }
//    }
//    id = (total);
//}
//id = list[min_index];

            //id = total/(float )pointIdxNKNSearch.size();
 //       if(id!=expected_id && expected_id>0) {
 //           std::cout<<"mistake id"<<id<<" expected "<< expected_id<<std::endl;
 //       }

            std::shared_ptr<AShader> shad  = std::shared_ptr<SedimentationShader>(new SedimentationShader(_id,1.0f,id,second_id));
            /*     std::shared_ptr<AShader> shad = new SedimentationShader(*shdr);*/
            temporary_list_of_shaders.push_back(shad);

            }

    }
    list_of_sedimentation_points = temporary_list_of_sedimentation_points;
    list_of_shaders = temporary_list_of_shaders;
}
map<MyMesh::VertexHandle,sedimentationData> SedimentationClassifier::SelectPointsForAdaptiveSubdivision(map<MyMesh::VertexHandle,sedimentationData> selected_vertices){
    map<MyMesh::VertexHandle,sedimentationData> vertices_to_be_subdivided;
    for(const auto entry: selected_vertices){
        glm::vec3 up_normal(0,0,1);
        MyMesh::Normal openMesh_normal = _mesh.normal(entry.first);
        glm::vec3 normal= glm::vec3(openMesh_normal[0],openMesh_normal [1],openMesh_normal[2]);
        float res =   (glm::dot(normal,up_normal));
    if(res> 0.85 ){//0.75
        continue;
    }else{
            vertices_to_be_subdivided.insert(entry);
        }
    }
    return vertices_to_be_subdivided;
}
map<MyMesh::VertexHandle,sedimentationData> SedimentationClassifier::SelectSedimentationPoints(){
    map<MyMesh::VertexHandle,sedimentationData> selected_vertices;
    MyMesh::VertexIter vertex_iterator = _mesh.vertices_begin();
    MyMesh::VertexIter vertex_iterator_end = _mesh.vertices_end();
   //simulation_data_wrapper = OpenMesh::getOrMakeProperty<MyMesh::VertexHandle,std::shared_ptr<SimulationData>>(_mesh, "simulation_data");

//    simulation_data_wrapper = OpenMesh::getOrMakeProperty<MyMesh::VertexHandle,std::shared_ptr<SimulationData>>(_mesh, "mazulanzas");

    for(;vertex_iterator != vertex_iterator_end;++vertex_iterator){

        //SimulationData* sd ;
        std::shared_ptr<SimulationData> sd; //= simulation_data_wrapper[*vertex_iterator];
        if(simulation_data_map.count(*vertex_iterator)>0){
            sd = simulation_data_map[*vertex_iterator];
        }else{
            continue;
        }
           std::vector<float> sediment_history;
           std::vector<float> material_stack_height;
           std::vector<float> stack_id;
           sd->getData(SimulationDataEnum::sedimentation_history,sediment_history);
           sd->getData(SimulationDataEnum::material_stack_height,material_stack_height);
           glm::vec3 initial_sedimentation_point;
           glm::vec3 pp;
           sd->getData(SimulationDataEnum::initial_sedimentation_point,initial_sedimentation_point);
           sd->getData(SimulationDataEnum::actual_point,pp);

           sd->getData(SimulationDataEnum::stack_id,stack_id);
           if(sediment_history.size()>0 && material_stack_height.size()>0){

               MyMesh::Point point = _mesh.point(*vertex_iterator);
               glm::vec3 actual_point (point[0],point[1],point[2]);
               std::vector<int> tmp_sediment_history;
               std::vector<int> tmp_stack_id;
               std::vector<float> tmp_material_stack_height;
               for(float entry : sediment_history){
                   if(entry == 0)
                       break;
                   else{
                       tmp_sediment_history.push_back(entry);
                        }
               }
               for(float entry : material_stack_height){

                       tmp_material_stack_height.push_back(entry);

               }

               for(int entry : stack_id){

                       tmp_stack_id.push_back(entry);

               }
               sedimentationData sdtmp = sedimentationData(initial_sedimentation_point,tmp_sediment_history,tmp_material_stack_height,tmp_stack_id);

            selected_vertices.insert(std::make_pair(*vertex_iterator,sdtmp));
           }
    }

return selected_vertices;
}
/*
SedimentationClassifier::CreatePointCloud(){

}
*/
void SedimentationClassifier::ComputeMockUpData_2(glm::vec3 actual_point,float size,int num_materials){
    std::vector<glm::vec3> intermediate_points;
    std::vector<float> intermediate_materials;
    std::shared_ptr<AShader> sedimentation_shader;
    std::vector<int> intermediate_stacks_id;

    int mat = 1;
    float x = actual_point[1];
    //size+= (float)x/(float)10;
    float offset = std::sin(x);
        float eps = 0.001;
    double int_part;
        for(float i=0;i<actual_point[2];i+=size){
        intermediate_points.push_back(glm::vec3(actual_point[0],actual_point[1],i+offset));
        int val = mat%num_materials;
        if (val==0)
            val =num_materials;
        intermediate_materials.push_back(val);
        intermediate_stacks_id.push_back(mat);
    mat++;
        //double rez = modf((double)i,&int_part);
        //if(rez <=0.001 || rez>=0.999)
        //    mat++;
    }
    if(intermediate_points.size()>0){
        sedimentation_shader =std::shared_ptr<SedimentationShader> (new SedimentationShader(_id,1.0f,intermediate_points,intermediate_materials,intermediate_stacks_id));
        list_of_sedimentation_points.push_back(actual_point);
        list_of_shaders.push_back(sedimentation_shader);
    }
}

void SedimentationClassifier::ComputeMockUpData(glm::vec3 actual_point,float size,int num_materials){
    std::vector<glm::vec3> intermediate_points;
    std::vector<float> intermediate_materials;
    std::vector<int> intermediate_stacks_id;
    std::shared_ptr<AShader> sedimentation_shader;
    int mat = 1;
    for(float i=0;i<actual_point[2];i+=size){
        intermediate_points.push_back(glm::vec3(actual_point[0],actual_point[1],(i)));
        int val = mat%num_materials;
        if (val==0)
            val = num_materials;
        intermediate_materials.push_back(val);
        intermediate_stacks_id.push_back(mat);
        mat++;
    }
    if(intermediate_points.size()>0){
        sedimentation_shader =std::shared_ptr<SedimentationShader> (new SedimentationShader(_id,1.0f,intermediate_points,intermediate_materials,intermediate_stacks_id));
        list_of_sedimentation_points.push_back(actual_point);
        list_of_shaders.push_back(sedimentation_shader);
    }
}

void SedimentationClassifier::ComputeSedimentationParametersForVertex(glm::vec3 actual_point,
                                                                      sedimentationData& sedimenation_data){
    std::vector<glm::vec3> intermediate_points;
    std::vector<float> intermediate_materials;
    std::vector<int> intermediate_stack_id;
         std::shared_ptr<AShader> sedimentation_shader;
    if(sedimenation_data.sediment_history.size()==1)   {

        intermediate_points.push_back(actual_point);
        intermediate_materials.push_back(sedimenation_data.sediment_history[0]);
        intermediate_stack_id.push_back(sedimenation_data.material_stack_id[0]);
/*      if(actual_point[2]- sedimenation_data.initial_position[2] > 0.01  ){
//            intermediate_materials.push_back(0.0f);
            intermediate_points.push_back(sedimenation_data.initial_position);
            intermediate_materials.push_back(sedimenation_data.sediment_history[0]);
//            intermediate_materials.push_back(0.0f);            /*
            list_of_shaders.push_back(new SedimentationShader(_id,1.0f, 0));
            list_of_sedimentation_points.push_back(actual_point);
            list_of_shaders.push_back(new SedimentationShader(_id,1.0f, sedimenation_data.sediment_history[0]));
            list_of_sedimentation_points.push_back(sedimenation_data.initial_position);
        }*/

    }else   {
        glm::vec3 initial_point = sedimenation_data.initial_position;
        glm::vec3 distance_vector =   actual_point - initial_point;
        int num_divisions =    sedimenation_data.sediment_history.size() -1 ;//- 2 + 1;
        float mod = sqrtf(dot(distance_vector,distance_vector));
        float step_length = mod / (float) num_divisions;
        glm::vec3 normalized_distance_vector = distance_vector/mod;
        glm::vec3 new_sedimentation_point;
    float bin_size = 0.4;
    float step_size = 0.25;
    float total_height = 0.0f;
    int index = 0;
    /*int material = sedimenation_data.sediment_history[index];
    int stack_id = sedimenation_data.material_stack_id[index];
    intermediate_materials.push_back(material);
    intermediate_stack_id.push_back (stack_id);
    intermediate_points.push_back(initial_point);
    for(float i = initial_point[2];i<actual_point[2];i+=step_size){
        new_sedimentation_point =  initial_point;
        new_sedimentation_point[2]+= i;
        if(i>sedimenation_data.material_stack_width[index]){
            if(index<sedimenation_data.sediment_history.size()-1)
                index++;
            material =sedimenation_data.sediment_history[index];
            stack_id =sedimenation_data.material_stack_id[index];
        }
        if(actual_point[2]-initial_point[2]<0)
        {
            intermediate_materials.push_back(0.0f);
            intermediate_stack_id.push_back(0);
        }else{
            intermediate_materials.push_back(material);
            intermediate_stack_id.push_back (stack_id);
        }
        intermediate_points.push_back(new_sedimentation_point);
    }*/


//        for(uint i = 1 ;i<sedimenation_data.sediment_history.size()-1;i++){
            for(uint i = 0 ;i<sedimenation_data.sediment_history.size();i++)    {
            //new_sedimentation_point=  initial_point + normalized_distance_vector * step_length * (float )i ;
            new_sedimentation_point =  initial_point;
//            new_sedimentation_point[2] +=  bin_size * roundf(sedimenation_data.material_stack_width[i]/bin_size) ;//* (float )i ;
            new_sedimentation_point[2] +=  sedimenation_data.material_stack_width[i] ;//* (float )i ;

            intermediate_points.push_back(new_sedimentation_point);
            intermediate_materials.push_back(sedimenation_data.sediment_history[i]);
            intermediate_stack_id.push_back (sedimenation_data.material_stack_id[i]);

        }

            intermediate_points.push_back(actual_point);
            intermediate_materials.push_back(sedimenation_data.sediment_history[sedimenation_data.sediment_history.size()-1]);
            intermediate_stack_id.push_back(sedimenation_data.material_stack_id[sedimenation_data.sediment_history.size()-1]);

    }
    if(intermediate_materials.size()>0){
        sedimentation_shader =std::shared_ptr<SedimentationShader> (new SedimentationShader(_id,1.0f,intermediate_points,intermediate_materials,intermediate_stack_id));
    list_of_sedimentation_points.push_back(actual_point);
    list_of_shaders.push_back(sedimentation_shader);
    }
}



set<MyMesh::FaceHandle> SedimentationClassifier::GetSetOfFaces(map<MyMesh::VertexHandle,sedimentationData> selected_vertices){
    set<MyMesh::FaceHandle> selected_faces;

    for(auto const entry: selected_vertices){

        MyMesh::VertexFaceIter vertex_face_circulator;
        vertex_face_circulator = _mesh.vf_iter(entry.first);
        for( ;vertex_face_circulator.is_valid(); ++vertex_face_circulator){
            //if(_mesh.valence(vertex_face_circulator)==4 || j == 0)
                selected_faces.insert(vertex_face_circulator);
        }
    }
    return selected_faces;
}
void SedimentationClassifier::AverageInputData(float treshold,float detail_scale,int K){
    CreatePointCloud();
    std::vector<glm::vec3> new_list_of_sedimentation_points;
    std::vector<std::shared_ptr<AShader>> new_list_of_shaders;
     for(uint i = 0;i<list_of_sedimentation_points.size();i++){
         glm::vec3 const entry = list_of_sedimentation_points[i];
         pcl::PointXYZLNormal new_point;
         //int K = 5;//or 27
         new_point.x = entry[0];
         new_point.y = entry[1];
         new_point.z = entry[2];
         std::vector<int> pointIdxNKNSearch(K);
       //  std::vector<SedimentationShader*> sedimentation_materials_list(K);
         std::vector<float> dists_list(K);
         std::vector<float> ids_list(K);
         float total_dist=0;
         float total= 0;
        float avg = 0;

        std::map<int,float> list_of_used_materials;
         std::vector<float> pointNKNSquaredDistance(K);
         if ( kdtree_input.nearestKSearch (new_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
             glm::vec3 total_position(0,0,0);
             float total_height = 0;
             int counter = 0;
             std::shared_ptr<SedimentationShader> sd =  std::static_pointer_cast<SedimentationShader>(list_of_shaders[pointIdxNKNSearch[0]]);
             float target_id = sd->GetStackId();//sd->GetStackId();
            float mat_id = sd->GetMaterialId();
             for(uint j = 0;j<pointIdxNKNSearch.size();j++){
                 std::shared_ptr<SedimentationShader> sd =  std::static_pointer_cast<SedimentationShader>(list_of_shaders[pointIdxNKNSearch[j]]);
                 float id = sd->GetStackId();//sd->GetMaterialId();
                 if(id==target_id){
                    total_position+= list_of_sedimentation_points[pointIdxNKNSearch[j]];
                    total_height += list_of_sedimentation_points[pointIdxNKNSearch[j]][2];
                    counter++;
                 }

                 glm::vec3 point = list_of_sedimentation_points[pointIdxNKNSearch[j]];
                 //if(point[2]-entry[2]>detail_scale && point[2]-entry[2]<detail_scale*2){
                 //    if(sd->GetMaterialId()>target_id  && sd->GetMaterialId()!= sd->GetMaterialSecondId()){
                 //        int new_id = target_id+1;
                 //        if(new_id == 6)
                 //            new_id = 1;
                 //        list_of_shaders[pointIdxNKNSearch[j]] = std::shared_ptr<AShader>(new SedimentationShader(_id,1.0f,new_id));
                 //    }
                 //}
             }

             /*for(uint j = 0;j<pointIdxNKNSearch.size();j++){
                 std::shared_ptr<SedimentationShader> sd =  std::static_pointer_cast<SedimentationShader>(list_of_shaders[pointIdxNKNSearch[j]]);



             } */

             float avg = counter /((float) pointIdxNKNSearch.size()) ;
             if(avg>treshold){
                std::shared_ptr<SedimentationShader> sd = std::shared_ptr<SedimentationShader>(new SedimentationShader(_id,1.0f,mat_id));
                new_list_of_shaders.push_back(sd);
                glm::vec3 new_position;
                new_position[0] = total_position[0]/(float)counter;
                new_position[1] = total_position[1]/(float)counter;
                new_position[2] = total_position[2]/(float)counter;
                float new_height = total_height/(float) counter;
                new_list_of_sedimentation_points.push_back(new_position);
               // new_position = list_of_sedimentation_points[pointIdxNKNSearch[0]];
               // new_position[2] = new_height;
               // new_list_of_sedimentation_points.push_back(new_position);
             }
/*             for(uint j = 0;j<pointIdxNKNSearch.size();j++){
                 std::shared_ptr<SedimentationShader> sd =  std::static_pointer_cast<SedimentationShader>(list_of_shaders[pointIdxNKNSearch[j]]);
                 float val = sd->GetMaterialId();
               //  avg += val;
                 if(list_of_used_materials.empty()) {

                     list_of_used_materials.insert(std::make_pair(val,0));

                 }else{
                     if(list_of_used_materials.count(val)>0)
                     {
                         list_of_used_materials[val]++;
                     }else{
                         list_of_used_materials.insert(std::make_pair(val,0));
                     }

                 }
             }
             float max = -INFINITY;
             float final_id = -1;
             for(std::pair<int,float>entry:list_of_used_materials){
                if(entry.second>max){
                    max = entry.second;
                    final_id = entry.first;
                }
             }

            // float mean = avg/(float)pointIdxNKNSearch.size();
           /*  float min_dist = INFINITY;
             float final_id = -1;
             for(std::pair<int,float>entry:list_of_used_materials){
                 float dist =std::abs(entry.second-mean);
                 if(dist<min_dist){
                     min_dist = dist;
                    final_id = entry.first;
                 }
             }*/
           // list_of_shaders[pointIdxNKNSearch[0]] = std::shared_ptr<SedimentationShader>(new SedimentationShader(_id,1.0f,target_id));
        }
    }
     list_of_sedimentation_points = new_list_of_sedimentation_points;
     list_of_shaders = new_list_of_shaders;

}


void  SedimentationClassifier::SelectTopMostVertices(map<MyMesh::VertexHandle,sedimentationData> selected_vertices) {
    std::vector<std::shared_ptr<AShader>>  temporary_list_of_shaders;
    std::vector<glm::vec3> temporary_list_of_sedimentation_points;
    //uint i = 0;i<selected_vertices.size();i++

int i = 0;
    for(auto const entry : list_of_shaders){
       i++;
        std::vector<float> temp_mat;
       std::vector<glm::vec3> temp_pts;
        std::shared_ptr<SedimentationShader> sd =std::static_pointer_cast<SedimentationShader>(entry);
       temp_mat = sd->getListOfIntermediateSedimentationMaterials();
       temp_pts = sd->getListOfIntermediateSedimentationPoints();
       int l = temp_mat.size();

        //if(i==l-1){
            temporary_list_of_shaders.push_back(std::shared_ptr<SedimentationShader> (new SedimentationShader(_id,1.0f,temp_mat.back())));
            temporary_list_of_sedimentation_points.push_back(temp_pts.back());
      //  }

    }
list_of_sedimentation_points= temporary_list_of_sedimentation_points;
list_of_shaders = temporary_list_of_shaders;
}

void SedimentationClassifier::AverageOutputData(float treshold,float detail_scale,int K){
    CreatePointCloud();
    std::vector<glm::vec3> new_list_of_sedimentation_points;
    std::vector<std::shared_ptr<AShader>> new_list_of_shaders;
     for(uint i = 0;i<list_of_sedimentation_points.size();i++){
         glm::vec3 const entry = list_of_sedimentation_points[i];
         pcl::PointXYZLNormal new_point;
         //int K = 5;//or 27
         new_point.x = entry[0];
         new_point.y = entry[1];
         new_point.z = entry[2];
         std::vector<int> pointIdxNKNSearch(K);
       //  std::vector<SedimentationShader*> sedimentation_materials_list(K);
         std::vector<float> dists_list(K);
         std::vector<float> ids_list(K);
         float total_dist=0;
         float total= 0;
        float avg = 0;

        std::map<int,float> list_of_used_materials;
         std::vector<float> pointNKNSquaredDistance(K);
         if ( kdtree_input.nearestKSearch (new_point, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
             glm::vec3 total_position(0,0,0);
             float total_height = 0;
             int counter = 0;
             std::shared_ptr<SedimentationShader> sd =  std::static_pointer_cast<SedimentationShader>(list_of_shaders[pointIdxNKNSearch[0]]);
             float target_id = sd->GetMaterialId();
            float mat_id = sd->GetMaterialId();
             for(uint j = 0;j<pointIdxNKNSearch.size();j++){
                 std::shared_ptr<SedimentationShader> sd =  std::static_pointer_cast<SedimentationShader>(list_of_shaders[pointIdxNKNSearch[j]]);
                 float id = sd->GetMaterialId();//sd->GetStackId();//sd->GetMaterialId();
                 if(id==target_id){
                    total_position+= list_of_sedimentation_points[pointIdxNKNSearch[j]];
                    total_height += list_of_sedimentation_points[pointIdxNKNSearch[j]][2];
                    counter++;
                 }

                 glm::vec3 point = list_of_sedimentation_points[pointIdxNKNSearch[j]];
                 //if(point[2]-entry[2]>detail_scale && point[2]-entry[2]<detail_scale*2){
                 //    if(sd->GetMaterialId()>target_id  && sd->GetMaterialId()!= sd->GetMaterialSecondId()){
                 //        int new_id = target_id+1;
                 //        if(new_id == 6)
                 //            new_id = 1;
                 //        list_of_shaders[pointIdxNKNSearch[j]] = std::shared_ptr<AShader>(new SedimentationShader(_id,1.0f,new_id));
                 //    }
                 //}
             }

             /*for(uint j = 0;j<pointIdxNKNSearch.size();j++){
                 std::shared_ptr<SedimentationShader> sd =  std::static_pointer_cast<SedimentationShader>(list_of_shaders[pointIdxNKNSearch[j]]);



             } */

             float avg = counter /((float) pointIdxNKNSearch.size()) ;
             if(avg>treshold){
                std::shared_ptr<SedimentationShader> sd = std::shared_ptr<SedimentationShader>(new SedimentationShader(_id,1.0f,mat_id));
                new_list_of_shaders.push_back(sd);
                glm::vec3 new_position;
                new_position[0] = total_position[0]/(float)counter;
                new_position[1] = total_position[1]/(float)counter;
                new_position[2] = total_position[2]/(float)counter;
                float new_height = total_height/(float) counter;
                new_list_of_sedimentation_points.push_back(new_position);
               // new_position = list_of_sedimentation_points[pointIdxNKNSearch[0]];
               // new_position[2] = new_height;
               // new_list_of_sedimentation_points.push_back(new_position);
             }
/*             for(uint j = 0;j<pointIdxNKNSearch.size();j++){
                 std::shared_ptr<SedimentationShader> sd =  std::static_pointer_cast<SedimentationShader>(list_of_shaders[pointIdxNKNSearch[j]]);
                 float val = sd->GetMaterialId();
               //  avg += val;
                 if(list_of_used_materials.empty()) {

                     list_of_used_materials.insert(std::make_pair(val,0));

                 }else{
                     if(list_of_used_materials.count(val)>0)
                     {
                         list_of_used_materials[val]++;
                     }else{
                         list_of_used_materials.insert(std::make_pair(val,0));
                     }

                 }
             }
             float max = -INFINITY;
             float final_id = -1;
             for(std::pair<int,float>entry:list_of_used_materials){
                if(entry.second>max){
                    max = entry.second;
                    final_id = entry.first;
                }
             }

            // float mean = avg/(float)pointIdxNKNSearch.size();
           /*  float min_dist = INFINITY;
             float final_id = -1;
             for(std::pair<int,float>entry:list_of_used_materials){
                 float dist =std::abs(entry.second-mean);
                 if(dist<min_dist){
                     min_dist = dist;
                    final_id = entry.first;
                 }
             }*/
           // list_of_shaders[pointIdxNKNSearch[0]] = std::shared_ptr<SedimentationShader>(new SedimentationShader(_id,1.0f,target_id));
        }
    }
     list_of_sedimentation_points = new_list_of_sedimentation_points;
     list_of_shaders = new_list_of_shaders;
}

void SedimentationClassifier::CreatePointCloud(){

    pcl::PointCloud<pcl::PointXYZLNormal>::Ptr cloud1 ( new pcl::PointCloud<pcl::PointXYZLNormal>);
      cloud_input = cloud1;

    cloud_input->width = list_of_sedimentation_points.size();//numpoints
    cloud_input->height = 1;
    cloud_input->points.resize (cloud_input->width * cloud_input->height);
    for(uint j = 0;j<list_of_sedimentation_points.size();j++){
    glm::vec3 const entry = list_of_sedimentation_points[j];
          cloud_input->points[j].x = entry[0];
          cloud_input->points[j].y = entry[1];
          cloud_input->points[j].z = entry[2];

        }
    kdtree_input.setInputCloud(cloud_input);
}

std::shared_ptr<AShader> SedimentationClassifier::GetShader(){return _shader;}
void SedimentationClassifier::ClassifyVertices(std::vector<glm::vec3>& list_of_points,
                                               std::vector<std::shared_ptr<AShader>>& list_of_data,
                                               float& details){




    auto selected_vertices = SelectSedimentationPoints();
    for(auto entry : selected_vertices) {
        MyMesh::Point point = _mesh.point(entry.first);
        glm::vec3 actual_point (point[0],point[1],point[2]);


      //  ComputeMockUpData(actual_point,0.5,4);

        ComputeSedimentationParametersForVertex(actual_point,entry.second);
        //delete entry.second;
    }

   auto selected_vertices_copy = selected_vertices;
  CreatePointCloud();

     int _subdiv_levels = 2;
     for(int i=0;i<_subdiv_levels;i++){
         if(i!=0)
         selected_vertices = SelectSedimentationPoints();
          selected_vertices =  SelectPointsForAdaptiveSubdivision(selected_vertices);
         auto set_of_faces = GetSetOfFaces(selected_vertices);
         SubdividerAndInterpolator<MyMesh> catmull(set_of_faces,simulation_data_map);
             catmull.attach(_mesh);
             catmull(1);
             catmull.detach();
    }
 selected_vertices = SelectSedimentationPoints();
    AssignSedimentationParameters(selected_vertices);
    //TPSFirstApprach(selected_vertices);
   //TPSSecondApproach(selected_vertices);
   AverageOutputData(0.0,0.0,10);
   //AverageOutputData(0.0,0.0,10);
//    AverageOutputData(0,0.0,5);
    //AverageOutputData(0.2,0.0);
  //  AverageOutputData(0.2,0.1);
   // AverageData();
//  AssignSedimentationParameters2(selected_vertices);
   // CreatePointCloud();
    //  AssignSedimentationParameters3(selected_vertices);
  // SelectTopMostVertices(selected_vertices);
    list_of_points = list_of_sedimentation_points;
    list_of_data = list_of_shaders;
    details = 0.5;
}


