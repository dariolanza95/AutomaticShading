#ifndef SEDIMENTATIONSHADER_H
#define SEDIMENTATIONSHADER_H
#ifndef GLM_ENABLE_EXPERIMENTAL
#define GLM_ENABLE_EXPERIMENTAL
#endif
#include "AShader.h"
#include "utils.h"
#include <sstream>
#include <glm/gtx/closest_point.hpp>
#include <glm/vec3.hpp>
#include <glm/glm.hpp>
#include <math.h>
class SedimentationShader : public AShader
{
    float material_id;
    float stack_id;
    std::vector<glm::vec3> list_of_intermediate_sedimentation_points;
    std::vector<float> list_of_intermediate_sedimentation_materials;
    std::vector<int> list_of_intermediate_stack_ids;
    glm::vec3 ProjectAlongStackDirection(glm::vec3 actual_point, float &projection);

public:
    bool GetLineId2(int material_id,glm::vec3 actual_point,int &stack_id,glm::vec3 &min_point,glm::vec3 &max_point);

    bool GetLineId(int id,glm::vec3 actual_point,glm::vec3 &min_point,glm::vec3 &max_point);
    float GetStackId();
    bool GetLineSedimentationStackId(int id,glm::vec3 actual_point,int &stack_id);

    ~SedimentationShader();
    bool getClosestPointWithSameId(glm::vec3 actual_point,int id,glm::vec3& closest_point);

    uint findClosestPointInList(glm::vec3 actual_point,float& dist);
    int getClosestPointIndex(glm::vec3 actual_point);
    float GetMaterialId();
    SedimentationShader(int id, float confidence,float material_id);
    SedimentationShader(int id, float confidence,float material_id,float stack_id);
    SedimentationShader(int id);
    SedimentationShader(int id,float confidence,std::vector<glm::vec3> list_of_intermediate_sedimentation_points,std::vector<float> list_of_intermediate_sedimentation_materials,
                        std::vector<int> list_of_intermediate_stack_ids);
    float utilityFunct(glm::vec3 actual_point, float id, float &weight);
    std::vector<glm::vec3> getListOfIntermediateSedimentationPoints();
    std::vector<float> getListOfIntermediateSedimentationMaterials();
    float getClosestPointMatId(glm::vec3 actual_point,float& dist);
    float  GetMaterialId(SedimentationShader sd1,glm::vec3 actual_point);
    void getSerializedData(std::vector<float>& data);
    void allocateData(std::vector<float> &data);
    std::string getShaderName();
    void getCloudPathName(std::string& path);
    void getSerializedTypes(std::vector<char*>& types,std::vector<char*>& var_names,int& num_variables);
};

#endif // SEDIMENTATIONSHADER_H
