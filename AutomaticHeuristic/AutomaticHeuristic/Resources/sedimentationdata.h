#ifndef SEDIMENTATIONDATA_H
#define SEDIMENTATIONDATA_H
#include <glm/vec3.hpp>
#include <glm/glm.hpp>
#include <vector>

class sedimentationData
{
public:
    std::vector<int> sediment_history;
    std::vector<int> material_stack_id;
    std::vector<float> material_stack_width;
    glm::vec3 initial_position;
    sedimentationData(glm::vec3 initial_position,std::vector<int> sediment_history,std::vector<float> material_stack_width,std::vector<int> stack_id);
    ~sedimentationData();
};

#endif // SEDIMENTATIONDATA_H
