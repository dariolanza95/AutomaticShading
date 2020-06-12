#ifndef SEDIMENTATIONDATA_H
#define SEDIMENTATIONDATA_H
#include <glm/vec3.hpp>
#include <glm/glm.hpp>
#include <vector>

class sedimentationData
{
public:
    std::vector<float> sediment_history;
    glm::vec3 initial_position;
    std::vector<float> material_stack_width;
    sedimentationData(glm::vec3 initial_position,std::vector<float> sediment_history,std::vector<float> material_stack_width);
    ~sedimentationData();
};

#endif // SEDIMENTATIONDATA_H
