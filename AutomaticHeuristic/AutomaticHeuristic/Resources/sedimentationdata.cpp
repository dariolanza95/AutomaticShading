#include "sedimentationdata.h"

sedimentationData::sedimentationData(glm::vec3 initial_position,std::vector<float> sediment_history,std::vector<float> material_stack_width):sediment_history(sediment_history)
,initial_position(initial_position),material_stack_width(material_stack_width)
{

}
sedimentationData::~sedimentationData(){}
