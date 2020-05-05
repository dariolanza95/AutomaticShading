#include "simulationdata.h"
using namespace std;

bool isANumber(const string str,float& converted)
{
    char cstr[str.size() + 1];
    strcpy(cstr, str.c_str());
    char* p;
    converted = strtof(cstr, &p);
    if(*p)
        return false;
    else
        return true;
}
void SimulationData::readLine(const string line )
{
    std::string buf;
    std::stringstream ss(line);
    std::vector<std::string> tokens;
    while (ss >> buf)
        tokens.push_back(buf);
    SimulationDataEnum data_name;
    bool waiting_for_a_number = false;
    bool waiting_for_a_vector = false;

       for(uint i = 0 ; i< tokens.size();i++) {
           string str= tokens[i];
            float converted = -1;
           if (!isANumber(str,converted)) {
              if(SimulationDataEnummap.count(str)>0 && !waiting_for_a_number){
                  data_name = SimulationDataEnummap[str];
                  waiting_for_a_number = true;
                  waiting_for_a_vector = false;
                }
              else{
                  if(waiting_for_a_number && str[0] == 'v'){
                      waiting_for_a_vector = true;
                      continue;
                  }
                  else {
                      stringstream err("Symbol not present ");
                      ss<<str;
                      throw ExceptionClass(err.str());
                  }

              }
           }
           else {
               if(waiting_for_a_vector){
                    glm::vec3 vec;
                    for(int j = 0;j<3;j++,i++)
                    {
                        str= tokens[i];
                        if(isANumber(str,converted)){
                            vec[j] = converted;
                        }else{
                            string err("Incomplete vector");
                            throw ExceptionClass (err);
                        }
                    }
                    map_of_vectors.insert(pair<SimulationDataEnum,glm::vec3> (data_name,vec));
               }
               else {
                    if(waiting_for_a_number){
                       map_of_floats.insert(pair<SimulationDataEnum,float> (data_name,converted));
                        waiting_for_a_number = false;
                    }
                    else{
                        stringstream err("Found a number without an associated name");
                        throw ExceptionClass(err.str());
                    }
               }

           }
       }}

void SimulationData::getData(SimulationDataEnum data_enum, glm::vec3& data){
    if(map_of_vectors.count(data_enum)>0)
        data = map_of_vectors[data_enum];

}


void SimulationData::getData(SimulationDataEnum data_enum, float& data){
    if(map_of_floats.count(data_enum)>0)
        data = map_of_floats[data_enum];

}

SimulationData::SimulationData(const string str)
{
try{
         readLine(str);
    }
    catch(ExceptionClass exc){
        std::cout<<exc.what()<<std::endl;
    }
}
