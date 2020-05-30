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

std::shared_ptr<SimulationData> SimulationData::Interpolate(std::shared_ptr<SimulationData> sd1,float t){
    std::map<SimulationDataEnum,float> new_map_of_floats;
    std::map<SimulationDataEnum,glm::vec3> new_map_of_vectors;
    std::map<SimulationDataEnum,std::vector<float>> new_map_of_lists;
    for(pair<SimulationDataEnum,float> entry: map_of_floats){
        if(sd1->map_of_floats.count(entry.first)){
            float a = entry.second;
            float b =sd1->map_of_floats.at(entry.first);
            float c = a*(1-t) + t*b;
            new_map_of_floats.insert(make_pair (entry.first,c));
        }else{
            new_map_of_floats.insert(entry);
        }
    }

    for(pair<SimulationDataEnum,float> entry: sd1->map_of_floats){
        if(new_map_of_floats.count(entry.first)){
            continue;
        }else{
            new_map_of_floats.insert(entry);
        }
    }


    for(pair<SimulationDataEnum,glm::vec3> entry: map_of_vectors){
        if(sd1->map_of_vectors.count(entry.first)){
            glm::vec3 a = entry.second;
            glm::vec3 b =sd1->map_of_vectors.at(entry.first);
            glm::vec3 c = a*(1-t) + t*b;
            new_map_of_vectors.insert(make_pair (entry.first,c));
        }else{
            new_map_of_vectors.insert(entry);
        }
    }

    for(pair<SimulationDataEnum,glm::vec3> entry: sd1->map_of_vectors){
        if(new_map_of_floats.count(entry.first)){
            continue;
        }else{
            new_map_of_vectors.insert(entry);
        }
    }

    for(pair<SimulationDataEnum,std::vector<float>> entry: map_of_lists){
        if(sd1->map_of_floats.count(entry.first)){
            std::vector<float> first_list = entry.second;
            std::vector<float> second_list =sd1->map_of_lists.at(entry.first);
            std::vector<float> new_list;
            int l_min,l_max;
            if(first_list.size() > second_list.size())
            {
                for(int i=0;i<second_list.size();i++){
                    float a = first_list[i];
                    float b = second_list[i];
                    float c = a*(1-t) + t*b;
                    new_list.push_back(c);
                }
                for(int i=second_list.size();i<first_list.size();i++){
                    float a = first_list[i];
                    float b = 0;
                    float c = a*(1-t) + t*b;
                    new_list.push_back(c);
                }


            }else{
                for(int i=0;i<first_list.size();i++){
                    float a = first_list[i];
                    float b = second_list[i];
                    float c = a*(1-t) + t*b;
                    new_list.push_back(c);
                }
                for(int i=first_list.size();i<second_list.size();i++){
                    float a = 0;
                    float b = second_list[i];
                    float c = a*(1-t) + t*b;
                    new_list.push_back(c);
                }
            }





            new_map_of_lists.insert(make_pair (entry.first,new_list));
        }else{
            new_map_of_lists.insert(entry);
        }
    }

    for(pair<SimulationDataEnum,std::vector<float>> entry: sd1->map_of_lists){
        if(new_map_of_lists.count(entry.first)){
            continue;
        }else{
            new_map_of_lists.insert(entry);
        }
    }

    return std::shared_ptr<SimulationData>(new SimulationData(new_map_of_floats,new_map_of_vectors,new_map_of_lists));

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
    bool waiting_for_a_list = false;
    int list_counter = 0;
int x = tokens.size();
       for(uint i = 0 ; i< tokens.size();i++) {
           string str= tokens[i];
            float converted = -1;
           if (!isANumber(str,converted)) {
              if(SimulationDataEnummap.count(str)>0 && !waiting_for_a_number){
                  data_name = SimulationDataEnummap[str];
                  waiting_for_a_number = true;
                  waiting_for_a_vector = false;
                  waiting_for_a_list = false;
                }
              else{
                  if(waiting_for_a_number && str[0] == 'v'){
                      waiting_for_a_vector = true;
                      waiting_for_a_list = false;
                      continue;
                  }
                  else {
                      if(waiting_for_a_number && str[0] == 'l'){
                          waiting_for_a_list = true;
                          waiting_for_a_vector = false;
                          waiting_for_a_number = false;
                          continue;
                      }
                      else{
                          stringstream err("Symbol not present ");
                          ss<<str;
                          throw ExceptionClass(err.str());
                      }
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
                    --i;
                    waiting_for_a_number = false;
               }
               else {
                    if(waiting_for_a_number){
                       map_of_floats.insert(pair<SimulationDataEnum,float> (data_name,converted));
                        waiting_for_a_number = false;
                    }
                    else{
                        if(waiting_for_a_list){
                            int list_size=0;
                            str= tokens[i];
                            if(isANumber(str,converted)){
                                list_size = converted;
                            }else{
                                string err("list without a size");
                                throw ExceptionClass (err);
                            }
                            std::vector<float> list;
                            for(int j = 0;j<list_size;j++){
                                i++;
                                str = tokens[i];
                                if(isANumber(str,converted)){
                                    list.push_back(converted);
                                }else{
                                    string err("incomplete list");
                                    throw ExceptionClass (err);
                                }
                            }
                            waiting_for_a_number = false;
                            /*if(list.size()<13){
                                list.clear();
                                list.push_back(1);
                                list.push_back(2);
                            }
                           /* list.clear();
                            list.push_back(1);
                            list.push_back(2);
                            list.push_back(3);
                            list.push_back(4);
                            list.push_back(5);
                            list.push_back(6);/*
                            list.push_back(1);
                            list.push_back(2);
                            list.push_back(3);*/
                            list.resize(40);
                            map_of_lists.insert(pair<SimulationDataEnum,std::vector<float>> (data_name,list));

                        }else{
                            stringstream err("Found a number without an associated name");
                            throw ExceptionClass(err.str());
                        }
                    }
               }

           }
       }


}

void SimulationData::getData(SimulationDataEnum data_enum, glm::vec3& data){
     if(map_of_vectors.count(data_enum)>0)
        data = map_of_vectors[data_enum];

}

void SimulationData::getData(SimulationDataEnum data_enum, std::vector<float>& data){
    if(map_of_lists.count(data_enum)>0)
        data = map_of_lists[data_enum];

}


void SimulationData::getData(SimulationDataEnum data_enum, float& data){
    if(map_of_floats.count(data_enum)>0)
        data = map_of_floats[data_enum];

}


SimulationData::SimulationData(){}

SimulationData::SimulationData( std::map<SimulationDataEnum,float> map_of_floats,
                                std::map<SimulationDataEnum,glm::vec3> map_of_vectors,
                                std::map<SimulationDataEnum,std::vector<float>> map_of_lists):
    map_of_floats(map_of_floats),
    map_of_lists(map_of_lists),
    map_of_vectors(map_of_vectors)
{}

SimulationData::SimulationData(const string str)
{
try{
         readLine(str);
    }
    catch(ExceptionClass exc){
        std::cout<<exc.what()<<std::endl;
    }
}

SimulationData::~SimulationData(){
  // std::cout<<"Call dtor"<<std::endl;
    // for(pair<SimulationDataEnum,std::vector<float>>  entry : map_of_lists){
   //     entry.second.~vector();
   // }
}
/*SimulationData SimulationData:: operator+( SimulationData sd) {
        SimulationData res;
        for(pair<SimulationDataEnum,float> p : this->map_of_floats){
            if(sd.map_of_floats.count(p.first)>0){
                p.second += sd.map_of_floats.at(p.first);
            }else{
                p.second += 0;
            }
            res.map_of_floats.insert(p);
        }

        for(pair<SimulationDataEnum,glm::vec3> p : this->map_of_vectors){
            if(sd.map_of_vectors.count(p.first)>0){
                p.second += sd.map_of_vectors.at(p.first);
            }else{
                p.second += 0;
            }
            res.map_of_vectors.insert(p);
        }

        return res;
     }


SimulationData* SimulationData:: operator/(SimulationData *sd) {
        SimulationData* res = new SimulationData();
        for(pair<SimulationDataEnum,float> p : this->map_of_floats){
            if(sd->map_of_floats.count(p.first)>0){
                p->second /= sd.map_of_floats.at(p.first);
            }else{
                p->second /= 1;
            }
            res.map_of_floats.insert(p);
        }

        for(pair<SimulationDataEnum,glm::vec3> p : this->map_of_vectors){
            if(sd->map_of_vectors.count(p.first)>0){
                p.second /= sd.map_of_vectors.at(p.first);
            }else{
                p->second /= 1;
            }
            res->map_of_vectors.insert(p);
        }

        return res;
     }

*/
