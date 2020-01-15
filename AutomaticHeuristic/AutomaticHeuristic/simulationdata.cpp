#include "simulationdata.h"

class SimulationDataException: public exception
{
  int _num_of_variables;
  int _num_of_data_in_file;
public:
  SimulationDataException(int num_of_variables,int num_of_data_in_file)
  {
      _num_of_variables = num_of_variables;
      _num_of_data_in_file = num_of_data_in_file;
  }
  virtual const char* what() const throw()
  {
    stringstream res;
    res<<"In the file there are at least"<< _num_of_data_in_file << " variables but the programm is expecting  "<< _num_of_variables<<endl;
    string temp(res.str());
    return temp.c_str();
  }
};


SimulationData::SimulationData(map<string,float> map)
{
    _map = map;
}

SimulationData::SimulationData(vector<string> nameVariables, const string& str)
{

    //vector<float> cont;
        char delim = ' ';
        int i = 0;
        std::size_t current, previous = 0;
        stringstream res;
        current = str.find(delim);
        string substring;
        while (current != std::string::npos)
        {
            substring=str.substr(previous, current - previous);
            //cont.push_back(atof(substring));
            if( i <nameVariables.size())
            {
                if(!substring.empty())
                    _map.insert(pair<string,float> (nameVariables[i++],stof(substring,NULL)));

            }
            else
            {
                res<<"In the file there are at least"<< i << " variables but the programm is expecting  "<< nameVariables.size()<<endl;
                string temp(res.str());
                throw temp.c_str();
            }
            previous = current + 1;
            current = str.find(delim, previous);
        }

        if(i<nameVariables.size())
        {
            if(!substring.empty())
                _map.insert(pair<string,float> (nameVariables[i++],stof(substring,NULL)));
        }
        else
        {
            res<<"In the file there are at least"<< i << " variables but the programm is expecting  "<< nameVariables.size()<<endl;
            string temp(res.str());
            throw temp.c_str();
        }
        if(_map.empty())
            throw "map is empty";
}
