#include "ribwriter.h"

RIBWriter::RIBWriter(string out_name_file)
{
    ofstream rib_file (out_name_file);

}

// Pattern "PxrMix" "PxrMix1" "reference float mix" ["simulation_mask:resultF"]
// "color color1" [0.3 0.3 0.3] "reference color color2" ["PxrOSL1:resultRGB"] "int clampMix" [0]
//

void MixNode(RIBNode node1,RIBNode node2,RIBNode node3)
{
    cout<<"FUNCTION NOT DEFINIED YET!!!"<<endl;
    abort();
}

void RIBWriter::MixNode()
{
   stringstream string_to_write;
    string_to_write<< " Pattern \"PxrMix\" ";
    string name = "PxrMix1";
    bool ref_mix = true;
    bool ref_color_1 = true;
    bool ref_color_2 = false;

    string ref = "reference";

    string ref_color_1_name = "PxrOSL1:resultRGB";
    string ref_color_2_name = "PxrOSL1:resultRGB";
    string color_1 = "0.3 0.3 0.3";
    string color_2 = "0.3 0.3 0.3";
    string ref_name = "simulation_mask:resultF";
    string_to_write<< " \" "<< name << " \" ";

    if(ref_mix)
    {
        string_to_write<< " \"reference  float mix\" [ \" "<<ref_name<< " \" ] ";

    }
    if(ref_color_1)
    {
     string_to_write<<" \" reference color color1 \" [ \" "<<ref_color_1_name << " \" ]";
    }
    else
    {
        string_to_write << " \" color color1 \" [ "<< color_1 << " ]";
    }


    if(ref_color_2)
    {
     string_to_write<<" \" reference color color2 \" [ \" "<<ref_color_2_name << " \" ]";
    }
    else
    {
        string_to_write << " \" color color2 \" [ "<< color_2 << " ]";
    }
    string_to_write << " \"int clampMix \" [0]  "<<endl;
  //  string_to_write<< "\"reference" <<"\"float mix\" [\"simulation_mask:resultF\"]
  //          \"color color1\" [0.3 0.3 0.3] \"reference color color2\" [\"PxrOSL1:resultRGB \"] \" int clampMix \" [0]  "<<endl;
    cout<<string_to_write.str();
    rib_file<<string_to_write.str();
}
