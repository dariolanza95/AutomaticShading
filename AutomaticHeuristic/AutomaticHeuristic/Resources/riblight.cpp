#include "riblight.h"

using namespace std;


int RIBLight::_shared_id = 0;
RIBLight::RIBLight(float intensity): _intensity(intensity)
{
    _unique_id = _shared_id++;
}

RIBLight::RIBLight():_intensity(0.1f)
{
    _unique_id = _shared_id++;
}

std::string RIBLight::Write(){
stringstream string_to_write;
stringstream name;
name<<"Light"<<_unique_id;
//First Define the object
string_to_write<<"ObjectBegin \""<<name.str()<<"\""<<std::endl;
string_to_write<<"Light \"PxrDomeLight\" \"PxrDomeLightShape\" \"float intensity\" ["<<_intensity<<"] \"float exposure\" [0]"<<std::endl;
string_to_write<<" \"color lightColor\" [1 1 1] \"float diffuse\" [1] \"int enableShadows\" [1] \"color shadowColor\" [0 0 0] "<<std::endl;
string_to_write<< "\"float importanceMultiplier\" [1]"<<std::endl;
string_to_write<< "ObjectEnd"<<std::endl;
//Then instantiate it


string_to_write<< "AttributeBegin"<<std::endl;
//string_to_write<< "TransformBegin"<<std::endl;
//			Transform [ 0.716910601 -1.38777878e-17 0.697165132 0  0.212405771 0.952457726 -0.218421653 0  -0.6640203 0.304670691 0.682827055 0  0 0 0 1 ]
//			ScopedCoordinateSystem "perspShape"
//		TransformEnd
//		TransformBegin
//			Transform [ 0.716910601 -1.38777878e-17 0.697165132 0  0.212405771 0.952457726 -0.218421653 0  -0.6640203 0.304670691 0.682827055 0  0 0 0 1 ]
//			ScopedCoordinateSystem "persp"
//		TransformEnd
string_to_write<< "Attribute \"Ri\" \"int Sides\" [2] "<<std::endl;
string_to_write<< "Attribute \"dice\" \"string referencecamera\" [\"\"]"<<std::endl;
string_to_write<< "Attribute \"grouping\" \"string membership\" [\"PxrDomeLightShape World\"]"<<std::endl;
string_to_write<< "Attribute \"identifier\" \"string name\" [\"|PxrDomeLight|PxrDomeLightShape\"] \"int id\" [296404090] \"int id2\" [0]"<<std::endl;
string_to_write<< "Attribute \"lighting\" \"int mute\" [0]"<<std::endl;
string_to_write<< "Attribute \"shade\" \"float relativepixelvariance\" [1]"<<std::endl;
string_to_write<< "Attribute \"trace\" \"int maxspeculardepth\" [4] \"int maxdiffusedepth\" [1]"<<std::endl;
string_to_write<< "Attribute \"visibility\" \"int camera\" [1] \"int indirect\" [0] \"int transmission\" [1]"<<std::endl;
        //Attribute "user" "string __materialid" ["PxrSurface1SG"]
        //Bxdf "PxrSurface" "PxrSurface1" "string __materialid" ["PxrSurface1SG"]

string_to_write << "Transform [ 1 0 0 0  0 1 0 0  0 0 1 0  5.57277536 -21.8803768 8.3754158 1 ]"<<std::endl;
string_to_write<<"ObjectInstance \""<<name.str() <<"\""<<std::endl;

string_to_write<<"AttributeEnd"<<std::endl;
return string_to_write.str();
}
