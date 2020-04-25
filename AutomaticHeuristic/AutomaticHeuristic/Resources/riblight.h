#ifndef RIBLIGHT_H
#define RIBLIGHT_H

#include <string>
#include <sstream>

class RIBLight
{
    static int _shared_id;
    int _unique_id;
    float _intensity;
public:
    RIBLight();
    RIBLight(float intensity);
    std::string Write();
};

#endif // RIBLIGHT_H
