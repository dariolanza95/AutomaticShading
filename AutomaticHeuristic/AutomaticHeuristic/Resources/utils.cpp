#include "utils.h"
namespace AutomaticShaders {

    Utils::Utils()
    {

    }

    char* Utils::fromStringToChar(std::string input)
    {
        char* output;
        output = (char *) malloc((input.size()+1) * sizeof(char));
        input.copy(output, input.size() + 1);
        output[input.size()] = '\0';
        return output;
    }
}
