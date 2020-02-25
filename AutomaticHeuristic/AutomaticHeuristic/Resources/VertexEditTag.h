#ifndef VERTEXEDIT_H
#define VERTEXEDIT_H

#include <string.h>
#include <stdlib.h>
#include <sstream>
#include <iostream>
#include <vector>
#include <iostream>

struct VertexChanges
{
    std::vector<int> vertex_path;
    std::vector<float> new_values;
};

class VertexEditTag
{
    std::vector<VertexChanges> list_of_vertex_changes;
    std::vector<int> indices_to_be_changed;
    std::string vertex_path;
    std::vector<VertexChanges> list_of_parameters_to_be_changed;
    public:
        VertexEditTag();
        VertexEditTag(std::vector<int> list_of_indices_inside_shader_parameters);
        void AddVertexChange(VertexChanges vertex_change);
        std::string GetVertexPath();
        std::string GetAttributesToBeChanged();
        std::string GetValuesToBeChanged();
};

#endif // VERTEXEDIT_H
