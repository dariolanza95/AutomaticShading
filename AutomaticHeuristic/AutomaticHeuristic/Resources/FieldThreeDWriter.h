#ifndef FIELDTHREEDWRITER_H
#define FIELDTHREEDWRITER_H


// --------------------OpenMesh----------------------------
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include "ShaderParameters.h"
#include <iostream>
#include <vector>

#include <boost/lexical_cast.hpp>
#include <boost/timer.hpp>

#include <Field3D/SparseField.h>
#include <Field3D/SparseFile.h>
#include <Field3D/Field3DFile.h>
#include <Field3D/FieldInterp.h>
#include <Field3D/InitIO.h>
#include <Field3D/Log.h>
typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;

class FieldThreeDWriter
{
    MyMesh _mesh;
    std::string _out_name_file;
public:
    FieldThreeDWriter(MyMesh mesh,std::string out_name_file);
    void Write();
};

#endif // FIELDTHREEDWRITER_H
