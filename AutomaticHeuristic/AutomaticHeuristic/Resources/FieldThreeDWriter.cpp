#include "FieldThreeDWriter.h"
using namespace  std;

using namespace boost;
using namespace std;

using namespace Field3D;
using namespace OpenMesh;
FieldThreeDWriter::FieldThreeDWriter(MyMesh mesh,string out_name_file): _mesh(mesh), _out_name_file(out_name_file)
{}


void FieldThreeDWriter::Write()
{

      std::string attribName("attrib");

      // Initialize IO
      initIO();

      //if we are sure that there wont be overhangs we can use numFields = 1, otherwise numFields = num_shaders_param+ id
      // and we would have to use a sparse matrix instead of a dense one
      int numFields = 1;

      DenseFieldf::Vec fields;
        int j = 0;
      for (int i = 0; i < numFields; i++) {
        // Create

        DenseFieldf::Ptr field(new DenseFieldf);

        //SetSize(grid.x,grid.y,num_shaders_parameters+id+depth)
        field->setSize(V3i(300,300,1));
        field->name = "shader_parameters";
        field->attribute = attribName;
        // Fill with values

    MyMesh::VertexIter vertex_iterator(_mesh.vertices_begin());
    MyMesh::VertexIter vertex_iterator_end(_mesh.vertices_end());
    DenseFieldf::iterator fi = field->begin();
    DenseFieldf::iterator fend = field->end();
  for( ; vertex_iterator != vertex_iterator_end && fi != fend; ++vertex_iterator,++fi)
  {

      MyMesh::Point point = _mesh.point( *vertex_iterator );
      V3d vsP;
      V3d lsP(point[0]/(300-1),point[1]/(300-1),0);
      //std::cout<<"point from mesh x "<< point[0] << " y "<< point[1] << " z "<< point[2];
      //  std::count<< point ;
      field->mapping()->localToVoxel(lsP, vsP);
      //std::cout <<" from vsP x "<< vsP[0] << " y "<< vsP[1] << " z "<< vsP[2]<< " fi" << *fi << std::endl;
        *fi = point[2];

  }
  fields.push_back(field);


}
  Field3DOutputFileHDF5 out;
  out.create(_out_name_file);
  for (int i = 0; i < numFields; i++) {
    out.writeScalarLayer<float>(fields[i]);
  }
std::cout<<"end of writing part";


Field<float>::Vec loadedFields;

Field3DInputFile in;

in.open(_out_name_file);

for (int i = 0; i < numFields; i++) {
  Field<float>::Vec fields = in.readScalarLayers<float>("shader_parameters", attribName);
  if (fields.size() != 1) {
    Msg::print("Got the wrong # of fields. Aborting.");
    exit(1);
  }
  loadedFields.push_back(fields[i]);
}

    for(int n= 0;n<numFields;n++)
    {
        //DenseFieldf::Ptr field(new DenseFieldf);
        //field = loadedFields[n];
        for(int i = 0;i<300;i++)
        {
            for(int j = 0;j<300;j++)
            {
                float f = loadedFields[n]->value(i,j,0);
                std::cout << " i "<< i << " j "<< j << " z "<< f<<std::endl;
            }
        }
    }


}
