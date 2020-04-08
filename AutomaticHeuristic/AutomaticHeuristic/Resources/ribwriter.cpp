#include "ribwriter.h"
#include <pointcloud.h>


RIBWriter::RIBWriter(MyMesh mesh,string out_name_file,Camera camera,vector<VertexEditTag> list_of_vertices_change):_mesh(mesh),_rib_file(out_name_file),_cam(camera),_vertex_edit_tag(list_of_vertices_change)
{}


void RIBWriter::WriteVertices()
{
    int i= 0;
     _rib_file<< " \"vertex point P\" [";
     MyMesh::VertexIter v_it, v_end(_mesh.vertices_end());
    for (v_it = _mesh.vertices_sbegin() ; v_it!= v_end;++v_it )
    {
        _rib_file<<" "<< _mesh.point(*v_it);
        if(i%50==0)
            _rib_file<< std::endl;
        i++;
    }
    _rib_file<< " ] ";
}



void RIBWriter::InstanceLandscape()
{
    string displname = "PxrDisplace1";
    string materialId = "PxrSurface1SG";
    string objID = "Landscape"; //13814000-efc9-d4ad-b851-107b442ef99d

    string line;
    line = "AttributeBegin \n";
    //line += "TransformBegin \n Transform [ 0.716910601 -1.38777878e-17 0.697165132 0  0.212405771 0.952457726 -0.218421653 0  -0.6640203 0.304670691 0.682827055 0  0 0 0 1 ] \n";
    //line += " ScopedCoordinateSystem \"perspShape\" \n";
    //line += " TransformEnd \n TransformBegin \n ";
    //line += " Transform [ 0.716910601 -1.38777878e-17 0.697165132 0  0.212405771 0.952457726 -0.218421653 0  -0.6640203 0.304670691 0.682827055 0  0 0 0 1 ] \n";
    //line += " ScopedCoordinateSystem \"persp\" \n";
    //line += " TransformEnd \n Attribute \"Ri\" \"int Sides\" [2] \n";
    line += " Attribute \"dice\" \"string referencecamera\" [\"\"] \n";
    line += " Attribute \"grouping\" \"string membership\" [\"World\"] \n";
    line += " Attribute \"identifier\" \"string name\" [\"|input:Mesh|input:MeshShape\"] \"int id\" [-1847441932] \"int id2\" [0] \n";
    line += " Attribute \"lightfilter\" \"string subset\" [\"\"] \n";
    line += " Attribute \"lighting\" \"int mute\" [0] \"string excludesubset\" [\"\"] \n";
    line += " Attribute \"shade\" \"float relativepixelvariance\" [1] \n";
    line += " Attribute \"trace\" \"int maxspeculardepth\" [4] \"int maxdiffusedepth\" [1] \n";
    line += " Attribute \"user\" \"string __materialid\" [\"" + materialId + "\"]";
    line += " Attribute \"visibility\" \"int camera\" [1] \"int indirect\" [1] \"int transmission\" [1] \n";
    line += " Bxdf \"PxrSurface\" \"PxrSurface1\" \"string __materialid\" [\"" + materialId + "\"] \n";
    line += " ObjectInstance \"" + objID + "\" \n AttributeEnd \n";
    _rib_file << line;

}

void RIBWriter:: WriteNumOfVerticesInEachFace()
{
    _rib_file << " [";
    // since we work witha triMesh the number of vertices for each face is always 3
    for(uint i = 0; i< _mesh.n_faces();i++)
    {
        _rib_file<< " 3";
        if(i%25==0)
            _rib_file<<std::endl;
    }
    _rib_file<< " ] "<<std::endl;

}

void RIBWriter::WriteIndexBuffer()
{
    int i = 0;
      _rib_file<< " ["<<std::endl;
    for (auto& face_handle : _mesh.faces())
    {
        OpenMesh::TriMesh_ArrayKernelT<>::FaceVertexIter face_vertex_circulator;
        for( face_vertex_circulator = _mesh.fv_iter(face_handle);face_vertex_circulator.is_valid();++face_vertex_circulator)
        {
             //std::cout<<"fv handle "<< *face_vertex_circulator<<std::endl;
            _rib_file<<" "<< *face_vertex_circulator;
            if(i%50==0)
                _rib_file<<std::endl;
            i++;
        }
    }
      _rib_file<< " ] "<<std::endl;
}


void RIBWriter::CopyInitialPart(ifstream &myfile)
{
      string line;
      bool found = false;
      bool foundtransform = false;
      size_t pos;
      string target = "ObjectBegin";
      string target2 = "AttributeEnd";
      string target3 = "Transform";
      while ( getline(myfile, line))
      {

        pos = line.find(target);
        if(pos != std::string::npos)
            found =true;
        if(!found)
        {
            if(!foundtransform)
            {
                pos = line.find(target3);
                if(pos!= std::string::npos)
                {
                    foundtransform= true;
                    WriteTransformationMatrix();
                }
                else
                {
                    _rib_file<<line<<std::endl;

                }
            }
            else
            {
                _rib_file<<line<<std::endl;

            }
        }

        else
        {
       //     cout<<line<<std::endl;
            pos = line.find(target2);
            if(pos!= std::string::npos)
            {
               // cout<<line<<std::endl;
                //_rib_file<< line.substr();
                break;
            }
        }
      }
}


void RIBWriter::RotateAlongX(glm::mat4x4 & mat,float angle)
{
    angle = glm::radians(angle);
    mat[1][1] = cos(angle);
    mat[1][2] = sin(angle);
    mat[2][1] = -sin(angle);
    mat[2][2] = cos(angle);
}

void RIBWriter::WriteTransformationMatrix()
{
    stringstream ss;
   // ss<<" Transform [ ";
    glm::mat4x4 viewMatrix = _cam.ViewMatrix();
    float aaa[16] =
    {
         1 , 0, 0, 0,
         0 , 0, -1, 0,
         0 , 1, 0, 0,
         0 , 0, 0, 1
    };

    float bbb[16] =
    {
        -1 , 0, 0, 0,
         0 , 1, 0, 0,
         0 , 0, -1, 0,
         0 , 0, 0, 1
    };


    float ccc[16] =
    {
        1 , 0, 0, 0,
         0 , 1, 0, 0,
         0 , 0, 1, 0,
         0 , 0, 0, 1
    };
    glm::mat4x4 correctiveMatrix_second = glm::make_mat4x4(bbb);
    glm::vec3 pos = _cam.Position();
    glm::mat4x4 correctiveMatrix = glm::make_mat4x4(ccc);  //glm::mat4x4( -1.0f,0.0f,0.0f,0.0f  0.0f,0.0f,1.0f,0.0f  0,1,0,0  0,0,0,1 );
    //glm::inverse(correctiveMatrix_second)*viewMatrix* correctiveMatrix
    //viewMatrix =  glm::inverse(viewMatrix)*(correctiveMatrix);

   // RotateAlongX( viewMatrix,120);
    //viewMatrix[3][0] = 0;
    //viewMatrix[3][1] = 0;
    //viewMatrix[3][2] = 0;
    //viewMatrix[3][3] = 1;



    //viewMatrix[3][0] = -151;
    //viewMatrix[3][1] = 137.203;
    //viewMatrix[3][2] = 287.644;
    //viewMatrix[3][3] = 1;

   // glm::inverse(viewMatrix);
  //  viewMatrix[0][3] = -100 * pos[0];
  //  viewMatrix[1][3] = -100 * pos[1];
  //  viewMatrix[2][3] = 300 * pos[2];
  //  viewMatrix[3][3] = 1;
  //
  //  viewMatrix[0][3] = -151;
  //  viewMatrix[1][3] = 137.203;
  //  viewMatrix[2][3] = 287.644;
  //  viewMatrix[3][3] = 1;


  //  for(int i= 0;i<4;i++)
  //  {
  //      for(int j= 0;j<4;j++)
  //      {
  //          // if( j == 3 && i != 3)
  //          // {
  //          //     ss<<" "<< 100 * pos[i];
  //          // }
  //          // else
  //          {
  //              ss<<" "<< viewMatrix[j][i];
  //          }
  //      }
  //  }
  //  ss<<" ]"<<std::endl;



    //ss << "Rotate "<< -1*_cam.rotX<<" 1 0 0"<<std::endl;
    //ss << "Rotate "<< _cam.rotY<<" 0 1 0"<<std::endl;
    //ss << "Rotate "<< _cam.rotZ<<" 0 0 1"<<std::endl;
    //std::cout<<ss.str();

   ss<<"Transform [ 1 0 0 0 0 -0.8660 -0.5  0 0 0.5 -0.8660 0 -151 137.203 447.644 1 ]"<<std::endl;
    _rib_file<<ss.str();
}

void RIBWriter::CopyFinalPart(ifstream& myfile)
{
      string line;
      while ( getline(myfile, line))
      {
        _rib_file<<line<<std::endl;
      }
}

void RIBWriter::WriteVertexEditTag()
{
    string line;
    for(uint i = 0;i < _vertex_edit_tag.size();i++)
    {
        line = _vertex_edit_tag[i].GetVertexPath();
        _rib_file<<line;
        line = _vertex_edit_tag[i].GetValuesToBeChanged();
        _rib_file<<line;
        line = _vertex_edit_tag[i].GetAttributesToBeChanged();
        _rib_file << line;
    }
}

void RIBWriter::WriteArchive()
{

    string displname = "PxrDisplace1";
    string materialId = "PxrSurface1SG";
    string objID = "Landscape";
    string line = "ObjectBegin \""+ objID+"\"\n";
           line += " Attribute \"Ri\" \"string Orientation\" [\"outside\"] \n" ;
           line += " Attribute \"dice\" \"float micropolygonlength\" [1] \"int rasterorient\" [1] \"int watertight\" [0] \"string referencecamera\" [\"\"] \n";
           line += " Attribute \"displacementbound\" \"float sphere\" [0.5] \"string CoordinateSystem\" [\"object\"] \n";
           line += " Attribute \"identifier\" \"string object\" [\"input:MeshShape\"] \n";
           line += " Attribute \"polygon\" \"int smoothdisplacement\" [1] \n";
           line += " Attribute \"trace\" \"int displacements\" [1] \"int autobias\" [1] \"float bias\" [0.00999999978] \n";
          // line += " Displace \"PxrDisplace\" \" "+ displname + "\" \"string __materialid\" [\"" +materialId+ "\"] \n";
           line += "HierarchicalSubdivisionMesh \"catmull-clark\" ";
           _rib_file<<line;
           WriteNumOfVerticesInEachFace();
           WriteIndexBuffer();
           line = "[\"creasemethod\" \"facevaryingpropagatecorners\" \"interpolateboundary\" ";
           line +=" \"facevaryinginterpolateboundary\"] [0 0 1 1 0 0 1 0 0 1 0 0] [1 1 3] [] [\"chaikin\"] ";
           _rib_file<<line;
           WriteVertexEditTag();
           WriteVertices();
           WriteSimulationData();
           _rib_file<< "ObjectEnd \n";
           InstanceLandscape();
}
void RIBWriter:: Write()
{
      ifstream myfile("../../Data/mountainsceneTemplate.rib");
      if(!myfile.is_open() || !_rib_file.is_open() )
          std::cout<<"err here"<<std::endl;
      CopyInitialPart(myfile);

      string displname = "PxrDisplace1";
      string materialId = "PxrSurface1SG";
      string objID = "Landscape";
      string line = "ObjectBegin \""+ objID+"\"\n";
             line += " Attribute \"Ri\" \"string Orientation\" [\"outside\"] \n" ;
             line += " Attribute \"dice\" \"float micropolygonlength\" [1] \"int rasterorient\" [1] \"int watertight\" [0] \"string referencecamera\" [\"\"] \n";
             line += " Attribute \"displacementbound\" \"float sphere\" [0.5] \"string CoordinateSystem\" [\"object\"] \n";
             line += " Attribute \"identifier\" \"string object\" [\"input:MeshShape\"] \n";
             line += " Attribute \"polygon\" \"int smoothdisplacement\" [1] \n";
             line += " Attribute \"trace\" \"int displacements\" [1] \"int autobias\" [1] \"float bias\" [0.00999999978] \n";
             line += " Displace \"PxrDisplace\" \" "+ displname + "\" \"string __materialid\" [\"" +materialId+ "\"] \n";
             line += "HierarchicalSubdivisionMesh \"catmull-clark\" ";
             _rib_file<<line;
             WriteNumOfVerticesInEachFace();
             WriteIndexBuffer();
             line = "[\"creasemethod\" \"facevaryingpropagatecorners\" \"interpolateboundary\" ";
             line +=" \"facevaryinginterpolateboundary\"] [0 0 1 1 0 0 1 0 0 1 0 0] [1 1 3] [] [\"chaikin\"] ";
             _rib_file<<line;
             WriteVertices();
             WriteSimulationData();
             _rib_file<< "ObjectEnd \n";
             InstanceLandscape();
             CopyFinalPart(myfile);
             std::cout<<"write ended bb"<< std::endl;
}




std::ostream &operator<< (std::ostream &out, const glm::vec3 &vec)
{
    out << vec.x << " " << vec.y << " "<< vec.z << endl;

    return out;
}


void RIBWriter::WriteSimulationData()
{
    auto shader_parameters_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, ShaderParameters*>(_mesh, "shader_parameters");
    int i=0;
    string newstring = " \"varying float simulation_data\"[";
    _rib_file<<newstring;
    for (auto& vertex_handle : _mesh.vertices())
    {
        ShaderParameters* const shader_param = shader_parameters_data_wrapper[vertex_handle];
        _rib_file<< shader_param->getId()<<".0"<< endl;
        if(i%50==0)
            _rib_file<< std::endl;
        i++;
    }
    _rib_file<< " ]";
    for(int j= 0;j<3;j++)
    {
        string newstring = " \"vertex float shader_property_";
        _rib_file<<newstring;
        _rib_file<<j;
        _rib_file << "\" [";

//        newstring += i + ;
        i=0;
        for (auto& vertex_handle : _mesh.vertices())
        {

            ShaderParameters* shader_param = shader_parameters_data_wrapper[vertex_handle];
            _rib_file<< shader_param->getValue(j)<< endl;
            if(i%50==0)
                _rib_file<< std::endl;
            i++;
        }
        _rib_file<< " ]  ";
    }

//    newstring = " \"vertex float[3] shader_vector";
//    _rib_file<<newstring;
//    _rib_file << "\" [";
//    i = 0;
//    for (auto& vertex_handle : _mesh.vertices())
//    {
//        ShaderParameters* shader_param = shader_parameters_data_wrapper[vertex_handle];
//        _rib_file<< shader_param->getVector()<< endl;
//        if(i%50==0)
//            _rib_file<< std::endl;
//        i++;
//    }
//    _rib_file<< " ]  ";
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
    _rib_file<<string_to_write.str();
}
