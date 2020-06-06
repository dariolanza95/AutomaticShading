#include "ribwriter.h"
#include <pointcloud.h>


//RIBWriter::RIBWriter(MyMesh mesh, string out_name_file, Camera camera, vector<VertexEditTag> list_of_vertices_change, vector<AShader *> list_of_used_shaders):_mesh(mesh),
//    _rib_file(out_name_file),_cam(camera),_vertex_edit_tag(list_of_vertices_change),list_of_used_shaders(list_of_used_shaders)
//{}
RIBWriter::RIBWriter(MyMesh mesh,string out_name_file,string shaders_path,string plugins_path,string output_name_image,Camera camera,vector<AShader*> list_of_used_shaders):
    _mesh(mesh),
_rib_file(out_name_file),
  _shaders_path(shaders_path),
  _plugins_path(plugins_path),
  _output_name_image(output_name_image),
  _cam(camera),
  _list_of_used_shaders(list_of_used_shaders)
{  }
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
    string material_id = _bxdf_node->GetMaterialId();
    string material_name = _bxdf_node->GetName();
    string _objID = "Landscape";

    string line;
    line = "AttributeBegin \n";
    line += " Attribute \"dice\" \"string referencecamera\" [\"\"] \n";
    line += " Attribute \"grouping\" \"string membership\" [\"World\"] \n";
    line += " Attribute \"identifier\" \"string name\" [\"|input:Mesh|input:MeshShape\"] \"int id\" [-1847441932] \"int id2\" [0] \n";
    line += " Attribute \"lightfilter\" \"string subset\" [\"\"] \n";
    line += " Attribute \"lighting\" \"int mute\" [0] \"string excludesubset\" [\"\"] \n";
    line += " Attribute \"shade\" \"float relativepixelvariance\" [1] \n";
    line += " Attribute \"trace\" \"int maxspeculardepth\" [4] \"int maxdiffusedepth\" [1] \n";
    line += " Attribute \"user\" \"string __materialid\" [\"" + material_id + "\"]";
    line += " Attribute \"visibility\" \"int camera\" [1] \"int indirect\" [1] \"int transmission\" [1] \n";
    line += " Bxdf \"PxrSurface\" \""+material_name+"\" \"string __materialid\" [\"" + material_id + "\"] \n";
    line += " ObjectInstance \"" + _objID + "\" \n AttributeEnd \n";
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


void RIBWriter::WriteBXDF(RIBNode* node){
    stringstream res;
    stringstream name("PxrSurface");
    name<<1;
    res<<"Bxdf \"PxrSurface\" \""<<name.str()<<"\" \"reference color diffuseColor\" [\""<<node->GetName()<<"\"] ";
    //res<<"\"string __materialid\" [\""<<PxrSurface1SG<<"\"]"<<std::endl;
    _rib_file<< res.str();

}
/*

void RIBWriter::WriteDisplacement(RIBNode* node){
    Displace "PxrDisplace" "PxrDisplace1" "reference float dispScalar" ["PxrWorleyD:resultF"]
            "reference float dispAmount" ["PxrWorleyD:resultDispl"] "int enabled" [1]
            "vector dispVector" [0 0 0] "vector modelDispVector" [0 0 0]
            "string __materialid" ["PxrSurface1SG"]
}*/

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

string RIBWriter::WriteTransformationMatrix()
{
    stringstream ss;/*
    ss<<" Transform [ ";
    glm::mat4x4 viewMatrix = _cam.ViewMatrix();
    viewMatrix = _cam.RIBMatrix();

    for(int i= 0;i<4;i++)
    {
        for(int j= 0;j<4;j++)
        {
            if( i == 3 && j != 3)
            {
                if(j==2)
                    ss<<" "<< -1 * viewMatrix[i][j] * _cam.GetScalingCostant();
                else
                    ss<<" "<<  viewMatrix[i][j] * _cam.GetScalingCostant();

            }
            else
            {
                ss<<" "<< viewMatrix[i][j];
            }
        }
    }
    ss<<" ]"<<std::endl;

*/

    //ss << "Rotate "<< -1*_cam.rotX<<" 1 0 0"<<std::endl;
    //ss << "Rotate "<< _cam.rotY<<" 0 1 0"<<std::endl;
    //ss << "Rotate "<< _cam.rotZ<<" 0 0 1"<<std::endl;
    //std::cout<<ss.str();
//ss<<" Transform [ 1 0 0 0 0 -0.8660 -0.5  0 0 0.5 -0.8660 0 -151 137.203 547.644 1 ]"<<std::endl;
 //  ss<<"Transform [ 1 0 0 0 0 -0.8660 -0.5  0 0 0.5 -0.8660 0 -151 137.203 447.644 1 ]"<<std::endl;
  // ss<<"  Transform [ 1 0 0 0 0 -0.8660 -0.5  0 0 0.5 -0.8660 0 -151 137.203 447.644 1 ]"<<std::endl;
    //ss<<" Transform [  1 0 0 0 0 -0.241805 -0.970205 0 0 0.970205 -0.241805 0 -164.45 43.2274 314.337 1 ]"<<std::endl;;
   // ss<<" Transform [  0.921877 -0.0256774 0.386597 0 0.384045 -0.0712268 -0.920367 0 0.0512425 0.996948 -0.0557853 0 -219.769 -11.0496 182.197 1 ]"<<std::endl;
 //   ss<<" Transform [  0.933785 -0.113397 -0.339363 0 -0.357194 -0.350701 -0.865463 0 -0.0209266 0.929384 -0.367977 0 -105.163 65.7922 341.325 1 ]";
//    ss<<" Transform [  1 0 0 0 0 0.138112 -0.990313 0 0 0.990313 0.138112 0 -151.494 -29.5937 107.547 1 ] "<<std::endl;
   // ss<<" Transform [  0.998505 5.58794e-09 0.0546532 0 0.0544339 -0.0893556 -0.994381 0 0.00489066 0.995869 -0.0892218 0 -172.423 16.756 312.531 1 ]"<<std::endl;
   // ss<<"Transform [  0.98823 -2.46614e-06 -0.152963 0 -0.140098 -0.401226 -0.904976 0 -0.0614017 0.915756 -0.396502 0 -202.102 83.1623 351.97 1 ]"<<std::endl;
   // ss<<" Transform [  0.998441 -3.72529e-09 -0.0558192 0 -0.0555465 -0.0986873 -0.993523 0 -0.00551111 0.995074 -0.0985334 0 -148.504 0 388.701 1 ]"<<std::endl;
  //  ss<<"Transform [  0.995919 0.000103932 0.0902541 0 0.0853934 -0.324781 -0.941873 0 0.0292195 0.945736 -0.323464 0 -140.859 46.2262 378.789 1 ]"<<std::endl;
    //ss<<" Transform [  1 0 0 0 -0 -0.0836014 -0.996427 0 0 0.996427 -0.0836014 0 -149.833 23.9523 293.692 1 ]"<<std::endl;
  //  ss<< " Transform [  0.996296 1.11759e-08 0.0859931 0 0.0859742 0.0209919 -0.996037 0 -0.00180183 0.999741 0.0209143 0 -122.256 -25.8181 317.678 1 ]"<<std::endl;
  //  ss<<"  Transform [  1 0 0 0 0 0.159969 -0.987046 0 0 0.987046 0.159969 0 -149.168 -56.5756 265.938 1 ]"<<std::endl;
ss<<" Transform [  1 0 0 0 -0 -0.0104214 -0.999906 0 0 0.999906 -0.0104214 0 -141.527 1.71687 296.649 1 ]"<<std::endl;
    return ss.str();
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
    float displacementBound = 1;
    string line = "ObjectBegin \""+ objID+"\"\n";
           line += " Attribute \"Ri\" \"string Orientation\" [\"outside\"] \n" ;
           line += " Attribute \"dice\" \"float micropolygonlength\" [1] \"int rasterorient\" [1] \"int watertight\" [1] \"string referencecamera\" [\"\"] \n";
           line += " Attribute \"displacementbound\" \"float sphere\" [1.5] \"string CoordinateSystem\" [\"object\"] \n";
           line += " Attribute \"identifier\" \"string object\" [\"input:MeshShape\"] \n";
           line += " Attribute \"polygon\" \"int smoothdisplacement\" [1] \n";
           line += " Attribute \"trace\" \"int displacements\" [1] \"int autobias\" [1] \"float bias\" [0.00999999978] \n";
          // line += " Displace \"PxrDisplace\" \" "+ displname + "\" \"string __materialid\" [\"" +materialId+ "\"] \n";
             bool subdiv = true;
           if(subdiv)
            line += "HierarchicalSubdivisionMesh \"catmull-clark\" ";
           else
            line += "PointsPolygons ";
           _rib_file<<line;
           WriteNumOfVerticesInEachFace();
           WriteIndexBuffer();
           if(subdiv)
           {
                line = "[\"creasemethod\" \"facevaryingpropagatecorners\" \"interpolateboundary\" ";
                line +=" \"facevaryinginterpolateboundary\"] [0 0 1 1 0 0 1 0 0 1 0 0] [1 1 3] [] [\"chaikin\"] ";
           }
           _rib_file<<line;
           WriteVertexEditTag();
           WriteVertices();
       //    WriteSimulationData();
           _rib_file<< "ObjectEnd \n";
           InstanceLandscape();
}


void RIBWriter::WriteInialization(){
    stringstream string_to_write;

    //string _output_file_name("try03");
    //string _plugins_path("C:/Users/Dario/Documents/rfm:::C:\\Program Files\\Pixar\\RenderManForMaya-22.6/lib/plugins:@:./");
    //string _shaders_path("C:/Users/Dario/Documents/rfm:::C:\\Program Files\\Pixar\\RenderManProServer-22.6/lib/shaders:@:./");
    string _procedural_path("C:/Users/Dario/Documents/rfm:::C:\\Program Files\\Pixar\\RenderManForMaya-22.6/lib/2019:@");
    string _output_path("./");

//    string plugins_path("C:/Users/Dario/Documents/rfm:::C:\\Program Files\\Pixar\\RenderManForMaya-22.6/lib/plugins:@:./");
//    string shaders_path("C:/Users/Dario/Documents/rfm:::C:\\Program Files\\Pixar\\RenderManProServer-22.6/lib/shaders:@:./");


    string_to_write<<"##RenderMan RIB"<<std::endl;
    string_to_write<<"# RenderMan 22.6 build 1987751"<<std::endl;
    string_to_write<<"# Export: "<<_output_path+_output_name_image<<".rib"<<std::endl;
    string_to_write<<"##rifcontrol insert begin -rif abind -rifend"<<std::endl;
    string_to_write<<"version 3.04999995"<<std::endl;
    string_to_write<<"Option \"ribparse\" \"string varsubst\" [\"\"]"<<std::endl;
    string_to_write<<"Option \"ribparse\" \"string varsubst\" [\"$\"]"<<std::endl;
    string_to_write<<"Option \"Ri\" \"int Frame\" [1] \"float PixelVariance\" [0.00999999978] \"string PixelFilterName\" [\"gaussian\"] "<<std::endl;
    string_to_write<<"\"float[2] PixelFilterWidth\" [2 2] \"int[2] FormatResolution\" [600 400] \"float FormatPixelAspectRatio\" [1.5] \"float[2] Clipping\" [0.100000001 10000] \"float[4] ScreenWindow\" [-1 1 -0.5625 0.5625]  "<<std::endl;
//    string_to_write<<"\"float[4] CropWindow\" [0.0 0.5 0.5 1.0]"<<std::endl;
    string_to_write<<" \"float[2] Shutter\" [0 0]"<<std::endl;
    string_to_write<<"Option \"bucket\" \"string order\" [\"circle\"]"<<std::endl;
    string_to_write<<"Option \"lighting\" \"int selectionlearningscheme\" [1]"<<std::endl;
    string_to_write<<"Option \"limits\" \"int threads\" [8] \"int[2] bucketsize\" [16 16] \"float[3] othreshold\" [0.995999992 0.995999992 0.995999992] \"float deepshadowerror\" [0.0100000007] \"int texturememory\" [4194304] \"int geocachememory\" [4194304] \"int opacitycachememory\" [2097152]"<<std::endl;

    string_to_write<<"Option \"searchpath\" \"string rixplugin\" [\""<<_plugins_path<<"\"]"<<std::endl;
    string_to_write<<" \"string shader\" [\""<< _shaders_path<<"\"] "<<std::endl;
    string_to_write<<"\"string texture\" [\"@\"] \"string procedural\" [\""<< _procedural_path <<"\"]"<<std::endl;
    //  " \"string dirmap\" ["[ \"UNC\" \"/myserver/\" \"Z:/\"]"]
    string_to_write<<"Option \"statistics\" \"int level\" [1] \"string xmlfilename\" [\"./"<<_output_path + _output_name_image<<"stat\"]"<<std::endl;
    string_to_write<<"Option \"user\" \"int osl:batched\" [1] \"int osl:verbose\" [0] \"int osl:statisticslevel\" [0] \"float sceneUnits\" [1] \"int iesIgnoreWatts\" [1]"<<std::endl;
    string_to_write<<"Integrator \"PxrPathTracer\" \"PxrPathTracer\" \"int maxPathLength\" [10]"<<std::endl;
    string_to_write<<" \"int maxContinuationLength\" [-1] \"int maxNonStochasticOpacityEvents\" [0] \"string sampleMode\" [\"bxdf\"] \"int numLightSamples\" [1] \"int numBxdfSamples\" [1] \"int numIndirectSamples\" [1] \"int numDiffuseSamples\" [1] \"int numSpecularSamples\" [1] \"int numSubsurfaceSamples\" [1] \"int numRefractionSamples\" [1] \"int allowCaustics\" [0] \"int accumOpacity\" [0] \"int rouletteDepth\" [4] \"float rouletteThreshold\" [0.200000003] \"int clampDepth\" [2] \"float clampLuminance\" [10]"<<std::endl;
    string_to_write<<"DisplayChannel \"color Ci\" \"string source\" [\"Ci\"]"<<std::endl;
    string_to_write<<"DisplayChannel \"float a\" \"string source\" [\"a\"]"<<std::endl;
    string_to_write<<"Projection \"PxrPerspective\" \"float fov\" [70] \"float fStop\" [9.99999968e+37] \"float focalLength\" [0] \"float focalDistance\" [1]"<<std::endl;
    string_to_write<< WriteTransformationMatrix();
    string_to_write<<"Camera \"|persp|perspShape\" \"float shutterOpenTime\" [0] \"float shutterCloseTime\" [1] \"int apertureNSides\" [0] \"float apertureAngle\" [0] \"float apertureRoundness\" [0] \"float apertureDensity\" [0] \"float dofaspect\" [1] \"float nearClip\" [0.100000001] \"float farClip\" [10000]"<<std::endl;
    string_to_write<<"Display \""<<_output_path + _output_name_image<<".exr\" \"openexr\" \"Ci,a\" \"int asrgba\" [1] \"string storage\" [\"scanline\"] \"string exrpixeltype\" [\"half\"] \"string compression\" [\"zips\"] \"float compressionlevel\" [45] \"string camera\" [\"|persp|perspShape\"]"<<std::endl;
    string_to_write<<"WorldBegin"<<std::endl<<"AttributeBegin"<<std::endl;
    _rib_file<<string_to_write.str();
}


void RIBWriter:: Write()
{
     //BXDFNode* rib_bxdf = new BXDFNode(rib_mix);

     ifstream myfile("../../Data/mountainsceneTemplate.rib");
      if(!myfile.is_open() || !_rib_file.is_open() )
          std::cout<<"err here"<<std::endl;


   //   CopyInitialPart(myfile);
      WriteInialization();
      WriteShadersList();
          //  WriteDisplacementLogic();
      string displname = "PxrDisplace1";
      string materialId = "PxrSurface1SG";
      string objID = "Landscape";
      stringstream line;
             line <<"AttributeEnd"<<std::endl;
             line << "ObjectBegin \""<< objID<<"\"\n";
             line << " Attribute \"Ri\" \"string Orientation\" [\"outside\"] \n" ;
             line << " Attribute \"dice\" \"float micropolygonlength\" [1] \"int rasterorient\" [1] \"int watertight\" [1] \"string referencecamera\" [\"\"] \n";
             line << " Attribute \"displacementbound\" \"float sphere\" ["<<_displ_node->GetDisplBound()<<"] \"string CoordinateSystem\" [\"object\"] \n";
             line << " Attribute \"identifier\" \"string object\" [\"input:MeshShape\"] \n";
             line << " Attribute \"polygon\" \"int smoothdisplacement\" [1] \n";
             line << " Attribute \"trace\" \"int displacements\" [1] \"int autobias\" [1] \"float bias\" [0.00999999978] \n";
             line << " Displace \"PxrDisplace\" \" "<< displname << "\" \"string __materialid\" [\"" << _bxdf_node->GetMaterialId() << "\"] \n";
             bool subdiv = true;
           if(subdiv)
            line << "HierarchicalSubdivisionMesh \"catmull-clark\" ";
           else
            line << "PointsPolygons ";
             _rib_file<<line.str();
             WriteNumOfVerticesInEachFace();
             WriteIndexBuffer();
             if(subdiv)
             {
                 line.str("");
                  line << "[\"creasemethod\" \"facevaryingpropagatecorners\" \"interpolateboundary\" ";
                  line << " \"facevaryinginterpolateboundary\"] [0 0 1 1 0 0 1 0 0 1 0 0] [1 1 3] [] [\"chaikin\"] ";
                   _rib_file<<line.str();
             }


             WriteVertices();
             WriteSimulationData();
             _rib_file<< "ObjectEnd \n";
             InstanceLandscape();
             RIBLight light(0.3f);
             _rib_file<<light.Write();
             _rib_file<<"WorldEnd"<<std::endl;
          //   CopyFinalPart(myfile);
             std::cout<<"write ended bb"<< std::endl;
}




std::ostream &operator<< (std::ostream &out, const glm::vec3 &vec)
{
    out << vec.x << " " << vec.y << " "<< vec.z << endl;

    return out;
}


void RIBWriter::WriteSimulationData()
{
   /* auto shader_parameters_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, ShaderParameters*>(_mesh, "shader_parameters");
    int i=0;
    string newstring = " \"varying float simulation_data\"[";
    _rib_file<<newstring;
    for (auto& vertex_handle : _mesh.vertices())
    {
        ShaderParameters* const shader_param = shader_parameters_data_wrapper[vertex_handle];
        _rib_file<< shader_param->GetId()<<".0"<< endl;
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
*/
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

void MixNode(RIBShaderNode node1,RIBShaderNode node2,RIBShaderNode node3)
{
    cout<<"FUNCTION NOT DEFINIED YET!!!"<<endl;
    abort();
}


void RIBWriter::WriteDisplacementLogic(){
    stringstream string_to_write("");
    RIBNode* rib_mask ;
    RIBNode* rib_shader1;
    RIBNode* rib_shader2;
    RIBNode* rib_result;
    RIBNode* rib_mix;
    int i = 0;
    for(AShader* shader : _list_of_used_shaders){
        BlendMode displ_blend_mode = shader->GetDisplBlendMode();
        if(i==0){
            rib_shader1 = new RIBConstant(glm::vec3( 0.0f, 0.0f, 0.0f));
        }else   {
            rib_shader1 = rib_result;
        }
        rib_shader2 = new RIBShaderNode(shader);
        if(displ_blend_mode == BlendMode::Add)
        {
            rib_result = new RIBAddNode(rib_shader1,rib_shader2);
        }
        else
        {
            rib_mask = new RIBMaskNode(shader);
            rib_result =     new RIBMixNode(rib_mask,rib_shader1,rib_shader2,true);

        }
        string_to_write<< rib_result->WriteNode()<<std::endl;

        //string_to_write<<rib_mask->WriteNode()<<std::endl;
        //if(i==0)
        //    string_to_write<<rib_shader1->WriteNode()<<std::endl;
        //string_to_write<<rib_shader2->WriteNode()<<std::endl;
        //string_to_write<<rib_mix->WriteNode()<<std::endl;

        _rib_file<<string_to_write.str();
        string_to_write.str(std::string());
        std::cout<<string_to_write.str()<<std::endl;
        i++;

    }

}


void RIBWriter::WriteShadersList()
{
    stringstream string_to_write("");
    int i = 0;
    RIBNode* rib_mask ;
    RIBNode* rib_shader1;
    RIBNode* rib_shader2;
    RIBNode* rib_mix = nullptr;
    RIBNode* rib_displ = nullptr;
    RIBNode* previous_rib_displ;
    for(AShader* shader : _list_of_used_shaders){

        BlendMode displ_blend_mode = shader->GetDisplBlendMode();

        rib_mask = new RIBMaskNode(shader);
        if(i==0){
            rib_shader1 = new RIBConstant(glm::vec3(((float)186)/256,((float)151)/256,((float)90)/256));
        }else   {
            rib_shader1 = rib_mix;
        }
        rib_shader2 = new RIBShaderNode(shader);
        rib_mix =     new RIBMixNode(rib_mask,rib_shader1,rib_shader2);

        string_to_write<<rib_mask->WriteNode()<<std::endl;
        if(i==0)
            string_to_write<<rib_shader1->WriteNode()<<std::endl;
        string_to_write<<rib_shader2->WriteNode()<<std::endl;
        string_to_write<<rib_mix->WriteNode()<<std::endl;
        if(i==0)
            rib_shader1 = new RIBConstant(0);
        else
            rib_shader1 = rib_displ;
        if(displ_blend_mode == BlendMode::Add)
        {
            rib_displ= new RIBAddNode(rib_shader1,rib_shader2);
        }
        else
        {
            rib_displ= new RIBAddNode(rib_shader1,rib_shader2,rib_mask,BlendMode::Overlay);
        }
        string_to_write<<rib_displ->WriteNode()<<std::endl;
        _rib_file<<string_to_write.str();
        string_to_write.str("");

    i++;
    }


    BXDFNode* rib_bxdf = new BXDFNode(rib_mix);
    _bxdf_node = rib_bxdf;
    string_to_write<< rib_bxdf->WriteNode()<<std::endl;

    DisplNode* displ_node = new DisplNode(rib_bxdf,rib_displ,1.0f);
    _displ_node = displ_node;
    string_to_write<< displ_node->WriteNode()<<std::endl;

    _rib_file<<string_to_write.str();

}
/*
RIBShaderNode RIBWriter::MixNode(string name,RIBShaderNode mixNode,RIBShaderNode color1,RIBShaderNode color2)
{
    stringstream string_to_write;
    string_to_write<< " Pattern \"PxrMix\" ";

//    string name = "PxrMix1";
    bool ref_mix = true;
    bool ref_color_1 = true;
    bool ref_color_2 = true;

    string ref = "reference";

    string ref_color_1_name = "PxrOSL1:resultRGB";
    string ref_color_2_name = "PxrOSL1:resultRGB";
    string color_1 = "0.3 0.3 0.3";
    string color_2 = "0.3 0.3 0.3";
    string ref_mix_name = "simulation_mask:resultF";
    string_to_write<< " \" "<< name << " \" ";
    if(1)
    {
        string_to_write<< " \"reference  float mix\" [ \" "<< mixNode.GetName()<< ":"<< mixNode.GetResultF()<<" \" ] ";

    }
    if(ref_color_1)
    {
     string_to_write<<" \" reference color color1 \" [ \" "<<color1.GetName()<<":"<<color1.GetRGB() << " \" ]";
    }
    else
    {
        string_to_write << " \" color color1 \" [ "<< color_1 << " ]";
    }

    if(ref_color_2)
    {
     string_to_write<<" \" reference color color2 \" [ \" "<<color1.GetName()<<":"<<color1.GetRGB()<< " \" ]";
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
    return RIBShaderNode("hola","paris");
            //RIBShaderNode(name);
}*/
