#include "screeclassifier.h"

ScreeClassifier::ScreeClassifier(MyMesh mesh) :AClassifier()
{
    _mesh = mesh;
    _repose_angle = 42;
    _treshold = 3;
}


ScreeClassifier::ScreeClassifier(MyMesh mesh,float repose_angle,float treshold) : AClassifier()
{
    _mesh = mesh;
    _repose_angle = repose_angle;
    _treshold = treshold;
}
map<MyMesh::VertexHandle,ShaderParameters*> ScreeClassifier::ClassifyVertices()
{
        MyMesh::Normal up_direction = Vec3f(0,0,1);
        MyMesh::FaceIter face_iterator,face_iterator_end(_mesh.faces_end());
        map<MyMesh::VertexHandle,ShaderParameters*> selected_faces ;

        for(face_iterator=_mesh.faces_begin();face_iterator != face_iterator_end;++face_iterator)
        {
            MyMesh::Normal mynormal = _mesh.normal(*face_iterator);
            float dot_result = dot(mynormal,up_direction);
            if(dot_result<0)
            {
                cout<<"here pls"<<endl;
            }
            float resulting_angle_in_radians = acos(dot_result);
            float resulting_angle_in_degree = resulting_angle_in_radians* (180.0/  M_PI);

            if(resulting_angle_in_degree <= _repose_angle + _treshold && resulting_angle_in_degree >= _repose_angle - _treshold)
            {
                MyMesh::FaceVertexIter face_vertex_iter = _mesh.fv_iter(face_iterator);
                for( face_vertex_iter;face_vertex_iter.is_valid();++face_vertex_iter)
                {
                ShaderParameters* shader_parameter = new ShaderParameters(_id,10);
                  selected_faces.insert(pair<MyMesh::VertexHandle,ShaderParameters*>(*face_vertex_iter,shader_parameter));
                }
            }
        }
return selected_faces;
}
