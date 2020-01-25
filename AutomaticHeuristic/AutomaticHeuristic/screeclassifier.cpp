#include "screeclassifier.h"

ScreeClassifier::ScreeClassifier(MyMesh mesh)
{
    _mesh = mesh;
    _repose_angle = 42;
    _treshold = 3;
}


ScreeClassifier::ScreeClassifier(MyMesh mesh,float repose_angle,float treshold)
{
    _mesh = mesh;
    _repose_angle = repose_angle;
    _treshold = treshold;
}

map<MyMesh::FaceHandle,float> ScreeClassifier::ClassifyVertices()
{

        MyMesh::Face face;

        MyMesh::Normal up_direction = Vec3f(0,-1,0);
        MyMesh::FaceIter face_iterator,face_iterator_end(_mesh.faces_end());
        map<MyMesh::FaceHandle,float> selected_faces ;
        int counter = 0;
        for(face_iterator=_mesh.faces_begin();face_iterator != face_iterator_end;++face_iterator)
        {
            face = _mesh.face(*face_iterator);
            MyMesh::Normal mynormal = _mesh.normal(*face_iterator);

            float dot_result = dot(mynormal,up_direction);
            float resulting_angle_in_radians = acos(dot_result);
            float resulting_angle_in_degree = resulting_angle_in_radians* (180.0/  M_PI);
            resulting_angle_in_degree = 90 - resulting_angle_in_degree;
            //resulting_angle_in_degree = resulting_angle_in_degree > 0 ? resulting_angle_in_degree : 0;

            if(resulting_angle_in_degree <= _repose_angle + _treshold && resulting_angle_in_degree >= _repose_angle - _treshold)
            {
                  MyMesh::FaceHandle face_handle = face_iterator.handle();
                  selected_faces.insert(pair<MyMesh::FaceHandle,float>(face_handle,1));
                  counter++;
            }

        }
        return selected_faces;


}
