#ifndef SUBDIVIDERANDINTERPOLATOR_IMPL_H
#define SUBDIVIDERANDINTERPOLATOR_IMPL_H

#include <iostream>
#include <sstream>
#include <fstream>
#include <string.h>
#include <stdlib.h>
#include "subdividerandinterpolator.h"

/* ========================================================================= *
 *                                                                           *
 *                               OpenMesh                                    *
 *           Copyright (c) 2001-2015, RWTH-Aachen University                 *
 *           Department of Computer Graphics and Multimedia                  *
 *                          All rights reserved.                             *
 *                            www.openmesh.org                               *
 *                                                                           *
 *---------------------------------------------------------------------------*
 * This file is part of OpenMesh.                                            *
 *---------------------------------------------------------------------------*
 *                                                                           *
 * Redistribution and use in source and binary forms, with or without        *
 * modification, are permitted provided that the following conditions        *
 * are met:                                                                  *
 *                                                                           *
 * 1. Redistributions of source code must retain the above copyright notice, *
 *    this list of conditions and the following disclaimer.                  *
 *                                                                           *
 * 2. Redistributions in binary form must reproduce the above copyright      *
 *    notice, this list of conditions and the following disclaimer in the    *
 *    documentation and/or other materials provided with the distribution.   *
 *                                                                           *
 * 3. Neither the name of the copyright holder nor the names of its          *
 *    contributors may be used to endorse or promote products derived from   *
 *    this software without specific prior written permission.               *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED *
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A           *
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER *
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,  *
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,       *
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR        *
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    *
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING      *
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS        *
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.              *
 *                                                                           *
 * ========================================================================= */

//=============================================================================
//
//  CLASS CatmullClarkT - IMPLEMENTATION
//
//=============================================================================

//== INCLUDES =================================================================

#include <OpenMesh/Tools/Utils/MeshCheckerT.hh>
#include <glm/vec3.hpp>
#include <glm/glm.hpp>
//== NAMESPACES ===============================================================

//namespace OpenMesh   { // BEGIN_NS_OPENMESH
//namespace Subdivider { // BEGIN_NS_SUBVIDER
//namespace Uniform    { // BEGIN_NS_UNIFORM

//== IMPLEMENTATION ==========================================================

template <typename MeshType, typename RealType>
bool
SubdividerAndInterpolator< MeshType, RealType > ::prepare(MeshType &_m)
{
  _m.add_property( vp_pos_ );
  _m.add_property( ep_pos_ );
  _m.add_property( fp_pos_ );
  _m.add_property( creaseWeights_ );

  // initialize all weights to 0 (= smooth edge)
  for( EdgeIter e_it = _m.edges_begin(); e_it != _m.edges_end(); ++e_it)
     _m.property(creaseWeights_, *e_it ) = 0.0;

  typename MeshType::FaceVertexIter face_vertex_circulator;
  typename MeshType::FaceEdgeIter face_edge_circulator;

      for(FaceHandle fh : _set_of_faces){
        face_vertex_circulator = _m.fv_iter(fh);
        for( ;face_vertex_circulator.is_valid(); ++face_vertex_circulator){
           _set_of_vertices.insert(face_vertex_circulator);
        }
        face_edge_circulator = _m.fe_iter(fh);
        for( ;face_edge_circulator.is_valid(); ++face_edge_circulator){ 
            _set_of_edges.insert(face_edge_circulator);
        }
    }


  return true;
}

//-----------------------------------------------------------------------------

template <typename MeshType, typename RealType>
bool
SubdividerAndInterpolator<MeshType,RealType>::cleanup( MeshType& _m  )
{
  _m.remove_property( vp_pos_ );
  _m.remove_property( ep_pos_ );
  _m.remove_property( fp_pos_ );
  _m.remove_property( creaseWeights_ );
  return true;
}

//-----------------------------------------------------------------------------



template <typename MeshType, typename RealType>
bool
SubdividerAndInterpolator<MeshType,RealType>::subdivide( MeshType& _m , size_t _n , const bool _update_points)
{
  // Do _n subdivisions
  for ( size_t i = 0; i < _n; ++i)
  {
std::cout<<"Subdivision"<<std::endl;
      //_set_of_faces
    // Compute face centroid
  /*  typename MeshType::FaceIter f_itr   = _set_of_faces.begin();// .faces_begin();
    typename MeshType::FaceIter f_end   = _set_of_faces.end();
    for ( ; f_itr != f_end; ++f_itr)
    {
      Point centroid;
      _m.calc_face_centroid( *f_itr, centroid);
      _m.property( fp_pos_, *f_itr ) = centroid;
    }
*/
       for(auto const face: _set_of_faces){
           /*MeshType::fv voh_it = _m.fv_iter( face );
           for( ; voh_it.is_valid(); ++voh_it )
           {
             pos += _m.point( _m.to_vertex_handle( *voh_it ) );
             valence+=1.0;
           }*/
           Point centroid;
           float valence =0;
           valence = _m.valence(face);
           if(face.idx() == 1273){
               std::cout <<"ajoh centroid"<<std::endl;
           }
           _m.calc_face_centroid( face, centroid);
           glm::vec3 v(centroid[0],centroid[1],centroid[2]);
           if(glm::any(glm::isnan(v))){
               std::cout<<"centroid is 0 valence is "<< valence<<std::endl;
           }
           _m.property( fp_pos_, face ) = centroid;
       }

    // Compute position for new (edge-) vertices and store them in the edge property
    //EdgeIter e_itr   = _set_of_edges.begin();
    //EdgeIter e_end   = _set_of_edges.end();
    //for ( ; e_itr != e_end; ++e_itr)
    //  compute_midpoint( _m, *e_itr, _update_points );

    for(auto const edge: _set_of_edges){
        compute_midpoint( _m, edge, _update_points );
    }






    // position updates activated?
    if(false)
    {
      // compute new positions for old vertices
      //VertexIter v_itr = _set_of_vertices.begin();//_m.vertices_begin();
      //VertexIter v_end = _set_of_vertices.end();  //_m.vertices_end();

        for(auto const vertex: _set_of_vertices){
            update_vertex(_m, vertex);
        }
//        for ( ; v_itr != v_end; ++v_itr)
//        update_vertex( _m, *v_itr );

      // Commit changes in geometry
//      v_itr  = _set_of_vertices.begin();//_m.vertices_begin();
//      for ( ; v_itr != v_end; ++v_itr)
//        _m.set_point(*v_itr, _m.property( vp_pos_, *v_itr ) );
//
      for(auto const vertex: _set_of_vertices){
          _m.set_point(vertex,_m.property( vp_pos_, vertex ));
//          update_vertex(_m, vertex);
      }
    }




    // Split each edge at midpoint stored in edge property ep_pos_;
    // Attention! Creating new edges, hence make sure the loop ends correctly.
//    e_itr = _set_of_edges.begin();//_m.edges_begin();
//    for ( ; e_itr != e_end; ++e_itr)
//      split_edge( _m, *e_itr );

    for(auto const edge: _set_of_edges){
        split_edge(_m, edge);
    }

    VertexIter v_itr = _m.vertices_begin();
    VertexIter v_end = _m.vertices_end();
/*for(;v_itr!= v_end;++v_itr){
    Point p = _m.point(*v_itr);
    glm::vec3 v (p[0],p[1],p[2]);
    if(glm::any(glm::isnan(v))){
        std::cout<<"ERR here"<<std::endl;
    }
}*/

    // Commit changes in topology and reconsitute consistency
    // Attention! Creating new faces, hence make sure the loop ends correctly.
//    f_itr   = _set_of_faces.begin();// _m.faces_begin();
//    for ( ; f_itr != f_end; ++f_itr)
//      split_face( _m, *f_itr);
    for(auto const face: _set_of_faces){
        split_face(_m, face);
    }





#if defined(_DEBUG) || defined(DEBUG)
    // Now we have an consistent mesh!
    assert( OpenMesh::Utils::MeshCheckerT<MeshType>(_m).check() );
#endif
  }

  _m.update_normals();
  return true;
}

//-----------------------------------------------------------------------------

template <typename MeshType, typename RealType>
void
SubdividerAndInterpolator<MeshType,RealType>::split_face( MeshType& _m, const FaceHandle& _fh)
{
  /*
     Split an n-gon into n quads by connecting
     each vertex of fh to vh.

     - _fh will remain valid (it will become one of the quads)
     - the halfedge handles of the new quads will
     point to the old halfedges
   */

  // Since edges already refined (valence*2)
  size_t valence = _m.valence(_fh)/2;

  // new mesh vertex from face centroid
  VertexHandle vh = _m.add_vertex(_m.property( fp_pos_, _fh ));


  HalfedgeHandle hend = _m.halfedge_handle(_fh);
  HalfedgeHandle hh = _m.next_halfedge_handle(hend);

  auto shader_parameters_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, ShadersWrapper*>(_m, "shader_parameters");
  auto sim_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle,SimulationData*> (_m,"simulation_data");



  VertexHandle  t_vh1( _m.to_vertex_handle(hend));
  //for now just  a simple NN interpolation should work
  ShadersWrapper* shader_param = shader_parameters_data_wrapper[t_vh1];
  ShadersWrapper* new_shader_param = new ShadersWrapper(*shader_param);
     shader_parameters_data_wrapper[vh] = new_shader_param;

     SimulationData* sim_data= sim_data_wrapper[t_vh1];
     SimulationData* new_sim_data= new SimulationData(*sim_data);
   sim_data_wrapper[vh] = new_sim_data;


  HalfedgeHandle hold = _m.new_edge(_m.to_vertex_handle(hend), vh);

  _m.set_next_halfedge_handle(hend, hold);
  _m.set_face_handle(hold, _fh);

  hold = _m.opposite_halfedge_handle(hold);

  for(size_t i = 1; i < valence; i++)
  {
    HalfedgeHandle hnext = _m.next_halfedge_handle(hh);

    FaceHandle fnew = _m.new_face();

    _m.set_halfedge_handle(fnew, hh);

    HalfedgeHandle hnew = _m.new_edge(_m.to_vertex_handle(hnext), vh);

    _m.set_face_handle(hnew,  fnew);
    _m.set_face_handle(hold,  fnew);
    _m.set_face_handle(hh,    fnew);
    _m.set_face_handle(hnext, fnew);

    _m.set_next_halfedge_handle(hnew, hold);
    _m.set_next_halfedge_handle(hold, hh);
    _m.set_next_halfedge_handle(hh, hnext);
    hh = _m.next_halfedge_handle(hnext);
    _m.set_next_halfedge_handle(hnext, hnew);

    hold = _m.opposite_halfedge_handle(hnew);
  }

  _m.set_next_halfedge_handle(hold, hh);
  _m.set_next_halfedge_handle(hh, hend);
  hh = _m.next_halfedge_handle(hend);
  _m.set_next_halfedge_handle(hend, hh);
  _m.set_next_halfedge_handle(hh, hold);

  _m.set_face_handle(hold, _fh);

  _m.set_halfedge_handle(vh, hold);
}

//-----------------------------------------------------------------------------

template <typename MeshType, typename RealType>
void
SubdividerAndInterpolator<MeshType,RealType>::split_edge( MeshType& _m, const EdgeHandle& _eh)
{
  HalfedgeHandle heh     = _m.halfedge_handle(_eh, 0);
  HalfedgeHandle opp_heh = _m.halfedge_handle(_eh, 1);

  HalfedgeHandle new_heh, opp_new_heh, t_heh;
  VertexHandle   vh;
  VertexHandle   vh1( _m.to_vertex_handle(heh));
  VertexHandle   vh2( _m.to_vertex_handle(opp_heh));
  Point          zero(0,0,0);
  // new vertex
  vh                = _m.new_vertex( zero );
  _m.set_point( vh, _m.property( ep_pos_, _eh ) );
 Point pos = _m.point(vh);
  glm::vec3 v (pos[0],pos[1],pos[2]);
  if(glm::any(glm::isnan(v))){
      std::cout<<"ERR here in split_edge"<<std::endl;
}
  //copy data from previous vertex
  auto shader_parameters_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, ShadersWrapper*>(_m, "shader_parameters");
  ShadersWrapper* shader_param_v_1 = shader_parameters_data_wrapper[vh1];
  ShadersWrapper* shader_param_v_2 = shader_parameters_data_wrapper[vh2];

  ShadersWrapper* new_shader_param = new ShadersWrapper(*shader_param_v_1);
          //(ShadersWrapper::interpolate(shader_param_v_1,shader_param_v_2,0));
  auto sim_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle,SimulationData*> (_m,"simulation_data");
//  auto OpenMesh::HandleToPropHandle<MyMesh::VertexHandle , SimulationData*>::type, MyMesh> simulation_data_wrapper;


  SimulationData* sim_data= sim_data_wrapper[vh1];
  SimulationData* new_sim_data= new SimulationData(*sim_data);

//  if(new_shader_param->_list.empty())
//      new_shader_param = new ShaderParameters();
  shader_parameters_data_wrapper[vh] = new_shader_param;
 // _set_of_vertices.insert(vh);
  sim_data_wrapper[vh] = new_sim_data;

  // Re-link mesh entities
  if (_m.is_boundary(_eh))
  {
    for (t_heh = heh;
        _m.next_halfedge_handle(t_heh) != opp_heh;
        t_heh = _m.opposite_halfedge_handle(_m.next_halfedge_handle(t_heh)))
    {}
  }
  else
  {
    for (t_heh = _m.next_halfedge_handle(opp_heh);
        _m.next_halfedge_handle(t_heh) != opp_heh;
        t_heh = _m.next_halfedge_handle(t_heh) )
    {}
  }

  new_heh     = _m.new_edge(vh, vh1);
  opp_new_heh = _m.opposite_halfedge_handle(new_heh);
  _m.set_vertex_handle( heh, vh );

  _m.set_next_halfedge_handle(t_heh, opp_new_heh);
  _m.set_next_halfedge_handle(new_heh, _m.next_halfedge_handle(heh));
  _m.set_next_halfedge_handle(heh, new_heh);
  _m.set_next_halfedge_handle(opp_new_heh, opp_heh);

  if (_m.face_handle(opp_heh).is_valid())
  {
    _m.set_face_handle(opp_new_heh, _m.face_handle(opp_heh));
    _m.set_halfedge_handle(_m.face_handle(opp_new_heh), opp_new_heh);
  }

  if( _m.face_handle(heh).is_valid())
  {
    _m.set_face_handle( new_heh, _m.face_handle(heh) );
    _m.set_halfedge_handle( _m.face_handle(heh), heh );
  }

  _m.set_halfedge_handle( vh, new_heh);
  _m.set_halfedge_handle( vh1, opp_new_heh );

  // Never forget this, when playing with the topology
  _m.adjust_outgoing_halfedge( vh );
  _m.adjust_outgoing_halfedge( vh1 );
}

//-----------------------------------------------------------------------------

template <typename MeshType, typename RealType>
void
SubdividerAndInterpolator<MeshType,RealType>::compute_midpoint( MeshType& _m, const EdgeHandle& _eh, const bool _update_points)
{
  HalfedgeHandle heh, opp_heh;

  heh      = _m.halfedge_handle( _eh, 0);
  opp_heh  = _m.halfedge_handle( _eh, 1);
  if(_eh.idx() == 567)
      std::cout<<"Ahoj"<<std::endl;
  Point pos( _m.point( _m.to_vertex_handle( heh)));

  pos +=  _m.point( _m.to_vertex_handle( opp_heh));

  glm::vec3 v (pos[0],pos[1],pos[2]);
  if(glm::any(glm::isnan(v))){
      std::cout<<"ERR here in midpoint"<<std::endl;

      if(_eh.idx()==636){
          std::cout<<"hola"<<std::endl;
      }

}

  // boundary edge: just average vertex positions
  // this yields the [1/2 1/2] mask
  if (_m.is_boundary(_eh) || !_update_points)
  {
    pos *= static_cast<RealType>(0.5);
  }
//  else if (_m.status(_eh).selected() )
//  {
//    pos *= 0.5; // change this
//  }

  else // inner edge: add neighbouring Vertices to sum
    // this yields the [1/16 1/16; 3/8 3/8; 1/16 1/16] mask
  {
      Point temp_pos ;
      glm::vec3 temp_v;

      //pos += _m.property(fp_pos_, _m.face_handle(heh));
      // pos += _m.property(fp_pos_, _m.face_handle(opp_heh));
      temp_pos=(_m.property(fp_pos_, _m.face_handle(heh)));
      temp_v = glm::vec3(temp_pos[0],temp_pos[1],temp_pos[2]);
      Point centroid;
      if(glm::any(glm::isnan(temp_v))  ){
          _m.calc_face_centroid( _m.face_handle(heh), centroid);
          _m.property( fp_pos_, _m.face_handle(heh)) = centroid;
          pos += centroid;
      }
      else{

          pos += _m.property(fp_pos_, _m.face_handle(heh));
      }




      temp_pos=(_m.property(fp_pos_, _m.face_handle(opp_heh)));
      temp_v = glm::vec3(temp_pos[0],temp_pos[1],temp_pos[2]);
      centroid;
      if(glm::any(glm::isnan(temp_v))  ){
          _m.calc_face_centroid( _m.face_handle(opp_heh), centroid);
          _m.property( fp_pos_, _m.face_handle(opp_heh)) = centroid;
          pos += centroid;
      }
      else{

          pos += _m.property(fp_pos_, _m.face_handle(opp_heh));
      }
    pos *= static_cast<RealType>(0.25);
  }

  _m.property( ep_pos_, _eh ) = pos;
  glm::vec3 f (pos[0],pos[1],pos[2]);
  if(glm::any(glm::isnan(f))){
      std::cout<<"ERR here in midpoint_2"<<std::endl;
}
}

//-----------------------------------------------------------------------------

template <typename MeshType, typename RealType>
void
SubdividerAndInterpolator<MeshType,RealType>::update_vertex( MeshType& _m, const VertexHandle& _vh)
{
  Point            pos(0.0,0.0,0.0);
    std::cout<<"Updating vertices"<<std::endl;
  // TODO boundary, Extraordinary Vertex and  Creased Surfaces
  // see "A Factored Approach to Subdivision Surfaces"
  // http://faculty.cs.tamu.edu/schaefer/research/tutorial.pdf
  // and http://www.cs.utah.edu/~lacewell/subdeval
  if ( _m.is_boundary( _vh))
  {
    Normal   Vec;
    pos = _m.point(_vh);
    VertexEdgeIter   ve_itr;
    for ( ve_itr = _m.ve_iter( _vh); ve_itr.is_valid(); ++ve_itr)
      if ( _m.is_boundary( *ve_itr))
        pos += _m.property( ep_pos_, *ve_itr);
    pos /= static_cast<typename OpenMesh::vector_traits<typename MeshType::Point>::value_type>(3.0);
  }
  else // inner vertex
  {
    /* For each (non boundary) vertex V, introduce a new vertex whose
       position is F/n + 2E/n + (n-3)V/n where F is the average of
       the new face vertices of all faces adjacent to the old vertex
       V, E is the average of the midpoints of all edges incident
       on the old vertex V, and n is the number of edges incident on
       the vertex.
       */

    /*
    Normal           Vec;
    VertexEdgeIter   ve_itr;
    double           valence(0.0);

    // R = Calculate Valence and sum of edge midpoints
    for ( ve_itr = _m.ve_iter( _vh); ve_itr; ++ve_itr)
    {
      valence+=1.0;
      pos += _m.property(ep_pos_, *ve_itr);
    }
    pos /= valence*valence;
    */

    RealType       valence(0.0);
    VOHIter voh_it = _m.voh_iter( _vh );
    for( ; voh_it.is_valid(); ++voh_it )
    {
      pos += _m.point( _m.to_vertex_handle( *voh_it ) );
      valence+=1.0;
    }
    pos /= valence*valence;

    VertexFaceIter vf_itr;
    Point          Q(0, 0, 0);

    for ( vf_itr = _m.vf_iter( _vh); vf_itr.is_valid(); ++vf_itr) //, neigboring_faces += 1.0 )
    {
      Q += _m.property(fp_pos_, *vf_itr);
    }

    Q /= valence*valence;//neigboring_faces;

    pos += _m.point(_vh) * (valence - RealType(2.0) )/valence + Q;
    //      pos = vector_cast<Vec>(_m.point(_vh));
  }

  _m.property( vp_pos_, _vh ) = pos;
}

//-----------------------------------------------------------------------------

//=============================================================================
//} // END_NS_UNIFORM
//} // END_NS_SUBDIVIDER
//} // END_NS_OPENMESH
//=============================================================================



//}


#endif // SUBDIVIDERANDINTERPOLATOR_IMPL_H
