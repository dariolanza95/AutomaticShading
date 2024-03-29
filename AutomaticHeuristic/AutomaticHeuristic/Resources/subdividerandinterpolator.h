#ifndef SUBDIVIDERANDINTERPOLATOR_H
#define SUBDIVIDERANDINTERPOLATOR_H
// --------------------OpenMesh----------------------------


#include "ShaderWrapper.h"

#include <OpenMesh/Core/Utils/vector_traits.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include  "simulationdata.h"
#include "mydefwrapper.h"
#include <OpenMesh/Tools/Subdivider/Uniform/SubdividerT.hh>
#include <set>
#if defined(OM_CC_MIPS)
#  include <math.h>
#else
#  include <cmath>
#endif




//class SubdividerAndInterpolator
//{
//
//    OpenMesh::Subdivider::Uniform::SubdividerT<MyMesh>* _subdivider;
//    MyMesh _mesh;
//public:
//    SubdividerAndInterpolator( OpenMesh::Subdivider::Uniform::SubdividerT<MyMesh>* subdivider,MyMesh mesh);
//    void SubdivideAndInterpolate(int subdivision_levels);
//};



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
//  CLASS CatmullClarkT
//
//=============================================================================




//== INCLUDES =================================================================


// -------------------- STL

//== FORWARDDECLARATIONS ======================================================

//== NAMESPACES ===============================================================

//namespace OpenMesh   { // BEGIN_NS_OPENMESH
//namespace Subdivider { // BEGIN_NS_SUBVIDER
//namespace Uniform    { // BEGIN_NS_UNIFORM

//== CLASS DEFINITION =========================================================


/** \class CatmullClarkT CatmullClarkT.hh
  Based on code from Leon Kos, CAD lab, Mech.Eng., University of Ljubljana, Slovenia
  (http://www.lecad.fs.uni-lj.si/~leon)

  \note Needs a PolyMesh to work on!
*/
template <typename MeshType, typename RealType = float>
class SubdividerAndInterpolator: public OpenMesh::Subdivider::Uniform::SubdividerT< MeshType, RealType >
{


public:

  typedef typename MeshType::FaceHandle             FaceHandle;
  typedef typename MeshType::VertexHandle           VertexHandle;
  typedef typename MeshType::EdgeHandle             EdgeHandle;
  typedef typename MeshType::HalfedgeHandle         HalfedgeHandle;

  typedef typename MeshType::Point                  Point;
  typedef typename MeshType::Normal                 Normal;
  typedef typename MeshType::FaceIter               FaceIter;
  typedef typename MeshType::EdgeIter               EdgeIter;
  typedef typename MeshType::VertexIter             VertexIter;

  typedef typename MeshType::VertexEdgeIter         VertexEdgeIter;
  typedef typename MeshType::VertexFaceIter         VertexFaceIter;

  typedef typename MeshType::VOHIter                VOHIter;

  typedef OpenMesh::Subdivider::Uniform::SubdividerT< MeshType, RealType >           parent_t;

  std::set<typename MeshType::VertexHandle> _set_of_vertices;
  std::set<typename MeshType::FaceHandle>   _set_of_faces;
  SimulationDataMap& _simulation_data_map;
  std::set<typename MeshType::EdgeHandle>   _set_of_edges;

  std::map<VertexHandle,std::shared_ptr<SimulationData>> temporary_map;

  /// Constructor
  SubdividerAndInterpolator( std::set<typename MeshType::FaceHandle> set_of_faces,SimulationDataMap& simulation_data_map ) : parent_t(),_set_of_faces(set_of_faces),_simulation_data_map(simulation_data_map) {

  }

  /// Constructor
    SubdividerAndInterpolator(MeshType &_m) : parent_t(_m) {


}

  ~SubdividerAndInterpolator() {}

public:

  const char *name() const { return "Uniform CatmullClark with Data interpolation"; }

protected:

  /// Initialize properties and weights
   bool prepare( MeshType& _m );

  /// Remove properties and weights
  bool cleanup( MeshType& _m );

  /** \brief Execute n subdivision steps
     *
     * @param _m Mesh to work on
     * @param _n Number of iterations
     * @param _update_points Unused here
     * @return successful?
     */
  bool subdivide( MeshType& _m, size_t _n , const bool _update_points = false);

private:

  //===========================================================================
  /** @name Topology helpers
   * @{ */
  //===========================================================================

  void split_edge( MeshType& _m, const EdgeHandle& _eh);

  void split_face( MeshType& _m, const FaceHandle& _fh);

  void compute_midpoint( MeshType& _m, const EdgeHandle& _eh, const bool _update_points);

  void update_vertex(MeshType& _m, const  VertexHandle& _vh);

  /** @} */


private:
  OpenMesh::VPropHandleT< Point > vp_pos_; // next vertex pos
  OpenMesh::EPropHandleT< Point > ep_pos_; // new edge pts
  OpenMesh::FPropHandleT< Point > fp_pos_; // new face pts
  OpenMesh::EPropHandleT<double> creaseWeights_;// crease weights


};


//=============================================================================
//} // END_NS_UNIFORM
//} // END_NS_SUBDIVIDER
//} // END_NS_OPENMESH
//=============================================================================

#if !defined(SUBDIVIDERANDINTERPOLATOR_IMPL_H)
#  include "subdividerandinterpolator_impl.h"
#endif
#endif // SUBDIVIDERANDINTERPOLATOR_H
