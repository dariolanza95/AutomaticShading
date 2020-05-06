#ifndef MYRULES_H
#define MYRULES_H

#include <OpenMesh/Tools/Subdivider/Adaptive/Composite/RulesT.hh>
#include <OpenMesh/Core/System/config.hh>
#include <OpenMesh/Tools/Subdivider/Adaptive/Composite/RuleInterfaceT.hh>
#include "shaderparameter.h"
#include <OpenMesh/Core/Utils/vector_traits.hh>
#include <OpenMesh/Core/Utils/PropertyManager.hh>
#include  "simulationdata.h"
#include "ShaderWrapper.h"
// -------------------- STL
#include <vector>

namespace OpenMesh   { // BEGIN_NS_OPENMESH
namespace Subdivider { // BEGIN_NS_SUBDIVIDER
namespace Adaptive   { // BEGIN_NS_ADAPTIVE

#define MOBJ Base::mesh_.data
#define FH face_handle
#define VH vertex_handle
#define EH edge_handle
#define HEH halfedge_handle
#define NHEH next_halfedge_handle
#define PHEH prev_halfedge_handle
#define OHEH opposite_halfedge_handle
#define TVH  to_vertex_handle
#define FVH  from_vertex_handle
/** Composite rule VV
 */
template <class M> class myVV : public RuleInterfaceT<M>
{
  COMPOSITE_RULE( myVV, M );
private:
  typedef RuleInterfaceT<M>                 Base;

public:

  typedef RuleInterfaceT<M> Inherited;

  explicit myVV(M& _mesh) : Inherited(_mesh) {}

  void raise(typename M::VertexHandle& _vh, state_t _target_state);
//  MIPS_WARN_WA(Face) // avoid warning
//  MIPS_WARN_WA(Edge) // avoid warning
};
template<class M>
void myVV<M>::raise(typename M::VertexHandle& _vh, state_t _target_state)
{
  if (MOBJ(_vh).state() < _target_state)
  {
    this->update(_vh, _target_state);

    // raise all neighbor vertices to level x-1
    typename M::VertexVertexIter              vv_it(Base::mesh_.vv_iter(_vh));
    typename M::VertexHandle                  vh;
    std::vector<typename M::VertexHandle>     vertex_vector;

    if (_target_state > 1) {

      for (; vv_it.is_valid(); ++vv_it) {

        vertex_vector.push_back(*vv_it);
      }

      while (!vertex_vector.empty()) {

        vh = vertex_vector.back();
        vertex_vector.pop_back();

        Base::prev_rule()->raise(vh, _target_state - 1);
      }

      for (; vv_it.is_valid(); ++vv_it) {

        vertex_vector.push_back(*vv_it);
      }

      while (!vertex_vector.empty()) {

        vh = vertex_vector.back();
        vertex_vector.pop_back();

        Base::prev_rule()->raise(vh, _target_state - 1);
      }
    }

    // calculate new position
    typename M::Point  position(0.0, 0.0, 0.0);
    typename M::Scalar valence(0.0);

    auto shader_parameters_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle, ShadersWrapper*>(Base::mesh_, "shader_parameters");
    auto sim_data_wrapper = OpenMesh::getOrMakeProperty<OpenMesh::VertexHandle,SimulationData*> (Base::mesh_,"simulation_data");
    SimulationData* sw = sim_data_wrapper[_vh];

    for (vv_it = Base::mesh_.vv_iter(_vh); vv_it.is_valid(); ++vv_it) {
        SimulationData* temp = sim_data_wrapper[*vv_it];
        if(sw==nullptr && temp == nullptr){
            std::cout<<"Err"<<std::endl;
        }
        if(sw==nullptr && temp!= nullptr)
        {
            sim_data_wrapper[vh] = new SimulationData(*temp);
        }else{
            if(sw!=nullptr && temp== nullptr)
            {
                sim_data_wrapper[*vv_it] =new SimulationData(*sw);// new SimulationData(*temp);
            }
        }
      valence  += 1.0;
      position += Base::mesh_.data(*vv_it).position(_target_state - 1);

    }

    position /= valence;

    MOBJ(_vh).set_position(_target_state, position);
    MOBJ(_vh).inc_state();
    sim_data_wrapper[vh] = sw;
    // check if last rule
    if (Base::number() == Base::n_rules() - 1) {

      Base::mesh_.set_point(_vh, position);
      MOBJ(_vh).set_final();
    }
  }
}






}
}
}
#endif // MYRULES_H
