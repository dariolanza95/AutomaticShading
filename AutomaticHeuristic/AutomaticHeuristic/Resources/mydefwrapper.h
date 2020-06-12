#ifndef MYDEFWRAPPER_H
#define MYDEFWRAPPER_H
#include <OpenMesh/Tools/Subdivider/Adaptive/Composite/RuleInterfaceT.hh>
#include <OpenMesh/Tools/Subdivider/Adaptive/Composite/CompositeT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
typedef OpenMesh::TriMesh_ArrayKernelT<OpenMesh::Subdivider::Adaptive::CompositeTraits>  MyMesh;

#include "simulationdata.h"
typedef std::map<MyMesh::VertexHandle,std::shared_ptr<SimulationData>> SimulationDataMap;
#endif // MYDEFWRAPPER_H
