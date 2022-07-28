#include <pinocchio/fwd.hpp>

#include "diff_drive_controller.h"
#include "dynamic-graph/python/module.hh"
#include "holonomic-projection.h"
#include "odometry.h"
#include "speed_limiter.h"

namespace dg = dynamicgraph;

typedef boost::mpl::vector<dg::DiffDriveController, dg::HolonomicProjection,
                           dg::Odometry>
    entities_t;

struct register_entity {
  template <typename T>
  inline void operator()(boost::type<T>) const {
    dynamicgraph::python::exposeEntity<T>();
  }
};

BOOST_PYTHON_MODULE(wrap) {
  bp::import("dynamic_graph");
  boost::mpl::for_each<entities_t, boost::type<boost::mpl::_> >(
      register_entity());
}
