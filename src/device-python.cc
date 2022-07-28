#include "dynamic-graph/python/module.hh"
#include "sot-tiago-device.hh"

namespace dg = dynamicgraph;

BOOST_PYTHON_MODULE(wrap) {
  bp::import("dynamic_graph.sot.core.wrap");

  dynamicgraph::python::exposeEntity<SoTTiagoDevice,
                                     bp::bases<dg::sot::Device> >();
}
