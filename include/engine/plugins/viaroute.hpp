#ifndef VIA_ROUTE_HPP
#define VIA_ROUTE_HPP

#include "engine/plugins/plugin_base.hpp"

#include "engine/api/route_parameters.hpp"
#include "engine/routing_algorithms.hpp"

#include "util/json_container.hpp"
#include "engine/guidance/leg_geometry.hpp"

#include <cstdlib>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace osrm
{
namespace engine
{
namespace plugins
{

class ViaRoutePlugin final : public BasePlugin
{
  private:
    const int max_locations_viaroute;
    const int max_alternatives;

  public:
    explicit ViaRoutePlugin(int max_locations_viaroute, int max_alternatives);

    Status HandleRequest(const RoutingAlgorithmsInterface &algorithms,
                         const api::RouteParameters &route_parameters,
                         osrm::engine::api::ResultT &json_result) const;

	Status InternalRequest(const RoutingAlgorithmsInterface &algorithms,
						   const std::vector<util::Coordinate> &coords,
						   std::vector<guidance::LegGeometry> & result) const;

	Status InternalRequest(const RoutingAlgorithmsInterface &algorithms,
	                       const std::vector<PhantomNodePair> &phantom_nodes,
	                       std::vector<guidance::LegGeometry> & result,
						   RouteConsumption & last_route_consumption) const;

	Status InternalRequest(const RoutingAlgorithmsInterface &algorithms,
	                       const std::vector<util::Coordinate> &coords,
	                       util::json::Object &json_result) const;

	Status InternalRequest(const RoutingAlgorithmsInterface &algorithms,
	                       const std::vector<PhantomNodePair> &phantom_nodes,
	                       util::json::Object &json_result) const;

};
} // namespace plugins
} // namespace engine
} // namespace osrm

#endif // VIA_ROUTE_HPP
