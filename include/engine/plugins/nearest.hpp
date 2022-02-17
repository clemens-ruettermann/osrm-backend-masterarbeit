#ifndef NEAREST_HPP
#define NEAREST_HPP

#include "engine/api/nearest_parameters.hpp"
#include "engine/datafacade/contiguous_internalmem_datafacade.hpp"
#include "engine/plugins/plugin_base.hpp"
#include "engine/routing_algorithms.hpp"
#include "osrm/json_container.hpp"

namespace osrm
{
namespace engine
{
namespace plugins
{

class NearestPlugin final : public BasePlugin
{
  public:
    explicit NearestPlugin(const int max_results);

    Status HandleRequest(const RoutingAlgorithmsInterface &algorithms,
                         const api::NearestParameters &params,
                         osrm::engine::api::ResultT &result) const;

	std::vector<PhantomNodePair> GetSnappedPhantomNodes(const RoutingAlgorithmsInterface &algorithms, const std::vector<util::Coordinate> & coordinates) const;
	std::vector<PhantomNode> SnapPhantomNodePairs(const std::vector<PhantomNodePair> & phantom_node_pairs) const;
	PhantomNode SnapPhantomNodePair(const PhantomNodePair & phantom_node_pair) const;
	PhantomNodePair GetPhantomNodePair(const RoutingAlgorithmsInterface &algorithms, const util::Coordinate & coordinate) const;



  private:
    const int max_results;
};
} // namespace plugins
} // namespace engine
} // namespace osrm

#endif /* NEAREST_HPP */
