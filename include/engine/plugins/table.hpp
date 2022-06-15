#ifndef TABLE_HPP
#define TABLE_HPP

#include "engine/plugins/plugin_base.hpp"

#include "engine/api/table_parameters.hpp"
#include "engine/routing_algorithms.hpp"

#include "util/json_container.hpp"

namespace osrm
{
namespace engine
{
namespace plugins
{

class TablePlugin final : public BasePlugin
{
  public:
    explicit TablePlugin(const int max_locations_distance_table);

    Status HandleRequest(const RoutingAlgorithmsInterface &algorithms,
                         const api::TableParameters &params,
                         osrm::engine::api::ResultT &result) const;

	std::tuple<std::vector<EdgeDuration>, std::vector<EdgeDistance>, std::vector<std::pair<EdgeDrivingFactor, EdgeResistanceFactor>>, std::vector<EdgeWeight> >
	InternalRequest(const RoutingAlgorithmsInterface &algorithms, const std::vector<PhantomNodePair> &phantom_node_pairs,
	                const std::vector<std::size_t> &source_indices,
	                const std::vector<std::size_t> &target_indices,
	                const bool & calculate_distance) const;
//
//	std::tuple<std::vector<EdgeDuration>, std::vector<EdgeDistance>, std::vector<EdgeConsumption> >
//	InternalRequest(const RoutingAlgorithmsInterface &algorithms, PhantomNode & source_node, std::vector<PhantomNode> & destination_nodes) const;
//
//	std::tuple<std::vector<EdgeDuration>, std::vector<EdgeDistance>, std::vector<EdgeConsumption> >
//	InternalRequest(const RoutingAlgorithmsInterface &algorithms, std::vector<PhantomNode> & source_nodes, std::vector<PhantomNode> & destination_nodes) const;

  private:
    const int max_locations_distance_table;
};
} // namespace plugins
} // namespace engine
} // namespace osrm

#endif // TABLE_HPP
