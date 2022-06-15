#include "engine/plugins/table.hpp"

#include "engine/api/table_api.hpp"
#include "engine/api/table_parameters.hpp"
#include "engine/routing_algorithms/many_to_many.hpp"
#include "engine/search_engine_data.hpp"
#include "util/coordinate_calculation.hpp"
#include "util/json_container.hpp"
#include "util/string_util.hpp"

#include <cstdlib>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include <boost/assert.hpp>

namespace osrm
{
namespace engine
{
namespace plugins
{

TablePlugin::TablePlugin(const int max_locations_distance_table)
    : max_locations_distance_table(max_locations_distance_table)
{
}

Status TablePlugin::HandleRequest(const RoutingAlgorithmsInterface &algorithms,
                                  const api::TableParameters &params,
                                  osrm::engine::api::ResultT &result) const
{
    if (!algorithms.HasManyToManySearch())
    {
        return Error("NotImplemented",
                     "Many to many search is not implemented for the chosen search algorithm.",
                     result);
    }

    BOOST_ASSERT(params.IsValid());

    if (!CheckAllCoordinates(params.coordinates))
    {
        return Error("InvalidOptions", "Coordinates are invalid", result);
    }

    if (params.bearings.size() > 0 && params.coordinates.size() != params.bearings.size())
    {
        return Error(
            "InvalidOptions", "Number of bearings does not match number of coordinates", result);
    }

    // Empty sources or destinations means the user wants all of them included, respectively
    // The ManyToMany routing algorithm we dispatch to below already handles this perfectly.
    const auto num_sources = params.sources.empty() ? params.coordinates.size() : params.sources.size();
    const auto num_destinations = params.destinations.empty() ? params.coordinates.size() : params.destinations.size();

    if (max_locations_distance_table > 0 && ((num_sources * num_destinations) > static_cast<std::size_t>(max_locations_distance_table * max_locations_distance_table)))
    {
        return Error("TooBig", "Too many table coordinates", result);
    }

    if (!CheckAlgorithms(params, algorithms, result))
        return Status::Error;

    const auto &facade = algorithms.GetFacade();

    auto phantom_nodes = GetPhantomNodes(facade, params);

    if (phantom_nodes.size() != params.coordinates.size())
    {
        return Error(
            "NoSegment", MissingPhantomErrorMessage(phantom_nodes, params.coordinates), result);
    }

    auto snapped_phantoms = SnapPhantomNodes(phantom_nodes);

    bool request_distance = params.annotations & api::TableParameters::AnnotationsType::Distance;
    bool request_duration = params.annotations & api::TableParameters::AnnotationsType::Duration;

    auto result_tables_tuple = algorithms.ManyToManySearch(
        snapped_phantoms, params.sources, params.destinations, request_distance);

    if ((request_duration && std::get<0>(result_tables_tuple).empty()) ||
        (request_distance && std::get<1>(result_tables_tuple).empty()))
    {
        return Error("NoTable", "No table found", result);
    }

    std::vector<api::TableAPI::TableCellRef> estimated_pairs;

    // Scan table for null results - if any exist, replace with distance estimates
    if (params.fallback_speed != INVALID_FALLBACK_SPEED || params.scale_factor != 1)
    {
        for (std::size_t row = 0; row < num_sources; row++)
        {
            for (std::size_t column = 0; column < num_destinations; column++)
            {
                const auto &table_index = row * num_destinations + column;
				//std::tuple<std::vector<EdgeDuration>, std::vector<EdgeDistance>, std::vector<EdgeConsumption> >
				auto edge_duration_table = std::get<0>(result_tables_tuple);
				auto edge_distance_table = std::get<1>(result_tables_tuple);
				auto edge_consumption_table = std::get<2>(result_tables_tuple);
                BOOST_ASSERT(table_index < edge_duration_table.size());
                if (params.fallback_speed != INVALID_FALLBACK_SPEED && params.fallback_speed > 0 && edge_duration_table[table_index] == MAXIMAL_EDGE_DURATION)
                {
                    const auto &source = snapped_phantoms[params.sources.empty() ? row : params.sources[row]];
                    const auto &destination = snapped_phantoms[params.destinations.empty() ? column : params.destinations[column]];

					double distance_estimate;
					if (params.fallback_coordinate_type == api::TableParameters::FallbackCoordinateType::Input) {
						distance_estimate = util::coordinate_calculation::fccApproximateDistance(source.input_location, destination.input_location);
					} else {
						distance_estimate = util::coordinate_calculation::fccApproximateDistance(source.location, destination.location);
					}


	                edge_duration_table[table_index] = distance_estimate / (double)params.fallback_speed;
                    if (!edge_distance_table.empty())
                    {
	                    edge_distance_table[table_index] = distance_estimate;
                    }

					if (!edge_consumption_table.empty()) {
						throw std::runtime_error{"Consumption is currently not supported for table requests"};
					}

                    estimated_pairs.emplace_back(row, column);
                }

                if (params.scale_factor > 0 && params.scale_factor != 1 && edge_duration_table[table_index] != MAXIMAL_EDGE_DURATION && edge_duration_table[table_index] != 0)
                {
                    EdgeDuration diff = MAXIMAL_EDGE_DURATION / edge_duration_table[table_index];

                    if (params.scale_factor >= diff)
                    {
	                    edge_duration_table[table_index] = MAXIMAL_EDGE_DURATION - 1;
                    }
                    else
                    {
	                    edge_duration_table[table_index] = std::lround(edge_duration_table[table_index] * params.scale_factor);
                    }
                }
            }
        }
    }

    api::TableAPI table_api{facade, params};
    table_api.MakeResponse(result_tables_tuple, snapped_phantoms, estimated_pairs, result);

    return Status::Ok;
}


std::tuple<std::vector<EdgeDuration>, std::vector<EdgeDistance>, std::vector<std::pair<EdgeDrivingFactor, EdgeResistanceFactor >>, std::vector<EdgeWeight> >
TablePlugin::InternalRequest(
		const RoutingAlgorithmsInterface &algorithms,
		const std::vector<PhantomNodePair> &phantom_node_pairs,
		const std::vector<std::size_t> &source_indices,
		const std::vector<std::size_t> &target_indices,
		const bool & calculate_distance) const {
	auto snapped_phantoms = SnapPhantomNodes(phantom_node_pairs);
	auto result_tables_tuple = algorithms.ManyToManySearch(snapped_phantoms, source_indices, target_indices, calculate_distance);
	return result_tables_tuple;
}


} // namespace plugins
} // namespace engine
} // namespace osrm
