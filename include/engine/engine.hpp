#ifndef ENGINE_HPP
#define ENGINE_HPP

#include "engine/api/match_parameters.hpp"
#include "engine/api/nearest_parameters.hpp"
#include "engine/api/route_parameters.hpp"
#include "engine/api/table_parameters.hpp"
#include "engine/api/tile_parameters.hpp"
#include "engine/api/trip_parameters.hpp"
#include "engine/datafacade_provider.hpp"
#include "engine/engine_config.hpp"
#include "engine/plugins/match.hpp"
#include "engine/plugins/nearest.hpp"
#include "engine/plugins/table.hpp"
#include "engine/plugins/tile.hpp"
#include "engine/plugins/trip.hpp"
#include "engine/plugins/viaroute.hpp"
#include "engine/routing_algorithms.hpp"
#include "engine/status.hpp"

#include "util/json_container.hpp"

#include <memory>
#include <string>

namespace osrm
{
namespace engine
{

class EngineInterface
{
  public:
    virtual ~EngineInterface() = default;
    virtual Status Route(const api::RouteParameters &parameters, api::ResultT &result) const = 0;
    virtual Status Table(const api::TableParameters &parameters, api::ResultT &result) const = 0;
    virtual Status Nearest(const api::NearestParameters &parameters,
                           api::ResultT &result) const = 0;
    virtual Status Trip(const api::TripParameters &parameters, api::ResultT &result) const = 0;
    virtual Status Match(const api::MatchParameters &parameters, api::ResultT &result) const = 0;
    virtual Status Tile(const api::TileParameters &parameters, api::ResultT &result) const = 0;

	virtual std::tuple<std::vector<EdgeDuration>, std::vector<EdgeDistance>, std::vector<EdgeConsumption>, std::vector<EdgeWeight> >
	ManyToManyInternal(const std::vector<PhantomNodePair> &phantom_node_pairs,
	                   const std::vector<std::size_t> &source_indices,
	                   const std::vector<std::size_t> &target_indices,
	                   const bool & calculate_distance) const = 0;

	virtual Status
	ViaRouteInternal(const std::vector<PhantomNodePair> &phantom_node_pairs,
	                 std::vector<guidance::LegGeometry> & result) const = 0;

	virtual Status ViaRouteInternal(const std::vector<util::Coordinate> &coords, util::json::Object &result) const = 0;
	virtual Status ViaRouteInternal(const std::vector<PhantomNodePair> &phantom_nodes, util::json::Object &result) const = 0;
	virtual Status ViaRouteInternal(const std::vector<util::Coordinate> &coords, std::vector<guidance::LegGeometry> & result) const = 0;

	virtual Status
	ViaRouteInternal(const std::vector<PhantomNodePair> &phantom_nodes, std::vector<guidance::LegGeometry> & result, RouteConsumption & last_route_consumption) const = 0;

	virtual PhantomNodePair GetPhantomNodePair(const util::Coordinate & coordinate) const = 0;
	virtual std::vector<PhantomNodePair> GetPhantomNodePairs(const std::vector<util::Coordinate> & coordinates) const = 0;

	virtual std::vector<PhantomNode> SnapPhantomNodes(const std::vector<PhantomNodePair> & phantom_node_pairs) const = 0;
	virtual PhantomNode SnapPhantomNode(const PhantomNodePair & phantom_node_pair) const = 0;
};

template <typename Algorithm> class Engine final : public EngineInterface
{
  public:
    explicit Engine(const EngineConfig &config)
        : route_plugin(config.max_locations_viaroute, config.max_alternatives),            //
          table_plugin(config.max_locations_distance_table),                               //
          nearest_plugin(config.max_results_nearest),                                      //
          trip_plugin(config.max_locations_trip),                                          //
          match_plugin(config.max_locations_map_matching, config.max_radius_map_matching), //
          tile_plugin()                                                                    //

    {
        if (config.use_shared_memory)
        {
            util::Log(logDEBUG) << "Using shared memory with name \"" << config.dataset_name
                                << "\" with algorithm " << routing_algorithms::name<Algorithm>();
            facade_provider = std::make_unique<WatchingProvider<Algorithm>>(config.dataset_name);
        }
        else if (!config.memory_file.empty() || config.use_mmap)
        {
            if (!config.memory_file.empty())
            {
                util::Log(logWARNING)
                    << "The 'memory_file' option is DEPRECATED - using direct mmaping instead";
            }
            util::Log(logDEBUG) << "Using direct memory mapping with algorithm "
                                << routing_algorithms::name<Algorithm>();
            facade_provider = std::make_unique<ExternalProvider<Algorithm>>(config.storage_config);
        }
        else
        {
            util::Log(logDEBUG) << "Using internal memory with algorithm "
                                << routing_algorithms::name<Algorithm>();
            facade_provider = std::make_unique<ImmutableProvider<Algorithm>>(config.storage_config);
        }
    }

    Engine(Engine &&) noexcept = delete;
    Engine &operator=(Engine &&) noexcept = delete;

    Engine(const Engine &) = delete;
    Engine &operator=(const Engine &) = delete;
    virtual ~Engine() = default;

    Status Route(const api::RouteParameters &params, api::ResultT &result) const override final
    {
        return route_plugin.HandleRequest(GetAlgorithms(params), params, result);
    }

    Status Table(const api::TableParameters &params, api::ResultT &result) const override final
    {
        return table_plugin.HandleRequest(GetAlgorithms(params), params, result);
    }

    Status Nearest(const api::NearestParameters &params, api::ResultT &result) const override final
    {
        return nearest_plugin.HandleRequest(GetAlgorithms(params), params, result);
    }

    Status Trip(const api::TripParameters &params, api::ResultT &result) const override final
    {
        return trip_plugin.HandleRequest(GetAlgorithms(params), params, result);
    }

    Status Match(const api::MatchParameters &params, api::ResultT &result) const override final
    {
        return match_plugin.HandleRequest(GetAlgorithms(params), params, result);
    }

    Status Tile(const api::TileParameters &params, api::ResultT &result) const override final
    {
        return tile_plugin.HandleRequest(GetAlgorithms(params), params, result);
    }

	std::vector<PhantomNode> GetSnappedPhantomNodes(const std::vector<util::Coordinate> & coordinates) const {
		api::BaseParameters params;
		params.exclude.template emplace_back("ferry");
		return nearest_plugin.GetSnappedPhantomNodes(GetAlgorithms(params), coordinates);
	}

	std::vector<PhantomNode> SnapPhantomNodes(const std::vector<PhantomNodePair> & phantom_node_pairs) const override {
		return nearest_plugin.SnapPhantomNodePairs(phantom_node_pairs);
	}

	PhantomNode SnapPhantomNode(const PhantomNodePair & phantom_node_pair) const override {
		return nearest_plugin.SnapPhantomNodePair(phantom_node_pair);
	}

	PhantomNodePair GetPhantomNodePair(const util::Coordinate & coordinate) const override {
		api::BaseParameters params;
		params.exclude.emplace_back("ferry");
		return nearest_plugin.GetPhantomNodePair(GetAlgorithms(params), coordinate);
	}

	virtual std::vector<PhantomNodePair> GetPhantomNodePairs(const std::vector<util::Coordinate> & coordinates) const override {
		api::BaseParameters params;
		params.exclude.emplace_back("ferry");
		return nearest_plugin.GetSnappedPhantomNodes(GetAlgorithms(params), coordinates);
	}

	std::tuple<std::vector<EdgeDuration>, std::vector<EdgeDistance>, std::vector<EdgeConsumption>, std::vector<EdgeWeight> >
	ManyToManyInternal(const std::vector<PhantomNodePair> &phantom_node_pairs,
	                   const std::vector<std::size_t> &source_indices,
	                   const std::vector<std::size_t> &target_indices,
	                   const bool & calculate_distance) const override {
		api::BaseParameters params;
		params.exclude.emplace_back("ferry");
		return table_plugin.InternalRequest(GetAlgorithms(params), phantom_node_pairs, source_indices, target_indices, calculate_distance);
	}

	Status
	ViaRouteInternal(const std::vector<PhantomNodePair> &phantom_node_pairs,
	                 std::vector<guidance::LegGeometry> & result) const override {
		api::BaseParameters params;
		params.exclude.emplace_back("ferry");
		RouteConsumption dummy;
		return route_plugin.InternalRequest(GetAlgorithms(params), phantom_node_pairs, result, dummy);
	}

	Status
	ViaRouteInternal(const std::vector<util::Coordinate> &coords, util::json::Object &result) const override {
		api::BaseParameters params;
		params.exclude.emplace_back("ferry");
		auto algo = GetAlgorithms(params);
		auto status = route_plugin.InternalRequest(algo, coords, result);
		return status;
	}

	Status
	ViaRouteInternal(const std::vector<PhantomNodePair> &phantom_nodes, util::json::Object &result) const override {
		api::BaseParameters params;
		params.exclude.emplace_back("ferry");
		auto algo = GetAlgorithms(params);
		auto status = route_plugin.InternalRequest(algo, phantom_nodes, result);
		return status;
	}

	Status
	ViaRouteInternal(const std::vector<util::Coordinate> &coords, std::vector<guidance::LegGeometry> & result) const override {
		api::BaseParameters params;
		params.exclude.emplace_back("ferry");
		auto algo = GetAlgorithms(params);
		auto status = route_plugin.InternalRequest(algo, coords, result);
		return status;
	}

	virtual Status
	ViaRouteInternal(const std::vector<PhantomNodePair> &phantom_node_pairs, std::vector<guidance::LegGeometry> & result, RouteConsumption & last_route_consumption) const override {
		api::BaseParameters params;
		params.exclude.emplace_back("ferry");
		auto status = route_plugin.InternalRequest(GetAlgorithms(params), phantom_node_pairs, result, last_route_consumption);
		return status;
	}

	Status
	ViaRouteInternalConsumptions(const std::vector<util::Coordinate> &coords, std::vector<RouteConsumption> & consumptions, std::vector<EdgeDistance> & distances) const {
		api::BaseParameters params;
		params.exclude.emplace_back("ferry");
		std::vector<std::vector<guidance::LegGeometry>> result;
		auto status = route_plugin.InternalRequest(GetAlgorithms(params), coords, result);
		if (status == Status::Error) {
			return status;
		}

		RouteConsumption route_consumption;
		EdgeDistance route_distance;
		for (const auto & route : result) {
			route_consumption = 0;
			route_distance = 0;
			for (const auto & leg : route) {
				for (const auto & annotation : leg.annotations) {
					route_consumption += annotation.consumption;
					route_distance += annotation.distance;
				}
			}
			consumptions.push_back(route_consumption);
			distances.push_back(route_distance);
		}
		return status;
	}

  private:
    template <typename ParametersT> auto GetAlgorithms(const ParametersT &params) const
    {
        return RoutingAlgorithms<Algorithm>{heaps, facade_provider->Get(params)};
    }
    std::unique_ptr<DataFacadeProvider<Algorithm>> facade_provider;
	mutable SearchEngineData<Algorithm> heaps;

    const plugins::ViaRoutePlugin route_plugin;
    const plugins::TablePlugin table_plugin;
    const plugins::NearestPlugin nearest_plugin;
    const plugins::TripPlugin trip_plugin;
    const plugins::MatchPlugin match_plugin;
    const plugins::TilePlugin tile_plugin;
};
} // namespace engine
} // namespace osrm

#endif // OSRM_IMPL_HPP
