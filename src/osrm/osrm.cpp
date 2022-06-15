#include "osrm/osrm.hpp"

#include "engine/algorithm.hpp"
#include "engine/api/match_parameters.hpp"
#include "engine/api/nearest_parameters.hpp"
#include "engine/api/route_parameters.hpp"
#include "engine/api/table_parameters.hpp"
#include "engine/api/trip_parameters.hpp"
#include "engine/engine.hpp"
#include "engine/engine_config.hpp"
#include "engine/status.hpp"
#include "charger_graph/charger_graph.hpp"
#include "charger_graph/files.hpp"
#include "util/timing_util.hpp"
#include <execution>
#include <memory>


#define DIJKSTRA_PARALELL

namespace osrm {

// Pimpl idiom

OSRM::OSRM(engine::EngineConfig &config) {
	using CH = engine::routing_algorithms::ch::Algorithm;
	using MLD = engine::routing_algorithms::mld::Algorithm;

	// First, check that necessary core data is available
	if (!config.use_shared_memory && !config.storage_config.IsValid()) {
		throw util::exception("Required files are missing, cannot continue.  Have all the "
		                      "pre-processing steps been run?");
	}

	// Now, check that the algorithm requested can be used with the data
	// that's available.

	if (config.algorithm == EngineConfig::Algorithm::CoreCH) {
		util::Log(logWARNING) << "Using CoreCH is deprecated. Falling back to CH";
		config.algorithm = EngineConfig::Algorithm::CH;
	}

	switch (config.algorithm) {
		case EngineConfig::Algorithm::CH:
			engine_ = std::make_unique<engine::Engine<CH>>(config);
			break;
		case EngineConfig::Algorithm::MLD:
			engine_ = std::make_unique<engine::Engine<MLD>>(config);
			break;
		default:
			util::exception("Algorithm not implemented!");
	}

	std::shared_ptr<std::vector<enav::ChargerGraphEdge>> edges = std::make_shared<std::vector<enav::ChargerGraphEdge>>();
	enav::files::readEdges(config.storage_config.GetPath(".osrm.charger_graph"), edges);
	std::vector<enav::Charger> chargers;
	enav::files::readChargers(config.storage_config.GetPath(".osrm.charger_graph.charger.json"), chargers);
	auto car = std::make_shared<enav::Car>(config.storage_config.GetPath(".osrm.car.properties").string());
	charger_graph_ = std::make_unique<enav::ChargerGraph>(car, edges, chargers);
}

OSRM::~OSRM() = default;

OSRM::OSRM(OSRM &&) noexcept = default;

OSRM &OSRM::operator=(OSRM &&) noexcept = default;

// Forward to implementation

Status OSRM::Route(const engine::api::RouteParameters &params, json::Object &json_result) const {
	osrm::engine::api::ResultT result = json::Object();
	auto status = engine_->Route(params, result);
	json_result = std::move(result.get<json::Object>());
	return status;
}

Status OSRM::Route(const RouteParameters &params, engine::api::ResultT &result) const {
	return engine_->Route(params, result);
}

std::vector<MyLegGeometry> OSRM::RouteInternal(const std::vector<util::Coordinate> & coords, const double wltp, const double car_weight) const {
	std::vector<engine::guidance::LegGeometry> route_legs;
	auto phantom_node_pairs = engine_->GetPhantomNodePairs(coords);
	auto status = engine_->ViaRouteInternal(phantom_node_pairs, route_legs);
	if (status == engine::Status::Ok) {
		std::vector<MyLegGeometry> ret;
		for (auto &leg: route_legs) {
			MyLegGeometry new_leg;

			std::vector<MyLegGeometry::Annotation> annotations;
			for (auto & an : leg.annotations) {
				annotations.emplace_back(MyLegGeometry::Annotation{
					an.distance,
					an.duration,
					an.weight,
					(std::int32_t )std::lround(an.consumption_factor_pair.first * wltp + an.consumption_factor_pair.second*car_weight),
					an.datasource});
			}

			new_leg.annotations = annotations;
			new_leg.osm_node_ids = leg.osm_node_ids;
			new_leg.locations = leg.locations;
			new_leg.segment_distances = leg.segment_distances;
			new_leg.segment_offsets = leg.segment_offsets;
			ret.emplace_back(new_leg);
		}
		return ret;
	} else {
		throw std::runtime_error{"Could not calculate route"};
	}
}


Status OSRM::Table(const engine::api::TableParameters &params, json::Object &json_result) const {
	osrm::engine::api::ResultT result = json::Object();
	auto status = engine_->Table(params, result);
	json_result = std::move(result.get<json::Object>());
	return status;
}

Status OSRM::Table(const TableParameters &params, engine::api::ResultT &result) const {
	return engine_->Table(params, result);
}

Status OSRM::Nearest(const engine::api::NearestParameters &params, json::Object &json_result) const {
	osrm::engine::api::ResultT result = json::Object();
	auto status = engine_->Nearest(params, result);
	json_result = std::move(result.get<json::Object>());
	return status;
}

Status OSRM::Nearest(const NearestParameters &params, engine::api::ResultT &result) const {
	return engine_->Nearest(params, result);
}

Status OSRM::Trip(const engine::api::TripParameters &params, json::Object &json_result) const {
	osrm::engine::api::ResultT result = json::Object();
	auto status = engine_->Trip(params, result);
	json_result = std::move(result.get<json::Object>());
	return status;
}

engine::Status OSRM::Trip(const engine::api::TripParameters &params,
                          engine::api::ResultT &result) const {
	return engine_->Trip(params, result);
}

Status OSRM::Match(const engine::api::MatchParameters &params, json::Object &json_result) const {
	osrm::engine::api::ResultT result = json::Object();
	auto status = engine_->Match(params, result);
	json_result = std::move(result.get<json::Object>());
	return status;
}

Status OSRM::Match(const MatchParameters &params, engine::api::ResultT &result) const {
	return engine_->Match(params, result);
}

Status OSRM::Tile(const engine::api::TileParameters &params, std::string &str_result) const {
	osrm::engine::api::ResultT result = std::string();
	auto status = engine_->Tile(params, result);
	str_result = std::move(result.get<std::string>());
	return status;
}

Status OSRM::Tile(const engine::api::TileParameters &params, engine::api::ResultT &result) const {
	return engine_->Tile(params, result);
}

using RouteConsumption = std::int64_t;




// utils for charger routing


static std::string routesToGeojson(
	const std::vector<osrm::engine::guidance::LegGeometry> &routes,
	const double battery_capacity_in_milli_wh,
	const double wltp,
	const double car_weight
) {

	RouteConsumption complete_consumption = 0;
	std::stringstream geojson_string_stream;
	geojson_string_stream << R"({)";
	geojson_string_stream << R"(
	"type": "FeatureCollection",
	"features": [)";
	for (size_t k = 0; k < routes.size(); k++) {
		const auto &route = routes[k];


		RouteConsumption battery_cap = battery_capacity_in_milli_wh;
		double distance_of_subroute = 0;
		BOOST_ASSERT(route.locations.size() == route.annotations.size() + 1);
		for (size_t j = 0; j < route.annotations.size(); j++) {
			distance_of_subroute += route.annotations[j].distance;


			geojson_string_stream << R"(
	{
		"type": "Feature",
		"geometry": {
			"type": "LineString",
			"coordinates": [)";
			auto begin = route.locations[j];
			geojson_string_stream << "[" << begin.ToString() << "], ";
			auto end = route.locations[j + 1];
			geojson_string_stream << "[" << end.ToString() << "]";
			geojson_string_stream << R"(]
		},)";
			geojson_string_stream << R"(
		"properties": {
			"consumption": )";
			auto tmp_consumption = route.annotations[j].consumption_factor_pair.first * wltp + route.annotations[j].consumption_factor_pair.second * car_weight;
			battery_cap -= tmp_consumption;
			complete_consumption += tmp_consumption;
			auto percentage = ((double) battery_cap / battery_capacity_in_milli_wh) * 100.0;
			geojson_string_stream << std::to_string(percentage);
			geojson_string_stream << R"(
		}
	})";

			if (k == routes.size() - 1 && j == route.annotations.size() - 1) {
				continue;
			}
			geojson_string_stream << ",";
		}
	}
	geojson_string_stream << "]" << std::endl;
	geojson_string_stream << R"(})";
	return geojson_string_stream.str();
}


static void route_result_to_custom_json(
		util::json::Object &result,
		std::uint32_t base_battery_capacity,
		std::vector<engine::guidance::LegGeometry> &route_legs,
		std::vector<std::uint32_t> &max_power_of_used_chargers,
		std::vector<util::Coordinate> &points,
		const double wltp,
		const double car_weight) {

	auto waypoints = util::json::Array();

	EdgeDuration duration = 0;
	EdgeDuration charging_time = 0;
	EdgeWeight weight = 0;
	RouteConsumption consumption = 0;

	util::FixedLatitude min_lat = route_legs[0].locations[0].lat;
	util::FixedLatitude max_lat = route_legs[0].locations[0].lat;
	util::FixedLongitude min_lon = route_legs[0].locations[0].lon;
	util::FixedLongitude max_lon = route_legs[0].locations[0].lon;
	for (size_t i = 0; i < route_legs.size(); i++) {
		const auto &it = route_legs[i];
		RouteConsumption battery_capacity = base_battery_capacity;
		waypoints.values.emplace_back();

		for (size_t j = 0; j < it.annotations.size(); j++) {
			auto tmp_lon = it.locations[j].lon;
			auto tmp_lat = it.locations[j].lat;
			if (min_lon > tmp_lon) {
				min_lon = tmp_lon;
			}
			if (min_lat > tmp_lat) {
				min_lat = tmp_lat;
			}
			if (max_lon < tmp_lon) {
				max_lon = tmp_lon;
			}
			if (max_lat < tmp_lat) {
				max_lat = tmp_lat;
			}

			duration += it.annotations[j].duration;
			auto current_consumption = it.annotations[j].consumption_factor_pair.first * wltp + it.annotations[j].consumption_factor_pair.second * car_weight;
			battery_capacity -= current_consumption;
			weight += it.annotations[j].weight;
			consumption += current_consumption;


			auto new_waypoint = util::json::Object();
			new_waypoint.values["lon"] = (double) toFloating(tmp_lon);
			new_waypoint.values["lat"] = (double) toFloating(tmp_lat);
			new_waypoint.values["charge_level"] = (double) battery_capacity / base_battery_capacity * 100.0;

			waypoints.values.emplace_back(new_waypoint);
		}

		if (i < route_legs.size() - 1) {
			charging_time += enav::calculate_charging_time(base_battery_capacity - battery_capacity,
			                                               max_power_of_used_chargers[i]);
		}
	}

	result.values["waypoints"] = waypoints;
	result.values["chargers"] = util::json::Array();

	auto bbox = util::json::Array();
	bbox.values.emplace_back((double) toFloating(min_lon));
	bbox.values.emplace_back((double) toFloating(min_lat));
	bbox.values.emplace_back(0);
	bbox.values.emplace_back((double) toFloating(max_lon));
	bbox.values.emplace_back((double) toFloating(max_lat));
	result.values["bbox"] = bbox;
	result.values["charging_time"] = charging_time / 60.0;
	result.values["driving_time"] = duration / 60.0;
	result.values["weight"] = util::json::Number(weight + charging_time);
	result.values["consumption"] = util::json::Number(consumption);

	auto stop_points = util::json::Array();
	for (const auto &it: points) {
		auto new_point = util::json::Object();
		new_point.values["lon"] = (double) toFloating(it.lon);
		new_point.values["lat"] = (double) toFloating(it.lat);

		stop_points.values.emplace_back(new_point);
	}
	result.values["points"] = stop_points;
	result.values["geojson"] = routesToGeojson(route_legs, base_battery_capacity, wltp, car_weight);
}


enav::ReachableChargers OSRM::getReachableChargers(
		const engine::PhantomNodePair &phantom_node_start,
		const engine::PhantomNodePair &phantom_node_end,
		const std::vector<enav::Charger> &chargers,
		const double lower_capacity_limit,
		const double upper_capacity_limit,
		const double wltp,
		const double car_weight) const {
	std::vector<enav::ReachableStartNode> from_start_reachable_chargers;
	std::vector<enav::ReachableEndNode> to_end_reachable_chargers;
	std::vector<engine::PhantomNodePair> all_nodes;

	for (const auto &it: chargers) {
		all_nodes.push_back(it.phantom_node_pair);
	}
	{
		all_nodes.push_back(phantom_node_start);
		TIMER_START(TMP_START);

		// The start node was added as last node and we want to calculate all routes from start to all chargers
		auto result = engine_->ManyToManyInternal(all_nodes, {all_nodes.size() - 1}, {}, false);
		TIMER_STOP(TMP_START);
		util::Log(logDEBUG) << "Calculating from start reachable chargers took " << TIMER_SEC(TMP_START) << "s";

		auto consumption_factors = std::get<2>(result);
		auto weights = std::get<3>(result);
		for (size_t i = 0; i < all_nodes.size() - 1; i++) {
			auto tmp_consumption = consumption_factors[i].first * wltp + consumption_factors[i].second * car_weight;
			if (upper_capacity_limit >= tmp_consumption && lower_capacity_limit <= tmp_consumption) {
				from_start_reachable_chargers.emplace_back(chargers[i], consumption_factors[i].first, consumption_factors[i].second, weights[i], wltp, car_weight);
			}
		}
		if (from_start_reachable_chargers.empty()) {
			return {{},
			        {}};
		}
		util::Log(logDEBUG) << "There are " << std::to_string(from_start_reachable_chargers.size())
		                    << " from start reachable chargers";
		auto it = std::find(all_nodes.begin(), all_nodes.end(), phantom_node_start);
		all_nodes.erase(it);
	}
	{
		all_nodes.push_back(phantom_node_end);
		TIMER_START(TMP_END);
		auto result = engine_->ManyToManyInternal(all_nodes, {}, {all_nodes.size() - 1}, false);
		TIMER_STOP(TMP_END);
		util::Log(logDEBUG) << "Calculating to end reachable chargers took " << TIMER_SEC(TMP_END) << "s";

		auto consumption_factors = std::get<2>(result);
		auto weights = std::get<3>(result);
		for (size_t i = 0; i < all_nodes.size() - 1; i++) {
			auto tmp_consumption = consumption_factors[i].first * wltp + consumption_factors[i].second * car_weight;
			if (upper_capacity_limit >= tmp_consumption && lower_capacity_limit <= tmp_consumption) {
				to_end_reachable_chargers.emplace_back(chargers[i].node_id, consumption_factors[i].first, consumption_factors[i].second, weights[i]);
			}
		}
		if (to_end_reachable_chargers.empty()) {
			return {{},
			        {}};
		}
		util::Log(logDEBUG) << "There are " << std::to_string(to_end_reachable_chargers.size())
		                    << " end reachable chargers";
	}
	return {std::move(from_start_reachable_chargers), std::move(to_end_reachable_chargers)};
}



static RouteConsumption consumptionOfLeg(const engine::guidance::LegGeometry &leg, const double wltp, const double car_weight) {
	RouteConsumption ret = 0;
	for (const auto &annotation: leg.annotations) {
		ret += annotation.consumption_factor_pair.first * wltp + annotation.consumption_factor_pair.second * car_weight;
	}
	return ret;
}


static void pointsToCSV(const std::vector<util::Coordinate> &coords, const std::string &path) {
	std::ofstream out{path};
	out << "lon,lat" << std::endl;
	for (const auto &it: coords) {
		out << it.ToString() << std::endl;
	}
}


static std::vector<util::Coordinate> getSearchPointsForLegAndLimit(
		const engine::guidance::LegGeometry &leg_geometry,
		const std::uint32_t lower_capacity_limit,
		const std::uint32_t upper_capacity_limit,
		const double search_radius,
		const double wltp,
		const double car_weight) {

	std::vector<util::Coordinate> search_points;
	RouteConsumption tmp_consumption = 0;
	for (size_t i = 0; i < leg_geometry.annotations.size(); i++) {
		tmp_consumption += leg_geometry.annotations[i].consumption_factor_pair.first * wltp + leg_geometry.annotations[i].consumption_factor_pair.second * car_weight;
		if (tmp_consumption < lower_capacity_limit) {
			continue;
		}
		if (tmp_consumption > upper_capacity_limit) {
			break;
		}

		if (!std::any_of(search_points.cbegin(), search_points.cend(), [&](const auto &existing_search_point) {
			return util::coordinate_calculation::haversineDistance(existing_search_point, leg_geometry.locations[i]) <
			       search_radius;
		})) {
			search_points.emplace_back(leg_geometry.locations[i]);
		}

	}
	return search_points;
}


/*
 * Returns a list with chargers along the given leg_geometry found at search points where the consumption is between
 * lower_capacity_limit and upper_capacity_limit within a radius of search_radius
 */
static std::vector<enav::Charger>
getPossibleChargers(engine::guidance::LegGeometry &leg_geometry, std::vector<enav::Charger> &charger_list,
                    const uint32_t lower_capacity_limit, const uint32_t upper_capacity_limit,
                    const double search_radius, std::vector<ChargerId> used_charger_ids, const double wltp, const double car_weight) {
	std::vector<enav::Charger> possible_chargers;
	auto search_points = getSearchPointsForLegAndLimit(leg_geometry, lower_capacity_limit, upper_capacity_limit, search_radius, wltp, car_weight);
	util::Log(logDEBUG) << "Found " << search_points.size() << " points to search for chargers";

	auto charger_within_searchradius_of_points = [&search_points, &search_radius, &used_charger_ids](
			const enav::Charger &charger) {
		if (std::any_of(used_charger_ids.cbegin(), used_charger_ids.cend(),
		                [&charger](const auto &used_id) { return charger.node_id == used_id; })) {
			return false;
		}
		return std::any_of(
				search_points.begin(),
				search_points.end(),
				[&charger, &search_radius](const util::Coordinate &coord) {
					return util::coordinate_calculation::haversineDistance(charger.coordinate, coord) < search_radius;
				}
		);
	};

	std::copy_if(std::execution::par_unseq,
	             charger_list.begin(),
	             charger_list.end(),
	             std::back_inserter(possible_chargers),
	             charger_within_searchradius_of_points
	);
	util::Log(logDEBUG) << "Found " << possible_chargers.size() << " possible chargers";

	std::vector<std::pair<double, enav::Charger>> possible_chargers_with_distance_to_route;
	for (auto &charger: possible_chargers) {
		double min_dist = search_radius;
		for (auto &loc: leg_geometry.locations) {
			auto dist = util::coordinate_calculation::haversineDistance(charger.coordinate, loc);
			if (dist < min_dist) {
				min_dist = dist;
			}
		}
		possible_chargers_with_distance_to_route.emplace_back(std::make_pair(min_dist, charger));
	}
	std::sort(
			std::execution::par_unseq,
			possible_chargers_with_distance_to_route.begin(),
			possible_chargers_with_distance_to_route.end(),
			[](const auto &p1, const auto &p2) {
				return p1.first < p2.first;
			});


	std::vector<enav::Charger> result;
	std::transform(possible_chargers_with_distance_to_route.begin(), possible_chargers_with_distance_to_route.end(),
	               std::back_inserter(result), [](const auto &p) { return p.second; });
	return result;
}


Status OSRM::EVRouteDijkstraAlongRoute(EVRouteParameters &parameters, json::Object &result) const {
	auto battery_capacity = osrm::enav::temperature_dependent_capacity(charger_graph_->car->base_battery_capacity_milli_watt_h, parameters.temperature);

	auto lower_capacity_limit = parameters.lower_capacity_limit;
	auto upper_capacity_limit = parameters.upper_capacity_limit;

	auto phantom_node_pair_start = engine_->GetPhantomNodePair(parameters.start);
	auto phantom_node_pair_end = engine_->GetPhantomNodePair(parameters.end);

	std::vector<engine::guidance::LegGeometry> first_route_result;
	auto status = engine_->ViaRouteInternal(
			std::vector<engine::PhantomNodePair>{phantom_node_pair_start, phantom_node_pair_end}, first_route_result);
	if (status == Status::Error) {
		util::Log(logERROR) << "[DijkstraAlongRoute]" << "Could not calculate route";
		return Status::Error;
	}

	if (first_route_result.size() == 1 && consumptionOfLeg(first_route_result[0], parameters.wltp, parameters.weight) <= upper_capacity_limit) {
		return pointsToFinalRoute(
			parameters.output_format,
			phantom_node_pair_start,
			phantom_node_pair_end,
			{},
			battery_capacity,
			parameters.wltp, parameters.weight,
			result);
	}

	TIMER_START(FILTER_CHARGERS);
	std::vector<enav::Charger> possible_chargers = getPossibleChargers(first_route_result[0],
	                                                                   charger_graph_->charger_list,
	                                                                   0,
	                                                                   std::numeric_limits<std::int32_t>::max(),
	                                                                   parameters.search_radius,
	                                                                   {},
																	   parameters.wltp,
																	   parameters.weight);


//	{
//		std::ofstream out{"/tmp/test2/possible_points_dijkstra_along.csv"};
//		out << "lon,lat" << std::endl;
//		for (const auto & it : possible_chargers) {
//			out << it.coordinate.ToString() << std::endl;
//		}
//	}


	TIMER_STOP(FILTER_CHARGERS);
	util::Log(logDEBUG) << "[DijkstraAlongRoute] " << "Filtering the chargers took " << TIMER_SEC(FILTER_CHARGERS) << "s";
	util::Log(logDEBUG) << "[DijkstraAlongRoute] " << "Possible chargers: " << possible_chargers.size();

	std::vector<ChargerId> possible_charger_ids;
	transform(possible_chargers.cbegin(), possible_chargers.cend(), std::back_inserter(possible_charger_ids),
	          [](const enav::Charger &c) { return c.node_id; });

	std::shared_ptr<std::vector<enav::ChargerGraphEdge>> edges = std::make_shared<std::vector<enav::ChargerGraphEdge>>();
	edges->reserve(possible_chargers.size() * charger_graph_->num_chargers);

	TIMER_START(GRAPH_SHRINKING);

	// Create a new list of edges where all edges are in the corridor of selected chargers
	{
		std::mutex mutex_;
		auto edges_end = charger_graph_->adj_list->cend();
		std::for_each(
				std::execution::par_unseq,
				possible_charger_ids.cbegin(),
				possible_charger_ids.cend(),
				[&mutex_, &edges, &edges_end, &possible_charger_ids, &lower_capacity_limit, &upper_capacity_limit, &parameters, this](
						const ChargerId &id) {
					auto start_it = charger_graph_->edge_begin_index_map.at(id);
					for (auto it = start_it; it != edges_end && it->start == id; it++) {
						for (auto &possible_end_charger_id: possible_charger_ids) {
							auto tmp_consumption = it->driving_factor * parameters.wltp + it->resistance_factor * parameters.weight;
							if (it->end == possible_end_charger_id && tmp_consumption >= lower_capacity_limit && tmp_consumption <= upper_capacity_limit) {
								std::lock_guard<std::mutex> lg{mutex_};
								edges->emplace_back(it->start, it->end, it->weight, it->driving_factor, it->resistance_factor);
								break;
							}
						}

					}
				});
	}


	TIMER_STOP(GRAPH_SHRINKING);
	util::Log(logDEBUG) << "[DijkstraAlongRoute] " << "Graph shrinking took " << TIMER_SEC(GRAPH_SHRINKING) << "secs";
	util::Log(logDEBUG) << "[DijkstraAlongRoute] " << "The new graph has " << edges->size() << " edges";
	if (edges->empty()) {
		result = util::json::Object();
		result.values["code"] = "InvalidQuery";
		result.values["message"] = "The filtered graph has no edges left";
		return Status::Error;
	}

	std::shared_ptr<enav::ChargerGraph> tmp_charger_graph =
			std::make_shared<enav::ChargerGraph>(charger_graph_->car,edges,possible_chargers);


//	{
//		std::ofstream out{"/tmp/test2/edges.geojson"};
//		out << tmp_charger_graph->edgesToString();
//	}

	auto reachable_chargers = getReachableChargers(
			phantom_node_pair_start,
			phantom_node_pair_end,
			tmp_charger_graph->charger_list,
			lower_capacity_limit,
			upper_capacity_limit,
			parameters.wltp,
			parameters.weight);
	if (reachable_chargers.to_end_reachable_chargers.empty() ||
	    reachable_chargers.from_start_reachable_chargers.empty()) {
		return Status::Error;
	}

//	{
//		std::ofstream out_to_end{"/tmp/test2/to_end.csv"};
//		out_to_end << "lon,lat" << std::endl;
//		for (const auto & it : reachable_chargers.to_end_reachable_chargers) {
//			out_to_end << possible_chargers[it.id].coordinate.ToString() << std::endl;
//		}
//	}
//
//	{
//		std::ofstream out_from_start{"/tmp/test2/from_start.csv"};
//		out_from_start << "lon,lat" << std::endl;
//		for (const auto & it : reachable_chargers.from_start_reachable_chargers) {
//			out_from_start << possible_chargers[it.id].coordinate.ToString() << std::endl;
//		}
//	}


	TIMER_START(ROUTE);
	auto charger_graph_result = tmp_charger_graph->shortestPath(
		reachable_chargers.from_start_reachable_chargers,
		reachable_chargers.to_end_reachable_chargers,
		parameters.wltp,
		parameters.weight);
	std::vector<ChargerId> used_charger_ids;
	std::transform(
			charger_graph_result.ids_of_path.cbegin(),
			charger_graph_result.ids_of_path.cend(),
			std::back_inserter(used_charger_ids),
			[&tmp_charger_graph](const auto & id) {
				for (const auto & it : tmp_charger_graph->renumbering_mapping) {
					if (it.second == id) {
						return it.first;
					}
				}
				throw std::runtime_error{"No matching between ids of shortest path and graph could be found. Aborting. Please contact a developer"};
			});
	TIMER_STOP(ROUTE);
	util::Log(logDEBUG) << "[DijkstraAlongRoute] " << "calculating the shortest charger route took " << TIMER_SEC(ROUTE) << " seconds" << std::endl;

	if (!charger_graph_result.found_route) {
		result = util::json::Object();
		result.values["code"] = "NoRoute";
		result.values["message"] = "No Shortest Path could be found in the filtered charger graph";
		return Status::Error;
	}


	return pointsToFinalRoute(
		parameters.output_format,
		phantom_node_pair_start,
		phantom_node_pair_end,
		used_charger_ids,
		battery_capacity,
		parameters.wltp,
		parameters.weight,
		result);
}


Status OSRM::EVRouteAlongRoute(EVRouteParameters &parameters, json::Object &result) const {
	if (parameters.algo == engine::api::EVRouteParameters::Algo::ALONG_ROUTE_SMALLEST_CONSUMPTION) {
		throw std::runtime_error{"Currently only ShortestDistance is supported as metric"};
	}

	auto battery_capacity = osrm::enav::temperature_dependent_capacity(charger_graph_->car->base_battery_capacity_milli_watt_h, parameters.temperature);

	std::uint32_t lower_capacity_limit = std::lround(parameters.lower_capacity_limit);
	std::uint32_t upper_capacity_limit = std::lround(parameters.upper_capacity_limit);

	auto phantom_node_pair_start = engine_->GetPhantomNodePair(parameters.start);
	auto phantom_node_pair_end = engine_->GetPhantomNodePair(parameters.end);
	std::vector<engine::PhantomNodePair> phantom_nodes{phantom_node_pair_start, phantom_node_pair_end};

	std::vector<ChargerId> used_charger_ids;

	RouteConsumption last_route_consumption;
	std::vector<engine::guidance::LegGeometry> tmp_result;
	auto status = this->engine_->ViaRouteInternal(phantom_nodes, tmp_result, last_route_consumption, parameters.wltp, parameters.weight);
	util::Log(logDEBUG) << "Consumption for direct route is " << last_route_consumption;
	if (status == Status::Error) {
		result = util::json::Object();
		result.values["code"] = "NoRoute";
		result.values["message"] = "No route could be calculated between start and end point";
		return Status::Error;
	}

	RouteConsumption prev_prev_consumption = 0;
	enav::Charger next_charger;


	while (last_route_consumption > upper_capacity_limit) {
		prev_prev_consumption = last_route_consumption;
		auto current_leg = tmp_result[tmp_result.size() - 1];
		if (consumptionOfLeg(current_leg, parameters.wltp, parameters.weight) < upper_capacity_limit) {
			break;
		}


		std::vector<enav::Charger> possible_chargers = getPossibleChargers(
				current_leg,
				charger_graph_->charger_list,
				lower_capacity_limit,
				upper_capacity_limit,
				parameters.search_radius,
				used_charger_ids,
				parameters.wltp,
				parameters.weight);


		bool found_valid_next_charger = false;

		//  just in case that the new route results in a consumption larger than the upper limit
		for (size_t i = 0; i < possible_chargers.size(); i++) {
			if (possible_chargers.empty()) {
				result = util::json::Object();
				result.values["code"] = "InvalidQuery";
				result.values["message"] = "Charger needed for route but no charger found";
				return Status::Error;
			}

			util::Log(logDEBUG) << "Number of possible chargers: " << possible_chargers.size();
			next_charger = possible_chargers[i];

			// Calculate Route from second last point in list to next chager to make sure that the inserted leg is correct
			std::vector<engine::PhantomNodePair> tmp_nodes{phantom_nodes[phantom_nodes.size() - 2],
			                                               next_charger.phantom_node_pair};
			std::vector<engine::guidance::LegGeometry> dummy_result;
			if (engine_->ViaRouteInternal(tmp_nodes, dummy_result) == engine::Status::Error) {
				continue;
			}


			BOOST_ASSERT(dummy_result.size() == 1);
			RouteConsumption tmp_consumption = consumptionOfLeg(dummy_result[0], parameters.wltp, parameters.weight);

			if (tmp_consumption <= upper_capacity_limit && tmp_consumption >= lower_capacity_limit) {
				found_valid_next_charger = true;
				break;
			}
		}

		if (!found_valid_next_charger) {
			util::Log(logERROR) << "Unable to find a valid charger along the route. Aborting";
			return engine::Status::Error;
		}


		util::Log(logDEBUG) << "Found new charger at " << next_charger.coordinate.ToString();
		phantom_nodes.insert(std::prev(phantom_nodes.end()), next_charger.phantom_node_pair);
		used_charger_ids.emplace_back(next_charger.node_id);

		tmp_result.clear();
		this->engine_->ViaRouteInternal(phantom_nodes, tmp_result, last_route_consumption, parameters.wltp, parameters.weight);
		if (prev_prev_consumption == last_route_consumption) {
			util::Log(logERROR) << "Twice in row the same consumption. This is very likely an error";
			std::ofstream out{"/tmp/test/invalid_along_routes" + util::uuid::generate_uuid_v4() + ".txt", std::ios_base::app};
			out << parameters.start.ToString() << ";" << parameters.end.ToString() << std::endl;
			return engine::Status::Error;
		}
	}

	if (phantom_nodes.size() < 2) {
		return engine::Status::Error;
	}

	return pointsToFinalRoute(
		parameters.output_format,
		phantom_node_pair_start,
		phantom_node_pair_end,
		used_charger_ids,
		battery_capacity,
		parameters.wltp,
		parameters.weight,
		result);
}


Status OSRM::EVRouteDijkstra(EVRouteParameters &parameters, json::Object &result) const {
	auto phantom_node_pair_start = engine_->GetPhantomNodePair(parameters.start);
	auto phantom_node_pair_end = engine_->GetPhantomNodePair(parameters.end);

	auto battery_capacity = osrm::enav::temperature_dependent_capacity(charger_graph_->car->base_battery_capacity_milli_watt_h, parameters.temperature);

	auto lower_capacity_limit = parameters.lower_capacity_limit;
	auto upper_capacity_limit = parameters.upper_capacity_limit;

	util::Log(logDEBUG) << "Calculating reachable chargers ...";
	auto reachable_chargers = getReachableChargers(
			phantom_node_pair_start,
			phantom_node_pair_end,
			charger_graph_->charger_list,
			lower_capacity_limit,
			upper_capacity_limit,
			parameters.wltp,
			parameters.weight);
	util::Log(logDEBUG) << "There are " << reachable_chargers.from_start_reachable_chargers.size()
	                    << " from start reachable chargers";
	util::Log(logDEBUG) << "There are " << reachable_chargers.to_end_reachable_chargers.size()
	                    << " to end reachable chargers";


	if (reachable_chargers.to_end_reachable_chargers.empty() ||
	    reachable_chargers.from_start_reachable_chargers.empty()) {
		result.values["code"] = "InvalidQuery";
		result.values["message"] = "Not enough chargers found from start or to end";
		return Status::Error;
	}
	util::Log(logDEBUG) << "Got reachable charger. Now starting Dijkstra ...";
	TIMER_START(ROUTE);
	auto charger_graph_result = this->charger_graph_->shortestPath(
		reachable_chargers.from_start_reachable_chargers,
		reachable_chargers.to_end_reachable_chargers,
		parameters.wltp,
		parameters.weight);
	if (!charger_graph_result.found_route) {
		return Status::Error;
	}


	TIMER_STOP(ROUTE);
	util::Log(logDEBUG) << "calculating the shortest charger route took " << TIMER_SEC(ROUTE) << " seconds"
	                    << std::endl;

	return pointsToFinalRoute(
		parameters.output_format,
		phantom_node_pair_start,
		phantom_node_pair_end,
		charger_graph_result.ids_of_path,
		battery_capacity,
		parameters.wltp,
		parameters.weight,
		result);
}


const Status OSRM::pointsToFinalRoute(const EVRouteParameters::OutputFormat output_format,
                                      const engine::PhantomNodePair start,
                                      const engine::PhantomNodePair end,
                                      const std::vector<ChargerId> & used_charger_ids,
									  const std::uint32_t battery_capacity,
									  const double wltp,
									  const double car_weight,
                                      json::Object &result) const {

	std::vector<engine::PhantomNodePair> phantom_node_pairs;
	phantom_node_pairs.emplace_back(start);
	std::transform(
			used_charger_ids.cbegin(),
			used_charger_ids.cend(),
			std::back_inserter(phantom_node_pairs),
			[this](const auto &charger_id) {
				return this->charger_graph_->charger_list[charger_id].phantom_node_pair;
			});
	phantom_node_pairs.emplace_back(end);

	Status status;
	if (output_format == engine::api::EVRouteParameters::OutputFormat::IP_FRONTEND) {
		result = util::json::Object();
		std::vector<engine::guidance::LegGeometry> route_legs;
		status = this->engine_->ViaRouteInternal(phantom_node_pairs, route_legs);
		if (status != engine::Status::Ok) {
			result.values["code"] = "InvalidQuery";
			result.values["message"] = "Unable to calculate final route";
			return Status::Error;
		}
		std::vector<std::uint32_t> max_power_of_used_chargers;
		std::transform(
				used_charger_ids.begin(),
				used_charger_ids.end(),
				std::back_inserter(max_power_of_used_chargers),
				[this](const auto &id) {
					return this->charger_graph_->charger_list[id].max_power;
				}
		);

		auto snapped_phantom_nodes = this->engine_->SnapPhantomNodes(phantom_node_pairs);
		std::vector<util::Coordinate> coords;
		std::transform(snapped_phantom_nodes.begin(), snapped_phantom_nodes.end(), std::back_inserter(coords),
		               [](const auto &it) { return it.location; });
		route_result_to_custom_json(result, battery_capacity, route_legs, max_power_of_used_chargers, coords, wltp, car_weight);

	} else if (output_format == engine::api::EVRouteParameters::OutputFormat::OSRM) {
		status = this->engine_->ViaRouteInternal(phantom_node_pairs, result);
	}
	return status;
}


} // namespace osrm
