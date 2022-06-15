#include "engine/plugins/viaroute.hpp"
#include "engine/api/route_api.hpp"
#include "engine/routing_algorithms.hpp"
#include "engine/status.hpp"

#include "util/for_each_pair.hpp"
#include "util/integer_range.hpp"
#include "util/json_container.hpp"

#include <cstdlib>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace osrm {
namespace engine {
namespace plugins {

ViaRoutePlugin::ViaRoutePlugin(int max_locations_viaroute, int max_alternatives)
		: max_locations_viaroute(max_locations_viaroute), max_alternatives(max_alternatives) {
}

Status ViaRoutePlugin::HandleRequest(const RoutingAlgorithmsInterface &algorithms,
                                     const api::RouteParameters &route_parameters,
                                     osrm::engine::api::ResultT &result) const {
	BOOST_ASSERT(route_parameters.IsValid());

	if (!algorithms.HasShortestPathSearch() && route_parameters.coordinates.size() > 2) {
		return Error("NotImplemented",
		             "Shortest path search is not implemented for the chosen search algorithm. "
		             "Only two coordinates supported.",
		             result);
	}

	if (!algorithms.HasDirectShortestPathSearch() && !algorithms.HasShortestPathSearch()) {
		return Error(
				"NotImplemented",
				"Direct shortest path search is not implemented for the chosen search algorithm.",
				result);
	}

	if (max_locations_viaroute > 0 &&
	    (static_cast<int>(route_parameters.coordinates.size()) > max_locations_viaroute)) {
		return Error("TooBig",
		             "Number of entries " + std::to_string(route_parameters.coordinates.size()) +
		             " is higher than current maximum (" +
		             std::to_string(max_locations_viaroute) + ")",
		             result);
	}

	// Takes care of alternatives=n and alternatives=true
	if ((route_parameters.number_of_alternatives > static_cast<unsigned>(max_alternatives)) ||
	    (route_parameters.alternatives && max_alternatives == 0)) {
		return Error("TooBig",
		             "Requested number of alternatives is higher than current maximum (" +
		             std::to_string(max_alternatives) + ")",
		             result);
	}

	if (!CheckAllCoordinates(route_parameters.coordinates)) {
		return Error("InvalidValue", "Invalid coordinate value.", result);
	}

	// Error: first and last points should be waypoints
	if (!route_parameters.waypoints.empty() &&
	    (route_parameters.waypoints[0] != 0 ||
	     route_parameters.waypoints.back() != (route_parameters.coordinates.size() - 1))) {
		return Error(
				"InvalidValue", "First and last coordinates must be specified as waypoints.", result);
	}

	if (!CheckAlgorithms(route_parameters, algorithms, result))
		return Status::Error;

	const auto &facade = algorithms.GetFacade();
	auto phantom_node_pairs = GetPhantomNodes(facade, route_parameters);
	if (phantom_node_pairs.size() != route_parameters.coordinates.size()) {
		return Error("NoSegment",
		             MissingPhantomErrorMessage(phantom_node_pairs, route_parameters.coordinates),
		             result);
	}
	BOOST_ASSERT(phantom_node_pairs.size() == route_parameters.coordinates.size());

	auto snapped_phantoms = SnapPhantomNodes(phantom_node_pairs);

	std::vector<PhantomNodes> start_end_nodes;
	auto build_phantom_pairs = [&start_end_nodes](const PhantomNode &first_node,
	                                              const PhantomNode &second_node) {
		start_end_nodes.push_back(PhantomNodes{first_node, second_node});
	};
	util::for_each_pair(snapped_phantoms, build_phantom_pairs);

	api::RouteAPI route_api{facade, route_parameters};

	InternalManyRoutesResult routes;

	// TODO: in v6 we should remove the boolean and only keep the number parameter.
	// For now just force them to be in sync. and keep backwards compatibility.
	const auto wants_alternatives =
			(max_alternatives > 0) &&
			(route_parameters.alternatives || route_parameters.number_of_alternatives > 0);
	const auto number_of_alternatives = std::max(1u, route_parameters.number_of_alternatives);

	// Alternatives do not support vias, only direct s,t queries supported
	// See the implementation notes and high-level outline.
	// https://github.com/Project-OSRM/osrm-backend/issues/3905
	if (1 == start_end_nodes.size() && algorithms.HasAlternativePathSearch() && wants_alternatives) {
		routes = algorithms.AlternativePathSearch(start_end_nodes.front(), number_of_alternatives);
	} else if (1 == start_end_nodes.size() && algorithms.HasDirectShortestPathSearch()) {
		routes = algorithms.DirectShortestPathSearch(start_end_nodes.front());
	} else {
		routes = algorithms.ShortestPathSearch(start_end_nodes, route_parameters.continue_straight);
	}

	// The post condition for all path searches is we have at least one route in our result.
	// This route might be invalid by means of INVALID_EDGE_WEIGHT as shortest path weight.
	BOOST_ASSERT(!routes.routes.empty());

	// we can only know this after the fact, different SCC ids still
	// allow for connection in one direction.

	if (routes.routes[0].is_valid()) {
		auto collapse_legs = !route_parameters.waypoints.empty();
		if (collapse_legs) {
			std::vector<bool> waypoint_legs(route_parameters.coordinates.size(), false);
			std::for_each(route_parameters.waypoints.begin(),
			              route_parameters.waypoints.end(),
			              [&](const std::size_t waypoint_index) {
				              BOOST_ASSERT(waypoint_index < waypoint_legs.size());
				              waypoint_legs[waypoint_index] = true;
			              });
			// First and last coordinates should always be waypoints
			// This gets validated earlier, but double-checking here, jic
			BOOST_ASSERT(waypoint_legs.front());
			BOOST_ASSERT(waypoint_legs.back());
			for (auto &route: routes.routes) {
				route = CollapseInternalRouteResult(route, waypoint_legs);
			}
		}

		route_api.MakeResponse(routes, start_end_nodes, result);
	} else {
		auto first_component_id = snapped_phantoms.front().component.id;
		auto not_in_same_component = std::any_of(snapped_phantoms.begin(),
		                                         snapped_phantoms.end(),
		                                         [first_component_id](const PhantomNode &node) {
			                                         return node.component.id != first_component_id;
		                                         });

		if (not_in_same_component) {
			return Error("NoRoute", "Impossible route between points", result);
		} else {
			return Error("NoRoute", "No route found between points", result);
		}
	}

	return Status::Ok;
}


Status ViaRoutePlugin::InternalRequest(
		const RoutingAlgorithmsInterface &algorithms,
		const std::vector<util::Coordinate> &coords,
		util::json::Object & result) const {
	api::RouteParameters params;
	params.number_of_alternatives = 0;
	params.coordinates = coords;
	params.annotations_type = api::RouteParameters::AnnotationsType::Consumption | api::RouteParameters::AnnotationsType::Distance | api::RouteParameters::AnnotationsType::Duration | api::RouteParameters::AnnotationsType::Speed;
	params.annotations = true;
	params.geometries = api::RouteParameters::GeometriesType::GeoJSON;
	params.steps = true;
	api::ResultT tmp_result = util::json::Object();
	auto status = HandleRequest(algorithms, params, tmp_result);

	result = tmp_result.get<util::json::Object>();
	return status;
}



Status ViaRoutePlugin::InternalRequest(
		const RoutingAlgorithmsInterface &algorithms,
		const std::vector<PhantomNodePair> &phantom_nodes,
		util::json::Object & result) const {



	api::RouteParameters params;
	params.number_of_alternatives = 0;
	params.annotations_type = api::RouteParameters::AnnotationsType::Consumption | api::RouteParameters::AnnotationsType::Distance | api::RouteParameters::AnnotationsType::Duration | api::RouteParameters::AnnotationsType::Speed;
	params.annotations = true;
	params.geometries = api::RouteParameters::GeometriesType::GeoJSON;
	params.steps = true;

	const auto &facade = algorithms.GetFacade();
	api::RouteAPI route_api{facade, params};

	InternalRouteResult route;
	auto snapped_phantoms = SnapPhantomNodes(phantom_nodes);
	for (auto & snapped : snapped_phantoms) {
		params.coordinates.emplace_back(snapped.location);
	}

	std::vector<PhantomNodes> start_end_nodes;
	auto build_phantom_pairs = [&start_end_nodes](const PhantomNode &first_node,
	                                              const PhantomNode &second_node) {
		start_end_nodes.push_back(PhantomNodes{first_node, second_node});
	};
	util::for_each_pair(snapped_phantoms, build_phantom_pairs);
	if (1 == start_end_nodes.size() && algorithms.HasDirectShortestPathSearch()) {
		route = algorithms.DirectShortestPathSearch(start_end_nodes.front());
	} else {
		route = algorithms.ShortestPathSearch(start_end_nodes, true);
	}

	if (!route.is_valid()) {
		return Status::Error;
	}

	route_api.MakeResponse(route, start_end_nodes, result);

	return Status::Ok;
}


/*
 * Assumes that the all nodes aside from start and end are chargers
 * The route_consumption represents the consumption on the last subroute
 */
Status ViaRoutePlugin::InternalRequest(const RoutingAlgorithmsInterface &algorithms,
                       const std::vector<PhantomNodePair> &phantom_node_pairs,
                       std::vector<guidance::LegGeometry> & result,
					   std::int64_t & last_route_consumption,
					   const double wltp,
					   const double weight) const {
	result.clear();
	InternalRouteResult route;

	auto snapped_phantoms = SnapPhantomNodes(phantom_node_pairs);

	std::vector<PhantomNodes> start_end_nodes;
	auto build_phantom_pairs = [&start_end_nodes](const PhantomNode &first_node,
	                                              const PhantomNode &second_node) {
		start_end_nodes.push_back(PhantomNodes{first_node, second_node});
	};
	util::for_each_pair(snapped_phantoms, build_phantom_pairs);
	if (1 == start_end_nodes.size() && algorithms.HasDirectShortestPathSearch()) {
		route = algorithms.DirectShortestPathSearch(start_end_nodes.front());
	} else {
		route = algorithms.ShortestPathSearch(start_end_nodes, true);
	}

	if (!route.is_valid()) {
		result.clear();
		return Status::Error;
	}
	auto number_of_subroutes = route.segment_end_coordinates.size();

	auto number_of_legs = route.segment_end_coordinates.size();
	result.reserve(number_of_legs);

	last_route_consumption = 0;
	for (size_t i = 0; i < number_of_subroutes; i++) {

		const auto &source_phantom = route.segment_end_coordinates[i].source_phantom;
		const auto &target_phantom = route.segment_end_coordinates[i].target_phantom;
		const auto &path_data_vec = route.unpacked_path_segments[i];
		const bool reversed_source = route.source_traversed_in_reverse[i];
		const bool reversed_target = route.target_traversed_in_reverse[i];
		const auto &facade = algorithms.GetFacade();
		auto leg_geometry = guidance::assembleGeometry(facade, path_data_vec, source_phantom,target_phantom, reversed_source,reversed_target);

		if (i == number_of_subroutes - 1) {
			for (auto & annotation : leg_geometry.annotations) {
				last_route_consumption += annotation.consumption_factor_pair.first * wltp + annotation.consumption_factor_pair.second * weight;
			}
		}
		result.push_back(std::move(leg_geometry));
	}

	return Status::Ok;
}



Status ViaRoutePlugin::InternalRequest(
		const RoutingAlgorithmsInterface &algorithms,
		const std::vector<util::Coordinate> &coords,
		std::vector<guidance::LegGeometry> & result) const {
	const auto &facade = algorithms.GetFacade();
	api::BaseParameters params;
	params.snapping = api::BaseParameters::SnappingType::Any;
	params.coordinates.insert(params.coordinates.begin(), coords.begin(), coords.end());
	auto phantom_node_pairs = GetPhantomNodes(facade, params);
	if (phantom_node_pairs.size() != coords.size()) {
		throw std::runtime_error("NoSegment: Could not find a matching segment for coordinates");
	}

	auto snapped_phantoms = SnapPhantomNodes(phantom_node_pairs);

	std::vector<PhantomNodes> start_end_nodes;
	auto build_phantom_pairs = [&start_end_nodes](const PhantomNode &first_node, const PhantomNode &second_node) {
		start_end_nodes.push_back(PhantomNodes{first_node, second_node});
	};
	util::for_each_pair(snapped_phantoms, build_phantom_pairs);

	InternalRouteResult route;
	if (1 == start_end_nodes.size() && algorithms.HasDirectShortestPathSearch()) {
		route = algorithms.DirectShortestPathSearch(start_end_nodes.front());
	} else {
		route = algorithms.ShortestPathSearch(start_end_nodes, true);
	}

	if (!route.is_valid()) {
		result.clear();
		return Status::Error;
	}
	auto number_of_subroutes = route.segment_end_coordinates.size();

	auto number_of_legs = route.segment_end_coordinates.size();
	result.reserve(number_of_legs);

	for (size_t i = 0; i < number_of_subroutes; i++) {
		const auto &source_phantom = route.segment_end_coordinates[i].source_phantom;
		const auto &target_phantom = route.segment_end_coordinates[i].target_phantom;
		const auto &path_data_vec = route.unpacked_path_segments[i];

		const bool reversed_source = route.source_traversed_in_reverse[i];
		const bool reversed_target = route.target_traversed_in_reverse[i];
		auto leg_geometry = guidance::assembleGeometry(facade, path_data_vec, source_phantom,target_phantom, reversed_source,reversed_target);
		result.push_back(std::move(leg_geometry));
	}

	return Status::Ok;
}





} // namespace plugins
} // namespace engine
} // namespace osrm
