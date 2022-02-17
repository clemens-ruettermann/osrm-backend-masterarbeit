//
// Created by kenspeckle on 3/21/22.
//

#include "charger_graph/charger_graph.hpp"
#include "charger_utils.hpp"
#include "charger_graph/files.hpp"
#include "util/timing_util.hpp"

#include <utility>
#include <threadpool.h>

namespace osrm {
namespace enav {


ChargerGraph::ChargerGraph(std::shared_ptr<Car> car_, std::shared_ptr<std::vector<ChargerGraphEdge>> & adj_list_, std::vector<Charger> charger_list_)
	: adj_list(std::move(adj_list_)), charger_list(std::move(charger_list_)), num_chargers(charger_list.size()), num_edges(adj_list->size()), edge_begin_index_map(), car(std::move(car_)), renumbering_mapping() {
	edge_begin_index_map.reserve(num_chargers);
	BOOST_ASSERT(num_edges != 0);
	BOOST_ASSERT(num_chargers != 0);

	charger_id_list = std::vector<ChargerId>(num_chargers);
	for (ChargerId i = 0; i < num_chargers; i++) {
		renumbering_mapping.insert(std::make_pair(charger_list[i].node_id, i));
		charger_id_list[i] = i;
		charger_list[i].node_id = i;
	}
	for (auto & it : *adj_list) {
		it.start = renumbering_mapping.at(it.start);
		it.end = renumbering_mapping.at(it.end);
	}

	std::sort(adj_list->begin(), adj_list->end(), [](const auto & e1, const auto & e2) {
		if (e1.start < e2.start) {
			return true;
		} else if (e1.start == e2.start) {
			return e1.end < e2.end;
		} else {
			return false;
		}
	});

	ChargerId current_node_id = 0;
	edge_begin_index_map.insert(std::make_pair(0, adj_list->cbegin()));
	const auto edges_end = adj_list->cend();
	for (auto it = adj_list->cbegin(); it < edges_end; it++) {
		if (it->start > current_node_id) {
			while (it->start > current_node_id) {
				current_node_id++;
				edge_begin_index_map.insert(std::make_pair(current_node_id, edges_end));
			}
			edge_begin_index_map[current_node_id] = it;
		}
	}
	while (current_node_id < num_chargers - 1) {
		current_node_id++;
		edge_begin_index_map.insert(std::make_pair(current_node_id, edges_end));
	}
}


constexpr ChargerId INVALID_NODE_ID = std::numeric_limits<ChargerId>::max();

std::vector<ChargerId> buildPathFromPrev(const std::vector<ChargerId> & prev, const ChargerId & start, const ChargerId & end) {
	std::vector<ChargerId> path;
	ChargerId current_node = end;
	do {
		path.emplace_back(current_node);
		current_node = prev[current_node];
		if (current_node == INVALID_NODE_ID) {
			path.clear();
			return path;
		}
	} while (current_node != start);
	path.emplace_back(start);
	std::reverse(path.begin(), path.end());
	return path;
}



ShortestPathResult ChargerGraph::shortestPath(const std::vector<ReachableStartNode> & starts, const std::vector<ReachableEndNode> & ends, const double wltp, const double car_weight) const {
	std::vector<ChargerId> best_path;
	EdgeWeight best_weight = INVALID_EDGE_WEIGHT;
	EdgeDrivingFactor best_driving_factor = INVALID_EDGE_DRIVING_FACTOR;
	EdgeResistanceFactor best_resistance_factor = INVALID_EDGE_RESISTANCE_FACTOR;

	unsigned long counter = 0;
	unsigned long number_starts = starts.size();
#define DIJKSTRA_PARALELL
#ifdef DIJKSTRA_PARALELL
	thread_pool pool;
	std::mutex my_mutex;

	const auto best_route_func = [&, this] (const ReachableStartNode & start) {
		auto tree = buildShortestPathTree(start, wltp, car_weight);
		auto & prev = std::get<0>(tree);
		auto & weights = std::get<1>(tree);
		auto & driving_factors = std::get<2>(tree);
		auto & resistance_factors = std::get<3>(tree);
		for (auto end : ends) {
			auto path = buildPathFromPrev(prev, start.id, end.id);
			if (path.empty()) {
				continue;
			}
			EdgeWeight total_weight = weights[end.id] + end.weight_to_end;
			std::lock_guard<std::mutex> lg(my_mutex);
			if (total_weight < best_weight) {
				best_weight = total_weight;
				best_driving_factor = driving_factors[end.id] + end.driving_factor_to_end;
				best_resistance_factor = resistance_factors[end.id] + end.resistance_factor_to_end;
				best_path = path;
			}

		}
		std::lock_guard<std::mutex> lg(my_mutex);
		util::Log(logDEBUG) << (++counter)+1 << "/" << number_starts << std::endl;
	};

	for (auto & start : starts) {
		pool.push_task(best_route_func, start);
	}
	pool.wait_for_tasks();

#else
	for (const auto & start : starts) {
		util::Log(logDEBUG) << "Starting dijkstra " << (counter+1);
		TIMER_START(DIJKSTRA);
		const auto tree = buildShortestPathTree(start);
		const auto & prev = std::get<0>(tree);
		const auto & weights = std::get<1>(tree);
		const auto & consumptions = std::get<2>(tree);
		for (const auto & end : ends) {
			auto path = buildPathFromPrev(prev, start.id, end.id);
			if (path.empty()) {
				continue;
			}
			EdgeWeight totalWeight = weights[end.id] + end.weight_to_end;
			if (totalWeight < best_weight) {
				best_weight = totalWeight;
				best_consumption = consumptions[end.id] + end.consumption_to_end;
				best_path = path;
			}
		}
		TIMER_STOP(DIJKSTRA);
		util::Log(logDEBUG) << (++counter)+1 << "/" << number_starts << "\t this took " << TIMER_SEC(DIJKSTRA) << " sec" ;
	}
#endif
	if (best_path.empty()) {
		return {false};
	}

	std::vector<util::Coordinate> best_path_coords;
	for (const auto & it : best_path) {
		best_path_coords.emplace_back(charger_list[it].coordinate);
	}
	return ShortestPathResult{best_path_coords, best_path, best_driving_factor, best_resistance_factor, true};
}

std::tuple<std::vector<ChargerId>, std::vector<EdgeWeight>, std::vector<EdgeDrivingFactor>, std::vector<EdgeResistanceFactor>>
ChargerGraph::buildShortestPathTree(const ReachableStartNode & start, const double wltp, const double car_weight) const {
	std::vector<EdgeWeight> weights(num_chargers, INVALID_EDGE_WEIGHT);
	std::vector<EdgeDrivingFactor> driving_factors;
	std::vector<EdgeResistanceFactor> resistance_factors;
	std::vector<ChargerId> prev(num_chargers, INVALID_NODE_ID);

	weights[start.id] = start.weight_to_reach_node;
	driving_factors[start.id] = start.driving_factor_to_reach_node;
	resistance_factors[start.id] = start.resistance_factor_to_reach_node;
	prev[start.id] = start.id;

	const auto edges_end = adj_list->cend();
	std::vector<ChargerId> unscanned_nodes{this->charger_id_list};
	while(!unscanned_nodes.empty()) {
		ChargerId current_node = INVALID_NODE_ID;
		EdgeWeight current_weight = INVALID_EDGE_WEIGHT;
		bool found = false;
		std::vector<ChargerId>::iterator it_of_current_node;
		for (auto it = unscanned_nodes.begin(); it < unscanned_nodes.end(); it++) {
			if (weights[*it] < current_weight) {
				current_node = *it;
				current_weight = weights[current_node];
				found = true;
				it_of_current_node = it;
			}
		}
		if (!found) {
			break;
		}
		if (current_node == INVALID_NODE_ID || current_node > this->charger_id_list.size() ) {
			throw std::runtime_error{"Current node is invalid"};
		}
		unscanned_nodes.erase(it_of_current_node);
		for (auto it = edge_begin_index_map.at(current_node); it != edges_end && it->start == current_node; it++) {
			// We need to calculate how long it would take at the finish to charge
			// This needs to be added to the charging time of the first charger
			auto current_end = it->end;
			auto new_weight = current_weight + it->weight + calculate_charging_time(it->driving_factor, it->resistance_factor, this->charger_list[current_end].max_power, wltp, car_weight);
			if (it->weight != INVALID_EDGE_WEIGHT && new_weight < weights[current_end]) {
				weights[current_end] = new_weight;
				prev[current_end] = current_node;
				driving_factors[current_end] = driving_factors[current_node] + it->driving_factor;
				resistance_factors[current_end] = resistance_factors[current_node] + it->resistance_factor;
			}
		}
	}
	return std::make_tuple(std::move(prev), std::move(weights), std::move(driving_factors), std::move(resistance_factors));
}

std::string ChargerGraph::edgesToString() const {

	std::stringstream geojson_string_stream;
	geojson_string_stream << R"({)";
	geojson_string_stream << R"(
	"type": "FeatureCollection",
	"features": [)";
	for (size_t i = 0; i < adj_list->size(); i++) {
			geojson_string_stream << R"(
	{
		"type": "Feature",
		"geometry": {
			"type": "LineString",
			"coordinates": [)";
			auto begin = charger_list[adj_list->at(i).start].coordinate;
			geojson_string_stream << "[" << begin.ToString() << "], ";
			auto end =charger_list[adj_list->at(i).end].coordinate;
			geojson_string_stream << "[" << end.ToString() << "]";
			geojson_string_stream << R"(]
		}
	})";

		if (i == adj_list->size() -1) {
			continue;
		}
		geojson_string_stream << ",";
	}

	geojson_string_stream << "]" << std::endl;
	geojson_string_stream << R"(})";
	return geojson_string_stream.str();

}

}
}
