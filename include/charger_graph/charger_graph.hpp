//
// Created by kenspeckle on 3/21/22.
//

#ifndef OSRM_CHARGER_GRAPH_H
#define OSRM_CHARGER_GRAPH_H

#include <vector>
#include <memory>
#include "charger.hpp"
#include "util/typedefs.hpp"
#include "charger_graph_edge.hpp"
#include "util/car.hpp"
#include "charger_filter.hpp"
#include "charger_utils.hpp"
#include "reachable_chargers.hpp"
#include "engine/engine_config.hpp"

namespace osrm {
namespace enav {

struct ShortestPathResult {
	std::vector<util::Coordinate> points_of_path;
	std::vector<ChargerId> ids_of_path;
	RouteConsumption consumption_of_route;
	bool found_route;

	ShortestPathResult(bool _found_route) : found_route(_found_route) {}
	ShortestPathResult(std::vector<util::Coordinate> _points, std::vector<ChargerId> _ids_of_path, RouteConsumption _consumption, bool _found_route)
		: points_of_path(std::move(_points)), ids_of_path(std::move(_ids_of_path)), consumption_of_route(_consumption), found_route(_found_route) {}
};


struct TmpShortestPathResult {
//	ChargerId start;
//	ChargerId end;
	std::vector<ChargerId> path;
	EdgeWeight weight;
//	RouteConsumption consumption;
};

class ChargerGraph {

public:
	explicit ChargerGraph(std::shared_ptr<Car>  car, std::shared_ptr<std::vector<ChargerGraphEdge>> & adj_list_, std::vector<Charger> charger_list_);
	ShortestPathResult shortestPath(const std::vector<ReachableStartNode> & starts, const std::vector<ReachableEndNode> & ends) const ;
	std::tuple<std::vector<ChargerId>, std::vector<EdgeWeight>, std::vector<RouteConsumption>> buildShortestPathTree(const ReachableStartNode & start) const;
	std::vector<TmpShortestPathResult> generateAllShortestPaths() const;

	const std::shared_ptr<std::vector<ChargerGraphEdge>> adj_list;
	std::unordered_map<ChargerId, std::vector<ChargerGraphEdge>::const_iterator> edge_begin_index_map;
	std::vector<Charger> charger_list;
	std::vector<ChargerId> charger_id_list;
	std::map<ChargerId, ChargerId> renumbering_mapping;
	const std::shared_ptr<Car> car;

	const size_t num_edges;
	const size_t num_chargers;

	std::string edgesToString() const;
};

}
}


#endif //OSRM_CHARGER_GRAPH_H
