//
// Created by kenspeckle on 20.04.22.
//

#ifndef OSRM_INCLUDE_CHARGER_GRAPH_REACHABLE_CHARGERS_HPP_
#define OSRM_INCLUDE_CHARGER_GRAPH_REACHABLE_CHARGERS_HPP_

#include "charger.hpp"
#include "charger_utils.hpp"

namespace osrm {
namespace enav {

struct ReachableStartNode {
	NodeID id;
	EdgeConsumption consumption_to_reach_node;
	EdgeWeight weight_to_reach_node;

public:
	ReachableStartNode(const Charger & charger, EdgeConsumption consumption, EdgeWeight weight) :
		id(charger.node_id), consumption_to_reach_node(consumption), weight_to_reach_node(weight + calculate_charging_time(consumption, charger.max_power)) {}
		ReachableStartNode(const NodeID id, EdgeConsumption consumption_to_reach_node, EdgeWeight weight_to_reach_node) : id(id), consumption_to_reach_node(consumption_to_reach_node), weight_to_reach_node(weight_to_reach_node) {}
};


struct ReachableEndNode {
	NodeID id;
	EdgeConsumption consumption_to_end;
	EdgeWeight weight_to_end;
public:
	ReachableEndNode(const NodeID nodeId, EdgeConsumption consumption, EdgeWeight weight) : id(nodeId), consumption_to_end(consumption), weight_to_end(weight) {}
};


struct ReachableChargers {
	ReachableChargers(std::vector<ReachableStartNode> from_start_reachable_chargers,
	                  std::vector<ReachableEndNode> to_end_reachable_chargers)
		: from_start_reachable_chargers(std::move(from_start_reachable_chargers)),
		  to_end_reachable_chargers(std::move(to_end_reachable_chargers)) {}

	std::vector<ReachableStartNode> from_start_reachable_chargers;
	std::vector<ReachableEndNode> to_end_reachable_chargers;
};

}
}

#endif //OSRM_INCLUDE_CHARGER_GRAPH_REACHABLE_CHARGERS_HPP_
