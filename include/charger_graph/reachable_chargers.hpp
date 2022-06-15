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
	EdgeDrivingFactor driving_factor_to_reach_node;
	EdgeResistanceFactor resistance_factor_to_reach_node;
	EdgeWeight weight_to_reach_node;

public:
	ReachableStartNode(const Charger & charger, EdgeDrivingFactor driving_factor, EdgeResistanceFactor resistance_factor, EdgeWeight weight, double wltp, double car_weight) :
		id(charger.node_id), driving_factor_to_reach_node(driving_factor), resistance_factor_to_reach_node(resistance_factor), weight_to_reach_node(weight + calculate_charging_time(driving_factor, resistance_factor, charger.max_power, wltp, car_weight)) {}
		ReachableStartNode(const NodeID id, EdgeDrivingFactor driving_factor, EdgeResistanceFactor resistance_factor, EdgeWeight weight_to_reach_node) : id(id), driving_factor_to_reach_node(driving_factor), resistance_factor_to_reach_node(resistance_factor), weight_to_reach_node(weight_to_reach_node) {}
};


struct ReachableEndNode {
	NodeID id;
	EdgeDrivingFactor driving_factor_to_end;
	EdgeResistanceFactor resistance_factor_to_end;
	EdgeWeight weight_to_end;
public:
	ReachableEndNode(const NodeID nodeId, EdgeDrivingFactor driving_factor, EdgeResistanceFactor resistance_factor, EdgeWeight weight)
		: id(nodeId), driving_factor_to_end(driving_factor), resistance_factor_to_end(resistance_factor), weight_to_end(weight) {}
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
