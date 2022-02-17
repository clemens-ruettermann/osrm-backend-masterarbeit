//
// Created by kenspeckle on 3/21/22.
//

#ifndef OSRM_CHARGER_HPP
#define OSRM_CHARGER_HPP
#include "osrm/coordinate.hpp"
#include "plug.hpp"
#include "plug_type.hpp"
#include <vector>
#include "util/typedefs.hpp"
#include "engine/phantom_node.hpp"
#include <cstdint>
#include <map>
#include <unordered_map>
#include <limits>

namespace osrm {
namespace enav {

static const unsigned int UNCLASSIFIED = std::numeric_limits<unsigned int>::max();
static const unsigned int NOISE = std::numeric_limits<unsigned int>::max()-1;
class Charger {

public:
	Charger() = default;
	Charger(ChargerId node_id, std::pair<engine::PhantomNode, engine::PhantomNode> phantom_node, Coordinate coord, bool is_cluster, unsigned long i1,
	        std::string _operator, std::vector<Plug> plugs, std::vector<Charger> charger_in_cluster,
	        unsigned long cluster_size, bool has_fast_charger, unsigned long min_power, unsigned long max_power);

	explicit Charger(ChargerId node_id, Coordinate coord);

	ChargerId node_id;
	std::pair<engine::PhantomNode, engine::PhantomNode> phantom_node_pair;
	osrm::util::Coordinate coordinate;
	unsigned long total_power_in_milli_w;
	std::string _operator;
	std::vector<Plug> plugs;
	bool has_fast_charger;
	unsigned long  min_power;
	unsigned long max_power;

	// clustering related atributes
	bool is_cluster;
	bool is_classified = false;
	std::vector<Charger> charger_in_cluster;
	unsigned int cluster_size;
	unsigned int cluster_id = UNCLASSIFIED;


	Charger(ChargerId node_id, std::pair<engine::PhantomNode, engine::PhantomNode> phantom_node_pair, osrm::util::Coordinate coordinate, std::string _operator, unsigned long  total_power, bool has_fast_charger, std::vector<Charger> charger_in_cluster);
	Charger(ChargerId node_id, std::pair<engine::PhantomNode, engine::PhantomNode> phantom_node_pair, osrm::util::Coordinate coordinate, std::string _operator, unsigned long  total_power, bool has_fast_charger, std::vector<Plug> plugs);
	bool has_plug_type(const PlugType & p) const;

};


}
}


#endif //OSRM_CHARGER_HPP
