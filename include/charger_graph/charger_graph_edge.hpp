//
// Created by kenspeckle on 3/28/22.
//

#ifndef OSRM_CHARGER_GRAPH_EDGE_H
#define OSRM_CHARGER_GRAPH_EDGE_H

#include <vector>
#include "util/typedefs.hpp"

namespace osrm {
namespace enav {

struct ChargerGraphEdge {
	NodeID start;
	NodeID end;
	EdgeWeight weight;
	EdgeConsumption consumption;

public:
	ChargerGraphEdge() = default;
	ChargerGraphEdge(const ChargerGraphEdge & o) : start(o.start), end(o.end), weight(o.weight), consumption(o.consumption) {}
	ChargerGraphEdge(NodeID start_, NodeID end_, EdgeWeight weight_, EdgeConsumption consumption_) : start(start_), end(end_), weight(weight_), consumption(consumption_) {}

	bool operator<(const ChargerGraphEdge &rhs) const {
		if (start < rhs.start)
			return true;
		if (rhs.start < start)
			return false;
		return end < rhs.end;
	}

	bool operator>(const ChargerGraphEdge &rhs) const {
		return rhs < *this;
	}

	bool operator<=(const ChargerGraphEdge &rhs) const {
		return !(rhs < *this);
	}

	bool operator>=(const ChargerGraphEdge &rhs) const {
		return !(*this < rhs);
	}
};
}
}


#endif //OSRM_CHARGER_GRAPH_EDGE_H
