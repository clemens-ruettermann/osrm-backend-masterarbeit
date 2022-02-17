//
// Created by kenspeckle on 3/7/22.
//

#ifndef OSRM_ELEVATION_NODE_HPP
#define OSRM_ELEVATION_NODE_HPP

#include "util/typedefs.hpp"
#include <iostream>

struct ElevationNode {
	ElevationNode() : node_id(), elevation() {}
	explicit ElevationNode(OSMNodeID node_id_, short elevation_) : node_id(node_id_), elevation(elevation_) {}
	ElevationNode(ElevationNode & o) = default;
	ElevationNode(ElevationNode && o) = default;
	~ElevationNode() = default;
	OSMNodeID node_id{};
	short elevation{};

	ElevationNode& operator=(const ElevationNode& other) {
		if (this != &other) {
			node_id = other.node_id;
			elevation = other.elevation;
		}
		return *this;
	}

	bool operator<(const ElevationNode &rhs) const {
		return node_id < rhs.node_id;
	}

	bool operator>(const ElevationNode &rhs) const {
		return rhs < *this;
	}

	bool operator<=(const ElevationNode &rhs) const {
		return !(rhs < *this);
	}

	bool operator>=(const ElevationNode &rhs) const {
		return !(*this < rhs);
	}

};
#endif //OSRM_ELEVATION_NODE_HPP
