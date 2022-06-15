//
// Created by kenspeckle on 4/8/22.
//

#ifndef OSRM_PLUG_H
#define OSRM_PLUG_H

#include <vector>
#include "plug_type.hpp"



struct Plug {
	unsigned long  power_in_milli_w;
	std::vector<PlugType> plug_types;
	Plug(const unsigned long power_in_milli_w_, std::vector<PlugType> plug_types_) : power_in_milli_w(power_in_milli_w_), plug_types(std::move(plug_types_)) {}

	bool operator==(const Plug &rhs) const {
		if (power_in_milli_w != rhs.power_in_milli_w || plug_types.size() != rhs.plug_types.size()) {
			return false;
		}
		for (size_t i = 0; i < plug_types.size(); i++) {
			if (plug_types[i] != rhs.plug_types[i]) {
				return false;
			}
		}
		return true;
	}

	bool operator!=(const Plug &rhs) const {
		return !(rhs == *this);
	}

	bool operator<(const Plug &rhs) const {
		if (power_in_milli_w < rhs.power_in_milli_w)
			return true;
		if (rhs.power_in_milli_w < power_in_milli_w)
			return false;
		return plug_types < rhs.plug_types;
	}

	bool operator>(const Plug &rhs) const {
		return rhs < *this;
	}

	bool operator<=(const Plug &rhs) const {
		return !(rhs < *this);
	}

	bool operator>=(const Plug &rhs) const {
		return !(*this < rhs);
	}
};


#endif //OSRM_PLUG_H
