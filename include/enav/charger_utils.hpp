//
// Created by kenspeckle on 16.02.22.
//

#ifndef OSRM_SRC_ENAV_CHARGER_UTILS_HPP_
#define OSRM_SRC_ENAV_CHARGER_UTILS_HPP_

#include <vector>
#include <algorithm>
#include "osrm/coordinate.hpp"
#include "charger_graph/charger.hpp"
#include "util/coordinate_calculation.hpp"
#include "util/log.hpp"

namespace osrm {
namespace enav {

inline EdgeDuration calculate_charging_time(const EdgeConsumption consumption_in_milli_wh, const std::uint32_t max_power) {
	 return static_cast<EdgeDuration>(((double)consumption_in_milli_wh / (double)max_power) * 3600.0);
}

inline std::uint32_t temperature_dependent_capacity(const std::uint32_t base_capacity_milli_wh, const double temperature) {
	if (temperature < -15) {
		return std::lround(0.33 * base_capacity_milli_wh);
	} else if (temperature > 20) {
		return base_capacity_milli_wh;
	} else {
		return base_capacity_milli_wh * (0.015120879 * temperature + 0.61758);
	}
}

}
}


#endif // OSRM_SRC_ENAV_CHARGER_UTILS_HPP_
