#ifndef ENGINE_API_EVROUTE_PARAMETERS_HPP
#define ENGINE_API_EVROUTE_PARAMETERS_HPP

#include "engine/api/base_parameters.hpp"

#include <cstddef>

#include <algorithm>
#include <iterator>
#include <vector>

namespace osrm
{
namespace engine
{
namespace api
{


/**
 * Parameters specific to the OSRM EV Route service.
 *
 * Holds member attributes:
 *  - source: indices into coordinates indicating sources for the Table service, no sources means
 *             use all coordinates as sources
 *  - destinations: indices into coordinates indicating destinations for the Table service, no
 *                  destinations means use all coordinates as destinations
 *
 * \see OSRM, Coordinate, Hint, Bearing, RouteParame, RouteParameters, TableParameters,
 *      NearestParameters, TripParameters, MatchParameters and TileParameters
 */
struct EVRouteParameters
{
	util::Coordinate start;
	util::Coordinate end;
	double battery_capacity = -1;
	bool battery_capacity_set = false;
	double wltp = -1;
	double weight = -1;
	double upper_capacity_limit = -1;
	double lower_capacity_limit = -1;

	double raw_upper_capacity_limit = 100;
	bool raw_upper_capacity_limit_set = false;
	double raw_lower_capacity_limit = 0;
	bool raw_lower_capacity_limit_set = false;

	double search_radius = 10000;
	double temperature = 20;

    enum class AnnotationsType
    {
        None = 0,
        Duration = 0x01,
        Distance = 0x02,
		Consumption = 0x03,
        All = Duration | Distance | Consumption
    };

	AnnotationsType annotations = AnnotationsType::All;

	enum class Algo {
		DIJKSTRA,
		ALONG_ROUTE_SHORTEST_DISTANCE,
		ALONG_ROUTE_SMALLEST_CONSUMPTION,
		DIJKSTRA_ALONG_ROUTE
	};

	Algo algo = Algo::ALONG_ROUTE_SHORTEST_DISTANCE;

	enum class OutputFormat {
		IP_FRONTEND,
		OSRM
	};

	OutputFormat output_format = OutputFormat::IP_FRONTEND;
    double scale_factor = 1;

	EVRouteParameters() = default;
    EVRouteParameters(util::Coordinate start_, util::Coordinate end_) : start(start_), end(end_) {}


    bool IsValid() const
    {
		if (!start.IsValid() || !end.IsValid()) {
			return false;
		}

        if (scale_factor <= 0) {
	        return false;
        }

	    if (battery_capacity < 0) {
		    return false;
	    }

		if (raw_upper_capacity_limit < 0 || raw_upper_capacity_limit > 100) {
			return false;
		}

		if (raw_lower_capacity_limit < 0) {
			return false;
		}

		if (raw_lower_capacity_limit >= raw_upper_capacity_limit) {
			return false;
		}


	    if (upper_capacity_limit < 0 || upper_capacity_limit > battery_capacity) {
		    return false;
	    }

	    if (lower_capacity_limit < 0) {
		    return false;
	    }

		if (wltp < 0) {
			return false;
		}

		if (weight < 0) {
			return false;
		}

        return true;
    }
};
inline bool operator&(EVRouteParameters::AnnotationsType lhs, EVRouteParameters::AnnotationsType rhs)
{
    return static_cast<bool>(
        static_cast<std::underlying_type_t<EVRouteParameters::AnnotationsType>>(lhs) &
        static_cast<std::underlying_type_t<EVRouteParameters::AnnotationsType>>(rhs));
}

inline EVRouteParameters::AnnotationsType operator|(EVRouteParameters::AnnotationsType lhs,
                                                  EVRouteParameters::AnnotationsType rhs)
{
    return (EVRouteParameters::AnnotationsType)(
        static_cast<std::underlying_type_t<EVRouteParameters::AnnotationsType>>(lhs) |
        static_cast<std::underlying_type_t<EVRouteParameters::AnnotationsType>>(rhs));
}

inline EVRouteParameters::AnnotationsType &operator|=(EVRouteParameters::AnnotationsType &lhs,
                                                      EVRouteParameters::AnnotationsType rhs)
{
    return lhs = lhs | rhs;
}
} // namespace api
} // namespace engine
} // namespace osrm

#endif // ENGINE_API_EVROUTE_PARAMETERS_HPP
