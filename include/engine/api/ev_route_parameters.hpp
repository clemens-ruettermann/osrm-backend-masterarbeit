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
	double upper_capacity_limit_percent = 1.0;
	double lower_capacity_limit_percent = 0.0;
    double fallback_speed = INVALID_FALLBACK_SPEED;
	bool skip_waypoints;
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

        if (fallback_speed <= 0) {
	        return false;
		}

        if (scale_factor <= 0) {
	        return false;
        }

		if (upper_capacity_limit_percent < 0) {
			return false;
		}

		if (lower_capacity_limit_percent < 0) {
			return false;
		}

		if (lower_capacity_limit_percent >= upper_capacity_limit_percent) {
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
