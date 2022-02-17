#include "engine/guidance/assemble_route.hpp"

#include <numeric>

namespace osrm
{
namespace engine
{
namespace guidance
{

Route assembleRoute(const std::vector<RouteLeg> &route_legs)
{
    auto distance = std::accumulate(
        route_legs.begin(), route_legs.end(), 0., [](const double sum, const RouteLeg &leg) {
            return sum + leg.distance;
        });
    auto duration = std::accumulate(
        route_legs.begin(), route_legs.end(), 0., [](const double sum, const RouteLeg &leg) {
            return sum + leg.duration;
        });
    auto weight = std::accumulate(
        route_legs.begin(), route_legs.end(), 0., [](const double sum, const RouteLeg &leg) {
            return sum + leg.weight;
        });
	double driving_factor = std::accumulate(
			route_legs.begin(), route_legs.end(), 0, [] (const double sum, const RouteLeg &leg) {
				return sum + leg.driving_factor;
			});

	double resistance_factor = std::accumulate(
			route_legs.begin(), route_legs.end(), 0, [] (const double sum, const RouteLeg &leg) {
				return sum + leg.resistance_factor;
			});

    return Route{distance, duration, driving_factor, resistance_factor, weight};
}

} // namespace guidance
} // namespace engine
} // namespace osrm
