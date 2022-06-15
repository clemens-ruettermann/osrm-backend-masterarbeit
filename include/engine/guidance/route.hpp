#ifndef ROUTE_HPP
#define ROUTE_HPP

#include <cstdint>

namespace osrm
{
namespace engine
{
namespace guidance
{

struct Route
{
    double distance;
    double duration;
	double driving_factor;
	double resistance_factor;
    double weight;
};
} // namespace guidance
} // namespace engine
} // namespace osrm

#endif
