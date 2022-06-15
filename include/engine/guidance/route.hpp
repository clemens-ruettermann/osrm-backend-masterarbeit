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
	std::int32_t consumption;
    double weight;
};
} // namespace guidance
} // namespace engine
} // namespace osrm

#endif
