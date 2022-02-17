#include "util/coordinate_calculation.hpp"

#ifndef NDEBUG
#include "util/log.hpp"
#endif
#include "osrm/coordinate.hpp"

#ifndef NDEBUG
#include <bitset>
#endif
#include <iomanip>
#include <iostream>
#include <limits>

namespace osrm
{
namespace util
{

bool Coordinate::IsValid() const
{
    return !(lat > FixedLatitude{static_cast<std::int32_t>(90 * COORDINATE_PRECISION)} ||
             lat < FixedLatitude{static_cast<std::int32_t>(-90 * COORDINATE_PRECISION)} ||
             lon > FixedLongitude{static_cast<std::int32_t>(180 * COORDINATE_PRECISION)} ||
             lon < FixedLongitude{static_cast<std::int32_t>(-180 * COORDINATE_PRECISION)});
}

bool FloatCoordinate::IsValid() const
{
    return !(lat > FloatLatitude{90} || lat < FloatLatitude{-90} || lon > FloatLongitude{180} ||
             lon < FloatLongitude{-180});
}

Coordinate Coordinate::FromDouble(const double lon, const double lat) {
	return Coordinate(toFixed(UnsafeFloatLongitude{lon}),toFixed(UnsafeFloatLatitude{lat}));
}

Coordinate Coordinate::FromFixed(const std::int32_t lon, const std::int32_t lat) {
	return Coordinate(FixedLongitude {lon},FixedLatitude{lat});
}

std::string Coordinate::ToString() const {
	return std::to_string((double) toFloating(lon)) + "," + std::to_string((double ) toFloating(lat));
}

std::string Coordinate::ToInvertedString() const {
	return std::to_string((double ) toFloating(lat)) + "," + std::to_string((double) toFloating(lon));
}

bool operator==(const Coordinate lhs, const Coordinate rhs)
{
    return lhs.lat == rhs.lat && lhs.lon == rhs.lon;
}
bool operator==(const FloatCoordinate lhs, const FloatCoordinate rhs)
{
    return lhs.lat == rhs.lat && lhs.lon == rhs.lon;
}

bool operator!=(const Coordinate lhs, const Coordinate rhs) { return !(lhs == rhs); }

bool Coordinate::operator<(const Coordinate &rhs) const {
	if (lon < rhs.lon)
		return true;
	if (rhs.lon < lon)
		return false;
	return lat < rhs.lat;
}

bool Coordinate::operator>(const Coordinate &rhs) const {
	return rhs < *this;
}

bool Coordinate::operator<=(const Coordinate &rhs) const {
	return !(rhs < *this);
}

bool Coordinate::operator>=(const Coordinate &rhs) const {
	return !(*this < rhs);
}

bool operator!=(const FloatCoordinate lhs, const FloatCoordinate rhs) { return !(lhs == rhs); }
} // namespace util
} // namespace osrm
