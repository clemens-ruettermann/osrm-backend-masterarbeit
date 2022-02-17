//
// Created by kenspeckle on 17.02.22.
//

#ifndef OSRM_SRC_ENAV_CONSUMPTION_UTILS_HPP_
#define OSRM_SRC_ENAV_CONSUMPTION_UTILS_HPP_

#include <vector>
#include "enav/constants.hpp"
#include "car.hpp"
#include <cmath>

namespace osrm
{
namespace util
{
namespace enav
{

std::int32_t aditional_consumptions(double temperature, bool night, bool windscreen_wipers_needed);

double calculate_road_resistance(const osrm::enav::Car &_car, double distance, double slope);

/**
 *
 * @param _car
 * @param traveled_distance	(in meter)
 * @param duration_in_seconds	(in seconds)
 * @param height		(in meter)
 * @return How much does the car consume (or recharge) on a section with distance, duration_in_seconds and slope
 */
double
calculate_milli_watt_h_consumption(const osrm::enav::Car &_car, const double traveled_distance, const double avg_speed, const float height);

std::pair<double, double> calculate_consumption_factors(const double & traveled_distance, const double & avg_speed, const double & height);

}
}
}

#endif //OSRM_SRC_ENAV_CONSUMPTION_UTILS_HPP_
