//
// Created by kenspeckle on 17.02.22.
//

#include "util/consumption_utils.hpp"
#include "util/car.hpp"
#include <boost/assert.hpp>
#include <iostream>

namespace osrm {
namespace util {

namespace enav {


/**
 *
 * @param temperature
 * @param night
 * @param windscreen_wipers_needed
 * @return additional consumption in milliwatt
 */
std::int32_t aditional_consumptions(double temperature, bool night, bool windscreen_wipers_needed) {
	std::int32_t consumption = CONSUMPTION_MEDIA;
	consumption += CONSUMPTION_MOTOR_TEMP;
	if (night) {
		consumption += CONSUMPTION_NIGHT_LIGHT;
	} else {
		consumption += CONSUMPTION_DAY_LIGHT;
	}
	if (windscreen_wipers_needed) {
		consumption += CONSUMPTION_WINDSHIELD_WIPERS;
	}
	if (temperature > 25) {
		consumption += CONSUMPTION_AIR_CONDITIONING;    // TODO dynamisch von Temperatur abhängig machen
	} else if (temperature < 15) {
		consumption += CONSUMPTION_HEATER;              // TODO dynamisch von Temperatur abhängig machen
	}
	return consumption;
}


/**
 *	Calculates road resistance based on slope and rolling coefficients
 * @param c (pointer to the used car)
 * @param distance (in meter)
 * @param avg_speed (in kmh)
 * @param slope (in meter)
 * @return
 */
double calculate_road_resistance(const osrm::enav::Car &_car, double distance, float slope) {
	//TODO use surface provided by OSM

	double gradient = std::atan(slope / distance);
	double surface_coefficient = ASPHALT_ROLLING_COEFFICIENT;
	double rolling_resistance = _car.weight_times_gravity * surface_coefficient * std::cos(gradient);;
	double gradient_resistance = _car.weight_times_gravity * std::sin(gradient);
	return rolling_resistance + gradient_resistance;
}

/**
 * Gives WLTP usage for current step_segment
 * @param avg_speed
 * @return mWH/100km according to average speed
 */
double get_wltp_factor(const double avg_speed) {
	// Order is turned around as it is more likely that the speed will be larger that 74kmh (e.g over land roads or highways)
	// And that way we can use branch prediction
	if (avg_speed > WLTP_SPEED_EXTRAHIGH) {
		return WLTP_EXTRAHIGH_COEFFICIENT;
	} else if (avg_speed > WLTP_SPEED_HIGH) {
		return WLTP_HIGH_COEFFICIENT;
	} else if (avg_speed > WLTP_SPEED_MEDIUM) {
		return WLTP_MEDIUM_COEFFICIENT;
	} else {
		return WLTP_LOW_COEFFICIENT;
	}
}


double calculate_milli_watt_h_consumption(
		const osrm::enav::Car &_car,
		const double traveled_distance,
		const double avg_speed,
		const float height) {
	const double current_wltp = get_wltp_factor(avg_speed) * _car.wltp;
	double milli_wh_used;
	double gradient_resistance = _car.weight_times_gravity * height;
	double driving_consumption = current_wltp * 10.0 * traveled_distance;
	if (height >= 0) {
		milli_wh_used = driving_consumption + gradient_resistance / 3.6;
	} else {
		milli_wh_used = driving_consumption + gradient_resistance * 0.7 / 3.6;
	}
	return milli_wh_used;
}


std::pair<double, double> calculate_consumption_factors(const double & traveled_distance, const double & avg_speed, const double & height) {
	double gradient = std::asin(height / traveled_distance);
	double luftlinie = std::cos(gradient) * traveled_distance;
	double surface_coefficient = ASPHALT_ROLLING_COEFFICIENT;

	auto driving_factor = get_wltp_factor(avg_speed) * 10.0 * traveled_distance;

	double resistance_factor;
	if (height >= 0) {
		resistance_factor = (GRAVITY * (surface_coefficient * std::cos(gradient) * luftlinie + height)) / 3.6;
	} else {
		resistance_factor = (GRAVITY * (surface_coefficient * std::cos(gradient) * luftlinie + height * 0.7)) / 3.6;
	}
	return std::make_pair(driving_factor, resistance_factor);
}


}
}
}