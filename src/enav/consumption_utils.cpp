//
// Created by kenspeckle on 17.02.22.
//

#include "consumption_utils.hpp"
#include "car.hpp"

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 00eba0c37 (Started Implementing)
=======
>>>>>>> 00eba0c37 (Started Implementing)

namespace enav {


static double aditional_consumptions(double temperature, bool night, bool windscreen_wipers_needed) {
	double consumption = CONSUMPTION_MEDIA;
	consumption += CONSUMPTION_MOTOR_TEMP;
<<<<<<< HEAD
<<<<<<< HEAD
=======
namespace enav {


double aditional_consumptions(double temperature, bool night, bool windscreen_wipers_needed) {
	double consumption = CONSUMPTION_MEDIA + CONSUMPTION_MOTOR_TEMP;
>>>>>>> c72258724 (Started implementing charger filtering and consumption calculations)
=======

namespace enav {


static double aditional_consumptions(double temperature, bool night, bool windscreen_wipers_needed) {
	double consumption = CONSUMPTION_MEDIA;
	consumption += CONSUMPTION_MOTOR_TEMP;
>>>>>>> ace6ba8fb (Continue ev specific code)
=======
>>>>>>> 00eba0c37 (Started Implementing)
=======
>>>>>>> 00eba0c37 (Started Implementing)
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


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> ace6ba8fb (Continue ev specific code)
=======
>>>>>>> 00eba0c37 (Started Implementing)
=======
>>>>>>> 00eba0c37 (Started Implementing)
/**
 *	Calculates road resistance based on slope and rolling coefficients
 * @param c (pointer to the used car)
 * @param distance (in meter)
 * @param avg_speed (in kmh)
 * @param slope (in meter)
 * @return
 */
static double calculate_road_resistance(const enav::car *_car, const double distance, const double slope) {
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 00eba0c37 (Started Implementing)
=======
>>>>>>> 00eba0c37 (Started Implementing)
	//TODO use surface provided by OSM
	double gradient = std::atan(slope/distance);
	double rolling_resistance = _car->get_mass_times_gravity() * ASPHALT_ROLLING_COEFFICIENT * std::cos(gradient);;
	double gradient_resistance = _car->get_mass_times_gravity() * std::sin(gradient);
	return rolling_resistance + gradient_resistance;
}

/**
 * Gives WLTP usage for current step_segment
 * @param avg_speed
 * @return KWH/100km according to average speed
 */
static double get_wltp_factor(const double avg_speed) {
<<<<<<< HEAD
<<<<<<< HEAD
=======
inline double calculate_road_resistance(const enav::car *_car, const double distance, const double slope) {
=======
>>>>>>> ace6ba8fb (Continue ev specific code)
	//TODO use surface provided by OSM
	double gradient = std::atan(slope/distance);
	double rolling_resistance = _car->get_mass_times_gravity() * ASPHALT_ROLLING_COEFFICIENT * std::cos(gradient);;
	double gradient_resistance = _car->get_mass_times_gravity() * std::sin(gradient);
	return rolling_resistance + gradient_resistance;
}

<<<<<<< HEAD
inline double get_wltp_factor(const double avg_speed) {
>>>>>>> c72258724 (Started implementing charger filtering and consumption calculations)
=======
/**
 * Gives WLTP usage for current step_segment
 * @param avg_speed
 * @return KWH/100km according to average speed
 */
static double get_wltp_factor(const double avg_speed) {
>>>>>>> ace6ba8fb (Continue ev specific code)
=======
>>>>>>> 00eba0c37 (Started Implementing)
=======
>>>>>>> 00eba0c37 (Started Implementing)
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

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
double calculate_kwh_change(
=======
inline double calculate_kwh_change(
>>>>>>> c72258724 (Started implementing charger filtering and consumption calculations)
=======
double calculate_kwh_change(
>>>>>>> ace6ba8fb (Continue ev specific code)
=======
double calculate_kwh_change(
>>>>>>> 00eba0c37 (Started Implementing)
=======
double calculate_kwh_change(
>>>>>>> 00eba0c37 (Started Implementing)
		const enav::car * _car,
		const double distance,
		const double duration,
		const double avg_speed,
		const double slope) {

	//TODO tests durchführen und gucken ob dieser Check notwendig ist
	if (distance <= 0 || duration <= 0) {
		throw std::runtime_error{"Distance or duration is <= 0: distance: " + std::to_string(distance) + "; duration: " + std::to_string(duration)};
	}

	//TODO Wetter einbauen
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 00eba0c37 (Started Implementing)
=======
>>>>>>> 00eba0c37 (Started Implementing)
	const double additional_consumption_kwh =
		aditional_consumptions(OPTIMAL_TEMPERATURE, false, false)*(duration/3600);

	double current_wltp = get_wltp_factor(avg_speed) * _car->wltp;
<<<<<<< HEAD
<<<<<<< HEAD
=======
	//TODO testen ob Division durch Mutliplikation ersetzt werden kann
=======
>>>>>>> ace6ba8fb (Continue ev specific code)
	const double additional_consumption_kwh =
		aditional_consumptions(OPTIMAL_TEMPERATURE, false, false)*(duration/3600);

	double current_wltp = get_wltp_factor(avg_speed) * _car->wltp;
<<<<<<< HEAD

>>>>>>> c72258724 (Started implementing charger filtering and consumption calculations)
=======
>>>>>>> ace6ba8fb (Continue ev specific code)
=======
>>>>>>> 00eba0c37 (Started Implementing)
=======
>>>>>>> 00eba0c37 (Started Implementing)
	double driving_consumption_kwh = current_wltp * distance / METERS_IN_100KM;
	double kwh_used = driving_consumption_kwh + additional_consumption_kwh;

	// Add power usage/gain from road resistance
	double road_resistance = calculate_road_resistance(_car, distance, slope);

	// If resistance is negative recuperation efficiency is taken into account
	if (road_resistance > 0) {
		kwh_used +=  (road_resistance * RESISTANCE_COEFFICIENT) * (current_wltp / METERS_IN_100KM * distance);
	} else {
		kwh_used += ((road_resistance * RESISTANCE_COEFFICIENT) * RECUPERATION_COEFFICIENT) * distance / 50;
	}
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> c72258724 (Started implementing charger filtering and consumption calculations)
=======
>>>>>>> ace6ba8fb (Continue ev specific code)
=======
>>>>>>> 00eba0c37 (Started Implementing)
=======
>>>>>>> 00eba0c37 (Started Implementing)
	return kwh_used;
}


<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> c72258724 (Started implementing charger filtering and consumption calculations)
=======
>>>>>>> ace6ba8fb (Continue ev specific code)
=======
>>>>>>> 00eba0c37 (Started Implementing)
=======
>>>>>>> 00eba0c37 (Started Implementing)
}
