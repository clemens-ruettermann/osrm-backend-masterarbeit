//
// Created by kenspeckle on 17.02.22.
//

#ifndef OSRM_SRC_ENAV_CONSUMPTION_UTILS_HPP_
#define OSRM_SRC_ENAV_CONSUMPTION_UTILS_HPP_

#include <vector>
#include "constants.hpp"
#include "car.hpp"
#include <cmath>

namespace enav {

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> 00eba0c37 (Started Implementing)
=======
>>>>>>> 00eba0c37 (Started Implementing)
static double aditional_consumptions(double temperature, bool night, bool windscreen_wipers_needed);





<<<<<<< HEAD
<<<<<<< HEAD
=======
double aditional_consumptions(double temperature, bool night, bool windscreen_wipers_needed);
=======
static double aditional_consumptions(double temperature, bool night, bool windscreen_wipers_needed);

>>>>>>> ace6ba8fb (Continue ev specific code)




<<<<<<< HEAD
/**
 * Gives WLTP usage for current step_segment
 * @param avg_speed
 * @return KWH/100km according to average speed
 */
inline double get_wltp_factor(const double avg_speed);
>>>>>>> c72258724 (Started implementing charger filtering and consumption calculations)
=======
>>>>>>> ace6ba8fb (Continue ev specific code)
=======
>>>>>>> 00eba0c37 (Started Implementing)
=======
>>>>>>> 00eba0c37 (Started Implementing)


/**
 *
 * @param _car
 * @param distance	(in meter)
 * @param duration	(in seconds)
 * @param slope		(in meter)
 * @return How much does the car consume (or recharge) on a section with distance, duration and slope
 */
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
double calculate_kwh_change(const enav::car * _car, const double distance, const double duration, const double avg_speed, const double slope);
double calculate_kwh_change_v1(const enav::car * _car, const double distance, const double duration, const double avg_speed, const double slope);
double calculate_kwh_change_v2(const enav::car * _car, const double distance, const double duration, const double avg_speed, const double slope);
double calculate_kwh_change_v3(const enav::car * _car, const double distance, const double duration, const double avg_speed, const double slope);
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
inline double calculate_kwh_change(const enav::car * _car, const double distance, const double duration, const double slope);
>>>>>>> c72258724 (Started implementing charger filtering and consumption calculations)
=======
>>>>>>> ace6ba8fb (Continue ev specific code)
=======
>>>>>>> 00eba0c37 (Started Implementing)
=======
>>>>>>> 00eba0c37 (Started Implementing)

}

#endif //OSRM_SRC_ENAV_CONSUMPTION_UTILS_HPP_
