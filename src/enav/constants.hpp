//
// Created by kenspeckle on 17.02.22.
//

#ifndef OSRM_SRC_ENAV_CONSTANTS_HPP_
#define OSRM_SRC_ENAV_CONSTANTS_HPP_

// General usage constants
#define GRAVITY 9.81
#define AVG_WEIGHT_PERSON 70
#define METERS_IN_100KM 100000


// Aditional consumptions
#define CONSUMPTION_MEDIA 40;
#define CONSUMPTION_HEATER 1200;
#define CONSUMPTION_AIR_CONDITIONING 6000;
#define CONSUMPTION_MOTOR_TEMP 500;
#define CONSUMPTION_DAY_LIGHT 8;
#define CONSUMPTION_NIGHT_LIGHT 125;
#define CONSUMPTION_WINDSHIELD_WIPERS 30;


// Charging constants
#define OPTIMAL_TEMPERATURE 21.5
#define FAST_CHARGE_KW = 120
#define FAST_CHARGE_PERCENTAGE = 30
#define SLOW_CHARGE_PERCENTAGE = 80
#define SLOW_CHARGE_COEFFICIENT = 0.3  // reduce charging by this amount if above SLOW_CHARGE_PERCENTAGE


// Surface restiance
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
// also for ordinary car tires on concrete, new asphalt, cobbles small new
#define ASPHALT_ROLLING_COEFFICIENT 0.012

//car tires on gravel - rolled new = 0.02
//car tires on cobbles-large worn = 0.03
// so we are building the average
#define GRAVEL_ROLLING_COEFFICIENT 0.025

//car tire on solid sand, gravel loose worn, soil medium hard = 0.04 - 0.08
#define SOIL_ROLLING_COEFFICIENT 0.06
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
#define ASPHALT_ROLLING_COEFFICIENT 0.012
>>>>>>> c72258724 (Started implementing charger filtering and consumption calculations)
=======
>>>>>>> ace6ba8fb (Continue ev specific code)
=======
>>>>>>> 00eba0c37 (Started Implementing)
=======
>>>>>>> 00eba0c37 (Started Implementing)
#define RESISTANCE_COEFFICIENT 0.0005
#define RECUPERATION_COEFFICIENT 0.05


// WLTP
// Average lowest speed gates for WLTP phases
#define WLTP_SPEED_LOW 0
#define WLTP_SPEED_MEDIUM 29.2
#define WLTP_SPEED_HIGH 48.05
#define WLTP_SPEED_EXTRAHIGH 74.3

// WLTP phase multipliers
#define WLTP_LOW_COEFFICIENT 0.8
#define WLTP_MEDIUM_COEFFICIENT 0.9
#define WLTP_HIGH_COEFFICIENT 1
#define WLTP_EXTRAHIGH_COEFFICIENT 1.32

#endif //OSRM_SRC_ENAV_CONSTANTS_HPP_
