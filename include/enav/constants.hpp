//
// Created by kenspeckle on 17.02.22.
//

#ifndef OSRM_SRC_ENAV_CONSTANTS_HPP_
#define OSRM_SRC_ENAV_CONSTANTS_HPP_

#include <cstddef>
#include <cstdint>

// General usage constants
#define GRAVITY 9.81
#define AVG_WEIGHT_PERSON 70
#define METERS_IN_100KM 100000


// Aditional consumptions in milli Watt
const std::uint32_t CONSUMPTION_MEDIA = 40000;
const std::uint32_t CONSUMPTION_HEATER = 1200000;
const std::uint32_t CONSUMPTION_AIR_CONDITIONING = 6000000;
const std::uint32_t CONSUMPTION_MOTOR_TEMP = 500000;
const std::uint32_t CONSUMPTION_DAY_LIGHT = 8000;
const std::uint32_t CONSUMPTION_NIGHT_LIGHT = 125000;
const std::uint32_t CONSUMPTION_WINDSHIELD_WIPERS = 30000;


// Charging constants
#define OPTIMAL_TEMPERATURE 21.5
#define FAST_CHARGE_KW = 120
#define SLOW_CHARGE_COEFFICIENT = 0.3  // reduce charging by this amount if above SLOW_CHARGE_PERCENTAGE


// Surface restiance
// also for ordinary car tires on concrete, new asphalt, cobbles small new
#define ASPHALT_ROLLING_COEFFICIENT 0.012

//car tires on gravel - rolled new = 0.02
//car tires on cobbles-large worn = 0.03
// so we are building the average
#define GRAVEL_ROLLING_COEFFICIENT 0.025

//car tire on solid sand, gravel loose worn, soil medium hard = 0.04 - 0.08
#define SOIL_ROLLING_COEFFICIENT 0.06
#define RESISTANCE_COEFFICIENT 0.0005
#define RECUPERATION_COEFFICIENT 0.05


// WLTP
// Average lowest speed gates for WLTP phases
#define WLTP_SPEED_LOW 0
#define WLTP_SPEED_MEDIUM 29.2
#define WLTP_SPEED_HIGH 48.05
#define WLTP_SPEED_EXTRAHIGH 74.3

// WLTP phase multipliers
const double WLTP_LOW_COEFFICIENT        = 0.8;
const double WLTP_MEDIUM_COEFFICIENT     = 0.9;
const double WLTP_HIGH_COEFFICIENT       = 1;
const double WLTP_EXTRAHIGH_COEFFICIENT  = 1.32;


// Charging
const double FAST_CHARGE_PERCENTAGE = 30.0;
const double SLOW_CHARGE_PERCENTAGE = 80;

#endif //OSRM_SRC_ENAV_CONSTANTS_HPP_
