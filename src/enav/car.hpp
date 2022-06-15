//
// Created by kenspeckle on 17.02.22.
//

#ifndef OSRM_SRC_ENAV_CAR_HPP_
#define OSRM_SRC_ENAV_CAR_HPP_
#include <string>
#include "constants.hpp"
#include <stdexcept>
#include <cmath>

namespace enav {

class car {
public:
	const std::string name;
	const double wltp;
	// in kg
	const unsigned int base_mass;
	const double base_battery_capacity;
private:
	unsigned int passenger_mass{AVG_WEIGHT_PERSON};
	unsigned int load_mass{0};
	unsigned int combined_mass{0};
	double mass_times_gravity{0};
	double temperature_dependent_battery_capacity;
	double current_temperature{15};

public:
	car(std::string _name, double _wltp, unsigned int _base_mass, double _base_battery_capacity);

	unsigned int get_mass() const;

	void recalculate_mass();
	void set_passenger_mass(unsigned int _passenger_mass);
	void set_load_mass(unsigned int _load_mass);
	void set_temperature(double _temperature);
	double get_mass_times_gravity() const;


	static double calculate_battery_capacity_from_temperature(double base_capacity, double temperature) {
		if (temperature < -20.0) {
			throw std::runtime_error{"Temperature is too low " + std::to_string(temperature)};
		}
		if (temperature > 40) {
			throw std::runtime_error{"Temperature is too high" + std::to_string(temperature)};
		}
		double percentage = (-0.000000000113 * pow(temperature, 7)
			- 0.0000000017996 * pow(temperature, 6)
			+ 0.0000006755586 * pow(temperature, 5)
			- 0.0000199188916 * pow(temperature, 4)
			- 0.0016352268049 * pow(temperature, 3)
			+ 0.0172473378987 * pow(temperature, 2)
			+ 2.1568800486655 * temperature
			+ 78.4762547237674);
		return percentage * 0.01 * base_capacity;
	}

};
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
=======
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> 18d37b1a2 (Implementing algorithms to perform routing for electric vehicles.)
=======
=======
>>>>>>> origin/add_consumption

//mass = 2124;
//power = 150;
//battery_capacity = 77;
//ac_power = 22;
//dc_power = 126;
//wltp = 18.3;
car id4{"ID4", 18.3, 2124, 77};
>>>>>>> c72258724 (Started implementing charger filtering and consumption calculations)
=======
>>>>>>> ace6ba8fb (Continue ev specific code)
=======
>>>>>>> 00eba0c37 (Started Implementing)
<<<<<<< HEAD
<<<<<<< HEAD
=======
=======
>>>>>>> 00eba0c37 (Started Implementing)
>>>>>>> 18d37b1a2 (Implementing algorithms to perform routing for electric vehicles.)
=======
=======
>>>>>>> 00eba0c37 (Started Implementing)
>>>>>>> origin/add_consumption
}
#endif //OSRM_SRC_ENAV_CAR_HPP_
