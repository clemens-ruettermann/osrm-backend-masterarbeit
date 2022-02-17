//
// Created by kenspeckle on 17.02.22.
//

#ifndef OSRM_SRC_ENAV_CAR_HPP_
#define OSRM_SRC_ENAV_CAR_HPP_

#include <string>
#include <stdexcept>
#include <cmath>
#include <vector>
#include "charger_graph/plug_type.hpp"

#define AVG_WEIGHT_PERSON 70

namespace osrm {
namespace enav {


class Car {
public:


	std::string name{"TestCar"};
	double wltp;
	// in kg
	unsigned int base_weight;
	std::uint32_t base_battery_capacity_milli_watt_h;
	double weight_times_gravity{0};
	std::vector<PlugType> supported_plug_types;
	unsigned long max_charging_power;

public:
	Car() = default;
	Car(const std::string &path);
	Car(std::string _name, double _wltp, unsigned int _base_weight, double _base_battery_capacity_kwh, std::vector<PlugType> supported_plug_types, unsigned long max_charging_power_kw);
	~Car() = default;

	void write_to_file(const std::string & path) const;


	static double calculate_battery_capacity_from_temperature(double base_capacity, double temperature) {
		if (temperature < -20.0) {
			throw std::runtime_error{"Temperature is too low " + std::to_string(temperature)};
		}
		if (temperature > 40) {
			throw std::runtime_error{"Temperature is too high" + std::to_string(temperature)};
		}
		//-0.000000000113 * x^7 - 0.0000000017996 * x^6 + 0.0000006755586 * x^5 - 0.0000199188916 * x^4 - 0.0016352268049 * x^3 + 0.0172473378987 * x^2 + 2.1568800486655 * x + 78.4762547237674
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
}
}
#endif //OSRM_SRC_ENAV_CAR_HPP_
