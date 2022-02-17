//
// Created by kenspeckle on 17.02.22.
//

#include "util/car.hpp"
#include <utility>
#include <vector>
#include <fstream>

#define GRAVITY 9.81

namespace osrm {
namespace enav {


Car::Car(std::string _name,
         const double _wltp,
         const unsigned int _base_weight,
         const double _base_battery_capacity_kwh,
		 std::vector<PlugType> supported_plug_types,
		 unsigned long max_charging_power_kw)
		: name(std::move(_name)),
		  wltp(_wltp),
		  base_weight(_base_weight),
		  base_battery_capacity_milli_watt_h(std::round(_base_battery_capacity_kwh * 1000 * 1000)),
		  supported_plug_types(std::move(supported_plug_types)),
		  max_charging_power(max_charging_power_kw * 1000 * 1000),
		  weight_times_gravity(base_weight * GRAVITY){
}


void Car::write_to_file(const std::string & path) const {
	std::ofstream out(path);
	if (!out.is_open()) {
		throw std::runtime_error{"Unable to open '" + path + "' to serialize the car"};
	}
	out << name << std::endl;
	out << wltp << std::endl;
	out << base_weight << std::endl;
	out << base_battery_capacity_milli_watt_h << std::endl;
	out << weight_times_gravity << std::endl;
	out << max_charging_power << std::endl;
	for (const auto & it : supported_plug_types) {
		for (const auto & itt : PLUG_TYPE_MAP) {
			if (it == itt.second) {
				out << itt.first << std::endl;
				break;
			}
		}
	}

	out.close();
}

Car::Car(const std::string & path) {

	std::ifstream in(path);
	if (!in.is_open()) {
		throw std::runtime_error{"Unable to open '" + path + "' to deserialize the car"};
	}

	std::string line;
	getline(in, line);
	this->name = line;

	getline(in, line);
	this->wltp = std::stod(line);

	getline(in, line);
	this->base_weight = std::stoul(line);

	getline(in, line);
	this->base_battery_capacity_milli_watt_h = std::stoul(line);

	getline(in, line);
	this->weight_times_gravity = std::stod(line);

	getline(in, line);
	this->max_charging_power = std::stoul(line);

	while (getline(in, line)) {
		this->supported_plug_types.push_back(PLUG_TYPE_MAP.at(line));
	}
	in.close();
}

}
}