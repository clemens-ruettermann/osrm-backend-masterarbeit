//
// Created by kenspeckle on 3/28/22.
//

#include <boost/test/unit_test.hpp>
#include "util/car.hpp"

using namespace osrm;
using namespace osrm::enav;



BOOST_AUTO_TEST_SUITE(car_test)

BOOST_AUTO_TEST_CASE(car_size_not_changed_test){
	constexpr auto size_of_car = sizeof(osrm::enav::Car);
}


BOOST_AUTO_TEST_CASE(car_serialization_test){

	Car car;
	car.name = "VW Id4";
	car.wltp = 18.3;
	car.base_weight = 2124;
	car.passenger_weight = 100;
	car.base_battery_capacity_milli_watt_h = 77 * 1000 * 1000;
	car.write_to_file("/home/kenspeckle/git/data/extract/germany_exact.osrm.properties");
	std::cout<<std::endl;
}

BOOST_AUTO_TEST_SUITE_END()
