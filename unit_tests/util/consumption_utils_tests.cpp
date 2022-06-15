#include <boost/test/unit_test.hpp>

#include "util/bearing.hpp"
#include "util/consumption_utils.hpp"
#include <cmath>

using namespace osrm;
using namespace osrm::enav;
using namespace osrm::util;
using namespace osrm::util::enav;

BOOST_AUTO_TEST_SUITE(consumption_utils_tests)

BOOST_AUTO_TEST_CASE(consumption_get_wltp_100km) {
	const double wltp = 18.3;
	Car car("TestCar", wltp, 2124, 77 * 1000 * 1000, 100, {});
	const double distance = 100 * 1000;
	const double speed = 50;
	const double duration = distance/speed * 3600;
	auto res = calculate_milli_watt_h_consumption(car, distance, speed, 0);
	BOOST_CHECK_EQUAL(res, wltp * 1000 * 1000);
}



BOOST_AUTO_TEST_CASE(consumption_get_wltp_1km) {
	const double wltp = 18.3;
	const unsigned int base_weight = 1000;
	Car car("TestCar", wltp, base_weight, 77 * 1000 * 1000, 0, {});
	double distance = 100 *1000;
	double origin_distance = distance;
	const double speed = 50;
	const double duration = distance/speed * 3600;
	const double height = 0;
	distance = std::sqrt(distance*distance + height*height);
	auto res = calculate_milli_watt_h_consumption(car, distance, speed, height);

	double pot_engery = base_weight * GRAVITY * height;
	pot_engery /= 3.6;
	double driving_expect = wltp * distance * 10.0;
	double rolling_expect = 0.012 * base_weight * GRAVITY * origin_distance;
	double expect = pot_engery + driving_expect + rolling_expect;

	auto diff = expect - res;
	BOOST_CHECK_EQUAL(res, expect);
}




BOOST_AUTO_TEST_SUITE_END()
