#include "extractor/compressed_edge_container.hpp"
#include "util/typedefs.hpp"
#include "util/car.hpp"
#include "osrm/coordinate.hpp"
#include "util/coordinate_calculation.hpp"
#include "util/consumption_utils.hpp"

#include <boost/test/unit_test.hpp>

#define BOOST_TEST_MODULE tolerance_01

BOOST_AUTO_TEST_SUITE(consumption_utils_test, * boost::unit_test::tolerance(0.00001))

using namespace osrm;
using namespace osrm::extractor;

std::pair<double, double> calculate_distance_slope_from_coords(
		const double lon1, const double lat1, const double elevation1,
		const double lon2, const double lat2, const double elevation2) {

	const util::FloatLongitude start_lon{lon1};
	const util::FloatLatitude start_lat{lat1};
	const osrm::util::Coordinate start{start_lon, start_lat};

	const util::FloatLongitude end_lon{lon2};
	const util::FloatLatitude end_lat{lat2};
	const osrm::util::Coordinate end{end_lon, end_lat};
	const double distance = osrm::util::coordinate_calculation::haversineDistance(start, end);
	const double slope = elevation2 - elevation1;

	return std::make_pair(distance, slope);
}

BOOST_AUTO_TEST_CASE(calculate_road_resistance_test)
{
	using namespace osrm::util::enav;
	osrm::enav::Car test_car{"VW ID.3 Pro S", 15.9, 2124, 77, 0, {}};

	BOOST_TEST(250.03728 == calculate_road_resistance(test_car, 18.386125607565972, 0.0));
	BOOST_TEST(-1292.88114508 == calculate_road_resistance(test_car, 20.210265419815858, -1.5));
	BOOST_TEST(79.94885704562435 == calculate_road_resistance(test_car, 134.75604355578477, -1.1));
}


BOOST_AUTO_TEST_CASE(calculate_milli_watt_h_consumption_test) {
	using namespace osrm::util::enav;
	osrm::enav::Car test_car{"VW ID.3 Pro S", 15.9, 2124, 77, 0, {}};

	const double avg_speed = 50;
	double distance = 0;
	double slope = 0;

//	BOOST_TEST(0 == calculate_milli_watt_h_consumption(&test_car, distance, (distance/1000) / avg_speed, avg_speed, slope));
//
//	distance = 1;
//	slope = 1;
//	BOOST_TEST(1344 == calculate_milli_watt_h_consumption(&test_car, distance, (distance/1000) / avg_speed, avg_speed, slope));
//
//	distance = 10;
//	slope = 10;
//	BOOST_TEST(13444 == calculate_milli_watt_h_consumption(&test_car, distance, (distance/1000) / avg_speed, avg_speed, slope));

	distance = 10;
	slope = -10;
	BOOST_TEST(-71194 == calculate_milli_watt_h_consumption(test_car, distance, avg_speed, slope));

	distance = 100;
	slope = -10;
	BOOST_TEST(-75325 == calculate_milli_watt_h_consumption(test_car, distance, avg_speed, slope));

}

//	std::pair<double, double> distance_slope_pair1 = calculate_distance_slope_from_coords(6.084915, 50.776278, 177.0, 6.085104, 50.776392, 177.0);
//	std::pair<double, double> distance_slope_pair2 = calculate_distance_slope_from_coords(6.085104, 50.776392, 177.0, 6.085293, 50.776528, 175.5);

/*

 	def test_calculate_road_resistance(self):
		step_segments_negative = route_A_C[1][0].steps[0].step_segments
		step_segments_positive = route_A_C[1][0].steps[15].step_segments

		self.assertAlmostEqual(routeCalculationService.calculate_road_resistance(TEST_CAR, step_segments_positive[4]),
							   360.57452514, 5)

	def test_get_current_wltp(self):
		test_point_a = Point(lat=50.775555, lon=6.083611)
		test_point_b = Point(lat=50.775565, lon=6.083611)
		test_step_segment = StepSegment(test_point_a, test_point_b)

		test_step_segment.avg_speed_in_kmh = routeCalculationService.WLTP_SPEED_LOW
		self.assertEqual(TEST_CAR.wltp * routeCalculationService.WLTP_LOW_COEFFICIENT,
						 routeCalculationService.get_current_wltp(TEST_CAR, test_step_segment))

		test_step_segment.avg_speed_in_kmh = routeCalculationService.WLTP_SPEED_MEDIUM
		self.assertEqual(TEST_CAR.wltp * routeCalculationService.WLTP_MEDIUM_COEFFICIENT,
						 routeCalculationService.get_current_wltp(TEST_CAR, test_step_segment))

		test_step_segment.avg_speed_in_kmh = routeCalculationService.WLTP_SPEED_HIGH
		self.assertEqual(TEST_CAR.wltp * routeCalculationService.WLTP_HIGH_COEFFICIENT,
						 routeCalculationService.get_current_wltp(TEST_CAR, test_step_segment))

		test_step_segment.avg_speed_in_kmh = routeCalculationService.WLTP_SPEED_EXTRAHIGH
		self.assertEqual(TEST_CAR.wltp * routeCalculationService.WLTP_EXTRAHIGH_COEFFICIENT,
						 routeCalculationService.get_current_wltp(TEST_CAR, test_step_segment))

		test_step_segment.avg_speed_in_kmh = -1
		self.assertEqual(TEST_CAR.wltp,
						 routeCalculationService.get_current_wltp(TEST_CAR, test_step_segment))

	def test_charge_level_on_route(self):
		chargers = []
		waypoints_a_c = routeCalculationService.charge_level_on_route(TEST_CAR, route_A_C, chargers)
		# Charge level according to volkswagen.de at the end of the trip 79%, 3% tolerance
		self.assertLessEqual(TEST_CAR.current_charge_level, 86)
		self.assertGreaterEqual(TEST_CAR.current_charge_level, 76)
		TEST_CAR.current_charge_level = 100

		waypoints_a_f = routeCalculationService.charge_level_on_route(TEST_CAR, route_A_F, chargers)
		# Charge level according to volkswagen.de at the end of the trip 25%, 15% tolerance
		self.assertLessEqual(TEST_CAR.current_charge_level, 50)
		self.assertGreaterEqual(TEST_CAR.current_charge_level, 10)
		TEST_CAR.charge(testCharger, 100)

		waypoints_a_m = routeCalculationService.charge_level_on_route(TEST_CAR, route_A_M, chargers)

		# Check for charge loss on first segment
		self.assertLessEqual(waypoints_a_c[0][0]['charge_level'], 100)
		self.assertLessEqual(waypoints_a_m[0][0]['charge_level'], 100)
		self.assertLessEqual(waypoints_a_f[0][0]['charge_level'], 100)

	def test_step_segment_charge_level_change(self):
		test_point_a = Point(lat=50.775555, lon=6.083611)
		test_point_b = Point(lat=50.775565, lon=6.083611)
		test_step_segment = StepSegment(test_point_a, test_point_b)
		test_step_segment.distance = 0
		test_step_segment.avg_speed_in_kmh = 50
		self.assertEqual(routeCalculationService.step_segment_charge_level_change(TEST_CAR, test_step_segment), 0)
		test_step_segment.distance = 1
		self.assertEqual(routeCalculationService.step_segment_charge_level_change(TEST_CAR, test_step_segment),
						 -0.0002323090438441559)
 */


BOOST_AUTO_TEST_SUITE_END()