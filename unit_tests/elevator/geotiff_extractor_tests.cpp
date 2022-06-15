#include "elevation/geotiff_extractor.hpp"

#include <boost/test/unit_test.hpp>

#include <algorithm>
#include <iterator>
#include <vector>
#include <fstream>

#include "common/to_string_utils.hpp"
#include "util/math_utils.h"
/**
 * The coords in elevation/gauss.geotiff have the following values
 * 0.5 0.5 0.5 0.5 0.5
 * 0.5 1.5 1.5 1.5 0.5
 * 0.5 1.5 2.5 1.5 0.5
 * 0.5 1.5 1.5 1.5 0.5
 * 0.5 0.5 0.5 0.5 0.5
 *
 * The geo coordinate limits for this geotiff are lon: [9.0, 9.5], lat: [51.0, 51.5]
 */


struct Fixture {
	Fixture(){
		// We avoid using GDALALLRegister() as we only need GeoTiff, so hopefully this will speed up things a little bit
		GetGDALDriverManager();
		GDALDriverManager::AutoLoadDrivers();
		GDALRegister_GTiff();
	}
	~Fixture(){
		GDALDumpOpenDatasets(stderr);
		GDALDestroyDriverManager();
	}
};

BOOST_FIXTURE_TEST_SUITE(geotiff_extractor_test, Fixture)

using namespace std;
using namespace osrm;
using namespace osrm::elevation;

BOOST_AUTO_TEST_CASE(test_is_geotiffpixel_valid) {
	std::vector<std::string> geotiff_path{TEST_DATA_DIR "/elevation/gauss.geotiff"};
	GeotiffExtractor extractor = GeotiffExtractor(geotiff_path);

	GeotiffPixelCoordinate coord_band_out_of_bounds{23, 0, 0};
	BOOST_CHECK(!extractor.pixel_coord_is_valid(coord_band_out_of_bounds));

	GeotiffPixelCoordinate coord_x_out_of_bounds{0, 23, 0};
	BOOST_CHECK(!extractor.pixel_coord_is_valid(coord_x_out_of_bounds));

	GeotiffPixelCoordinate coord_y_out_of_bounds{0, 0, 23};
	BOOST_CHECK(!extractor.pixel_coord_is_valid(coord_y_out_of_bounds));

	GeotiffPixelCoordinate coord_x_negative{0, -23, 0};
	BOOST_CHECK(!extractor.pixel_coord_is_valid(coord_x_negative));

	GeotiffPixelCoordinate coord_y_negative{0, 0, -23};
	BOOST_CHECK(!extractor.pixel_coord_is_valid(coord_y_negative));

	GeotiffPixelCoordinate coord_is_valid{0, 0, 0};
	BOOST_CHECK(extractor.pixel_coord_is_valid(coord_is_valid));
}

BOOST_AUTO_TEST_CASE(test_get_pixel_value) {
	std::vector<std::string> geotiff_path{TEST_DATA_DIR "/elevation/gauss.geotiff"};
	GeotiffExtractor extractor = GeotiffExtractor(geotiff_path);

	GeotiffPixelCoordinate coord_upper_left_corner{0, 0, 0};
	BOOST_CHECK_EQUAL(extractor.get_value_for_pixel(coord_upper_left_corner), 0.5);
	GeotiffPixelCoordinate coord_lower_left_corner{0, 4, 0};
	BOOST_CHECK_EQUAL(extractor.get_value_for_pixel(coord_lower_left_corner), 0.5);
	GeotiffPixelCoordinate coord_upper_right_corner{0, 0, 4};
	BOOST_CHECK_EQUAL(extractor.get_value_for_pixel(coord_upper_right_corner), 0.5);
	GeotiffPixelCoordinate coord_lower_right_corner{0, 4, 4};
	BOOST_CHECK_EQUAL(extractor.get_value_for_pixel(coord_lower_right_corner), 0.5);

	GeotiffPixelCoordinate coord_left_mid_corner{0, 1, 1};
	BOOST_CHECK_EQUAL(extractor.get_value_for_pixel(coord_left_mid_corner), 1.5);

	GeotiffPixelCoordinate coord_mid{0, 2, 2};
	BOOST_CHECK_EQUAL(extractor.get_value_for_pixel(coord_mid), 2.5);
}


BOOST_AUTO_TEST_CASE(test_get_value_matrix_corners) {
	/**
	 * In the corners we expect the values from the border to extend
	 */
	std::vector<std::string> geotiff_path{TEST_DATA_DIR "/elevation/numbered.geotiff"};
	GeotiffExtractor extractor = GeotiffExtractor(geotiff_path);

	GeotiffPixelCoordinate upper_left_corner_coordinate{0, 0, 0};
	auto matrix = extractor.get_matrix_values_for_center_pixel(upper_left_corner_coordinate);
	array<array<float, 3>, 3> expect{{
			                                  {{0.0, 0.0, 1.0}},
			                                  {{0.0, 0.0, 1.0}},
			                                  {{5.0, 5.0, 6.0}}
	                                  }};
	BOOST_CHECK_EQUAL(matrix.vals, expect);
	BOOST_CHECK_EQUAL(*matrix.row(0)[0], 0.0);
	BOOST_CHECK_EQUAL(*matrix.row(0)[1], 0.0);
	BOOST_CHECK_EQUAL(*matrix.row(0)[2], 1.0);
	BOOST_CHECK_EQUAL(*matrix.row(1)[0], 0.0);
	BOOST_CHECK_EQUAL(*matrix.row(1)[1], 0.0);
	BOOST_CHECK_EQUAL(*matrix.row(1)[2], 1.0);
	BOOST_CHECK_EQUAL(*matrix.row(2)[0], 5.0);
	BOOST_CHECK_EQUAL(*matrix.row(2)[1], 5.0);
	BOOST_CHECK_EQUAL(*matrix.row(2)[2], 6.0);


	GeotiffPixelCoordinate lower_left_corner_coordinate{0, 0, 4};
	matrix = extractor.get_matrix_values_for_center_pixel(lower_left_corner_coordinate);
	std::array<array<float, 3>, 3> expect2 = {{
			                                           {{15.0, 15.0, 16.0}},
			                                           {{20.0, 20.0, 21.0}},
			                                           {{20.0, 20.0, 21.0}}
	                                           }};
	BOOST_CHECK_EQUAL(matrix.vals, expect2);

	GeotiffPixelCoordinate upper_right_corner_coordinate{0, 4, 0};
	matrix = extractor.get_matrix_values_for_center_pixel(upper_right_corner_coordinate);
	array<array<float, 3>, 3> expect3 = {{
			                                      {{3.0, 4.0, 4.0}},
			                                      {{3.0, 4.0, 4.0}},
			                                      {{8.0, 9.0, 9.0}}
	                                      }};

	BOOST_CHECK_EQUAL(matrix.vals, expect3);

	GeotiffPixelCoordinate lower_right_corner_coordinate{0, 4, 4};
	matrix = extractor.get_matrix_values_for_center_pixel(lower_right_corner_coordinate);
	array<array<float, 3>, 3> expect4 = {{
			                                      {{18.0, 19.0, 19.0}},
			                                      {{23.0, 24.0, 24.0}},
			                                      {{23.0, 24.0, 24.0}}
	                                      }};
	BOOST_CHECK_EQUAL(matrix.vals, expect4);
}


BOOST_AUTO_TEST_CASE(test_get_value_matrix_edges) {
	/**
	 * At the edges we expect the values to extend
	 */
	std::vector<std::string> geotiff_path{TEST_DATA_DIR "/elevation/numbered.geotiff"};
	GeotiffExtractor extractor = GeotiffExtractor(geotiff_path);

	GeotiffPixelCoordinate left_edge_coordinate{0, 0, 3};
	auto matrix = extractor.get_matrix_values_for_center_pixel(left_edge_coordinate);
	array<array<float, 3>, 3> expect = {{
			                                      {{10.0, 10.0, 11.0}},
			                                      {{15.0, 15.0, 16.0}},
			                                      {{20.0, 20.0, 21.0}}
	                                      }};
	BOOST_CHECK_EQUAL(matrix.vals, expect);

	GeotiffPixelCoordinate lower_edge_coordinate{0, 3, 4};
	matrix = extractor.get_matrix_values_for_center_pixel(lower_edge_coordinate);
	std::array<array<float, 3>, 3> expect3 = {{
			                                           {{17.0, 18.0, 19.0}},
			                                           {{22.0, 23.0, 24.0}},
			                                           {{22.0, 23.0, 24.0}}
	                                           }};
	BOOST_CHECK_EQUAL(matrix.vals, expect3);

	GeotiffPixelCoordinate right_edge_coordinate{0, 4, 1};
	matrix = extractor.get_matrix_values_for_center_pixel(right_edge_coordinate);
	array<array<float, 3>, 3> expect2 = {{
			                                      {{3.0, 4.0, 4.0}},
			                                      {{8.0, 9.0, 9.0}},
			                                      {{13.0, 14.0, 14.0}}
	                                      }};
	BOOST_CHECK_EQUAL(matrix.vals, expect2);

	GeotiffPixelCoordinate upper_edge_coordinate{0, 1, 0};
	matrix = extractor.get_matrix_values_for_center_pixel(upper_edge_coordinate);
	array<array<float, 3>, 3> expect4{{
			                                  {{0.0, 1.0, 2.0}},
			                                  {{0.0, 1.0, 2.0}},
			                                  {{5.0, 6.0, 7.0}}
	                                  }};
	BOOST_CHECK_EQUAL(matrix.vals, expect4);
}


BOOST_AUTO_TEST_CASE(test_get_value_matrix_mid) {
	std::vector<std::string> geotiff_path{TEST_DATA_DIR "/elevation/numbered.geotiff"};
	GeotiffExtractor extractor = GeotiffExtractor(geotiff_path);

	GeotiffPixelCoordinate upper_edge_coordinate{0, 2, 2};
	auto matrix = extractor.get_matrix_values_for_center_pixel(upper_edge_coordinate);
	array<array<float, 3>, 3> expect{{
			                                  {{6.0, 7.0, 8.0}},
			                                  {{11.0, 12.0, 13.0}},
			                                  {{16.0, 17.0, 18.0}}
	                                  }};
	BOOST_CHECK_EQUAL(matrix.vals, expect);
}




BOOST_AUTO_TEST_CASE(test_get_geotiff_pixelcoords) {
	std::vector<std::string> geotiff_path{TEST_DATA_DIR "/elevation/numbered.geotiff"};
	GeotiffExtractor extractor = GeotiffExtractor(geotiff_path);

	auto coord_center = extractor.get_pixel_coordinate(9.25, 51.25);
	BOOST_CHECK_EQUAL(2, coord_center.x);
	BOOST_CHECK_EQUAL(2, coord_center.y);


	auto coord_upper_left = extractor.get_pixel_coordinate(9.01, 51.49);
	BOOST_CHECK_EQUAL(0, coord_upper_left.x);
	BOOST_CHECK_EQUAL(0, coord_upper_left.y);

	auto coord_upper_right = extractor.get_pixel_coordinate(9.49, 51.49);
	BOOST_CHECK_EQUAL(4, coord_upper_right.x);
	BOOST_CHECK_EQUAL(0, coord_upper_right.y);

	auto coord_lower_left = extractor.get_pixel_coordinate(9.01, 51.01);
	BOOST_CHECK_EQUAL(0, coord_lower_left.x);
	BOOST_CHECK_EQUAL(4, coord_lower_left.y);

	auto coord_lower_right = extractor.get_pixel_coordinate(9.49, 51.01);
	BOOST_CHECK_EQUAL(4, coord_lower_right.x);
	BOOST_CHECK_EQUAL(4, coord_lower_right.y);
}


BOOST_AUTO_TEST_CASE(test_get_value_for_pixel) {
	std::vector<std::string> geotiff_path{TEST_DATA_DIR "/elevation/numbered.geotiff"};
	GeotiffExtractor extractor = GeotiffExtractor(geotiff_path);

	auto coord_center = extractor.get_pixel_coordinate(9.25, 51.25);
	auto val_center = extractor.get_value_for_pixel(coord_center);
	BOOST_CHECK_EQUAL(12, val_center);

	auto coord_upper_left = extractor.get_pixel_coordinate(9.01, 51.49);
	auto val_upper_left = extractor.get_value_for_pixel(coord_upper_left);
	BOOST_CHECK_EQUAL(0, val_upper_left);

	auto coord_upper_right = extractor.get_pixel_coordinate(9.49, 51.49);
	auto val_upper_right = extractor.get_value_for_pixel(coord_upper_right);
	BOOST_CHECK_EQUAL(4, val_upper_right);

	auto coord_lower_left = extractor.get_pixel_coordinate(9.01, 51.01);
	auto val_lower_left = extractor.get_value_for_pixel(coord_lower_left);
	BOOST_CHECK_EQUAL(20, val_lower_left);

	auto coord_lower_right = extractor.get_pixel_coordinate(9.49, 51.01);
	auto val_lower_right = extractor.get_value_for_pixel(coord_lower_right);
	BOOST_CHECK_EQUAL(24, val_lower_right);
}


BOOST_AUTO_TEST_CASE(test_get_value_for_pixel_multiple_geotiffs, * boost::unit_test::tolerance(0.000001)) {
	std::vector<std::string> geotiff_paths{TEST_DATA_DIR "/elevation/monaco_north.tif", TEST_DATA_DIR "/elevation/monaco_south.tif"};
	GeotiffExtractor extractor = GeotiffExtractor(geotiff_paths);

	// The real values are calculated by gdallocationinfo
	auto coord_north = extractor.get_pixel_coordinate(7.4295763,43.7428030);
	auto val_north = extractor.get_value_for_pixel(coord_north);
	BOOST_CHECK_EQUAL(187, coord_north.x);
	BOOST_CHECK_EQUAL(248, coord_north.y);
	BOOST_TEST(36.6130066 == val_north);

	auto coord_south = extractor.get_pixel_coordinate(7.4255185,43.7383235);
	auto val_south = extractor.get_value_for_pixel(coord_south);
	BOOST_CHECK_EQUAL(173, coord_south.x);
	BOOST_CHECK_EQUAL(12, coord_south.y);
	BOOST_TEST(56.3583488 == val_south);
}


BOOST_AUTO_TEST_CASE(test_pixel_coord_to_wgs84, * boost::unit_test::tolerance(0.000001)) {
	std::vector<std::string> geotiff_paths{TEST_DATA_DIR "/elevation/numbered.geotiff"};
	GeotiffExtractor extractor = GeotiffExtractor(geotiff_paths);

	auto p = extractor.pixel_coord_to_wgs84(0, 0, 0);
	BOOST_TEST(p.first == 9.05);
	BOOST_TEST(p.second == 51.45);
}


BOOST_AUTO_TEST_CASE(test_pixel_coord_to_wgs84_monaco, * boost::unit_test::tolerance(0.000001)) {
	std::vector<std::string> geotiff_paths{TEST_DATA_DIR "/elevation/monaco_north.tif"};
	GeotiffExtractor extractor = GeotiffExtractor(geotiff_paths);

	auto p = extractor.pixel_coord_to_wgs84(0, 0, 0);
	double expected_lon = 7.3691608;
	double expected_lat = 43.7973058;
	BOOST_TEST(p.first == expected_lon);
	BOOST_TEST(p.second == expected_lat);
}


BOOST_AUTO_TEST_CASE(test_get_value, * boost::unit_test::tolerance(0.000001)) {
	std::vector<std::string> geotiff_paths{TEST_DATA_DIR "/elevation/monaco_north.tif"};
	GeotiffExtractor extractor = GeotiffExtractor(geotiff_paths);

	// The real values are calculated by gdallocationinfo
	auto coord_north = extractor.get_pixel_coordinate(7.4344551,43.7507997);

	auto val_north = extractor.get_value(coord_north);
	BOOST_TEST(109.22179703267571 == val_north);

	auto coord_north2 = extractor.get_pixel_coordinate(7.4295763,43.7428030);
	auto val_north2 = extractor.get_value(coord_north2);
	BOOST_TEST(31.455985182399736 == val_north2);
}


BOOST_AUTO_TEST_CASE(test_get_value_gauss, * boost::unit_test::tolerance(0.000001)) {
	std::vector<std::string> geotiff_paths{TEST_DATA_DIR "/elevation/gauss.geotiff"};
	GeotiffExtractor extractor = GeotiffExtractor(geotiff_paths);

	auto coord_center = extractor.get_pixel_coordinate(9.2500000,  51.2500000);
	BOOST_CHECK_EQUAL(coord_center.x, 2);
	BOOST_CHECK_EQUAL(coord_center.y, 2);

	auto val_center = extractor.get_value(coord_center);
	BOOST_TEST(109.22179703267571 == val_center);
}


BOOST_AUTO_TEST_SUITE_END()
