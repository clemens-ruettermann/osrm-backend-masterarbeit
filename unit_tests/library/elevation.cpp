#include <boost/test/unit_test.hpp>

#include "osrm/elevator.hpp"
#include "osrm/elevation_config.hpp"

#include <thread>

BOOST_AUTO_TEST_SUITE(library_elevation)

BOOST_AUTO_TEST_CASE(test_elevation_with_invalid_config)
{
	osrm::ElevationConfig config;
	config.requested_num_threads = std::thread::hardware_concurrency();
	BOOST_CHECK_THROW(osrm::extract_elevation(config),
	                  std::exception); // including osrm::util::exception, osmium::io_error, etc.
}

BOOST_AUTO_TEST_CASE(test_extract_with_valid_config)
{
	osrm::ElevationConfig config;
	config.input_path = OSRM_TEST_DATA_DIR "/monaco.osm.pbf";
	config.UseDefaultOutputNames(OSRM_TEST_DATA_DIR "/monaco.osm.pbf");
	config.elevation_geotiffs = {OSRM_TEST_DATA_DIR "/../../profiles/car.lua"};
	config.requested_num_threads = std::thread::hardware_concurrency();
	BOOST_CHECK_NO_THROW(osrm::extract_elevation(config));
}

BOOST_AUTO_TEST_SUITE_END()
