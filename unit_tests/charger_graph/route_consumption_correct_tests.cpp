//
// Created by kenspeckle on 3/21/22.
//

#include <boost/filesystem.hpp>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <vector>
#include "charger_graph/charger_graph_builder.hpp"
#include "engine/engine_config.hpp"

//#include <charger_graph/charger_graph_builder.hpp>

using namespace osrm;
using namespace osrm::engine;
using namespace osrm::enav;

BOOST_AUTO_TEST_SUITE(consumptions_test)

using namespace osrm::enav;

using CH = ::CH;

inline osrm::engine::Engine<engine::routing_algorithms::ch::Algorithm> getEngine(const std::string & base_path) {
	osrm::EngineConfig config;
	config.storage_config = {base_path};
	config.use_shared_memory = false;
	config.algorithm = osrm::EngineConfig::Algorithm::CH;
	return engine::Engine<engine::routing_algorithms::ch::Algorithm>(config);
}

BOOST_AUTO_TEST_CASE(manytomany_same_result_as_via_route_street) {
	auto engine = getEngine(TEST_DATA_DIR "/street.osrm");

	util::Coordinate start = util::Coordinate::FromDouble(7.4352131, 43.7500887);
	PhantomNode phantom_start = engine.GetPhantomNodePair(start);
	util::Coordinate end = util::Coordinate::FromDouble(7.4344551, 43.7508865);
	PhantomNode phantom_end = engine.GetPhantomNodePair(end);
	std::vector<util::Coordinate> point_vec{start, end};

	std::vector<std::vector<osrm::engine::guidance::LegGeometry>> via_route_legs;
	auto status_via_route_normal = engine.ViaRouteInternal(point_vec, via_route_legs);
	BOOST_CHECK(status_via_route_normal == Status::Ok);
	BOOST_CHECK_EQUAL(1, via_route_legs.size());
	RouteConsumption consumption_normal_via_route = 0;
	double distance_normal_via_route = 0;
	double duration_normal_via_route = 0;
	for (const auto & leg : via_route_legs[0]) {
		for (const auto & annotation : leg.annotations) {
			consumption_normal_via_route += annotation.consumption;
			distance_normal_via_route += annotation.distance;
			duration_normal_via_route += annotation.duration;
		}
	}

	std::vector<RouteConsumption> via_route_consumptions_result;
	std::vector<EdgeDistance> via_route_distances_result;
	auto status_via_route_consumptions = engine.ViaRouteInternalConsumptions(point_vec, via_route_consumptions_result, via_route_distances_result);
	BOOST_CHECK(status_via_route_consumptions == Status::Ok);
	BOOST_CHECK_EQUAL(via_route_consumptions_result.size(), 1);

	auto manytomany_result = engine.ManyToManyInternal({phantom_start, phantom_end}, {}, {}, false);
	auto manytomany_durations = std::get<0>(manytomany_result);
	auto manytomany_distances = std::get<1>(manytomany_result);
	BOOST_CHECK(manytomany_distances.empty());
	auto manytomany_consumptions = std::get<2>(manytomany_result);
	auto manytomany_weights = std::get<3>(manytomany_result);

	// Check that the routes (i,i) have all 0 values
	BOOST_CHECK_EQUAL(0, manytomany_consumptions[0]);
	BOOST_CHECK_EQUAL(0, manytomany_consumptions[3]);

	BOOST_CHECK_EQUAL(0, manytomany_durations[0]);
	BOOST_CHECK_EQUAL(0, manytomany_durations[3]);

	BOOST_CHECK_EQUAL(0, manytomany_weights[0]);
	BOOST_CHECK_EQUAL(0, manytomany_weights[3]);

	// Check duration and distances with via_route
	BOOST_CHECK_EQUAL(manytomany_durations[1], std::lround(duration_normal_via_route*10));

	// Check that the consumption from the first manytomany result is equal to the via_route results
	BOOST_CHECK_EQUAL(manytomany_consumptions[1], consumption_normal_via_route);
	std::cout << "Consumption: " << consumption_normal_via_route << std::endl;
	BOOST_CHECK_EQUAL(manytomany_consumptions[1], via_route_consumptions_result[0]);
	BOOST_CHECK_EQUAL(consumption_normal_via_route, via_route_consumptions_result[0]);
}



BOOST_AUTO_TEST_CASE(manytomany_same_result_as_via_route_monaco) {
	auto engine = getEngine(TEST_DATA_DIR "/monaco.osrm");

	util::Coordinate start = util::Coordinate::FromDouble(7.4352131, 43.7500887);
	PhantomNode phantom_start = engine.GetPhantomNodePair(start);
	util::Coordinate end = util::Coordinate::FromDouble(7.4344551, 43.7508865);
	PhantomNode phantom_end = engine.GetPhantomNodePair(end);
	std::vector<util::Coordinate> point_vec{start, end};

	std::vector<std::vector<osrm::engine::guidance::LegGeometry>> via_route_legs;
	auto status_via_route_normal = engine.ViaRouteInternal(point_vec, via_route_legs);
	BOOST_CHECK(status_via_route_normal == Status::Ok);
	BOOST_CHECK_EQUAL(1, via_route_legs.size());
	RouteConsumption consumption_normal_via_route = 0;
	double distance_normal_via_route = 0;
	double duration_normal_via_route = 0;
	for (const auto & leg : via_route_legs[0]) {
		for (const auto & annotation : leg.annotations) {
			consumption_normal_via_route += annotation.consumption;
			distance_normal_via_route += annotation.distance;
			duration_normal_via_route += annotation.duration;
		}
	}

	std::vector<RouteConsumption> via_route_consumptions_result;
	std::vector<EdgeDistance> via_route_distances_result;
	auto status_via_route_consumptions = engine.ViaRouteInternalConsumptions(point_vec, via_route_consumptions_result, via_route_distances_result);
	BOOST_CHECK(status_via_route_consumptions == Status::Ok);
	BOOST_CHECK_EQUAL(via_route_consumptions_result.size(), 1);

	auto manytomany_result = engine.ManyToManyInternal({phantom_start, phantom_end}, {}, {}, true);
	auto manytomany_durations = std::get<0>(manytomany_result);
	auto manytomany_distances = std::get<1>(manytomany_result);
	auto manytomany_consumptions = std::get<2>(manytomany_result);
	auto manytomany_weights = std::get<3>(manytomany_result);

	// Check that the routes (i,i) have all 0 values
	BOOST_CHECK_EQUAL(0, manytomany_consumptions[0]);
	BOOST_CHECK_EQUAL(0, manytomany_consumptions[3]);

	BOOST_CHECK_EQUAL(0, manytomany_durations[0]);
	BOOST_CHECK_EQUAL(0, manytomany_durations[3]);

	BOOST_CHECK_EQUAL(0, manytomany_distances[0]);
	BOOST_CHECK_EQUAL(0, manytomany_distances[3]);

	BOOST_CHECK_EQUAL(0, manytomany_weights[0]);
	BOOST_CHECK_EQUAL(0, manytomany_weights[3]);

	// Check duration and distances with via_route
	BOOST_CHECK_EQUAL(manytomany_durations[1], std::lround(duration_normal_via_route*10));
	BOOST_CHECK_EQUAL(manytomany_distances[1], distance_normal_via_route);

	// Check that the consumption from the first manytomany result is equal to the via_route results
	BOOST_CHECK_EQUAL(manytomany_consumptions[1], consumption_normal_via_route);
	BOOST_CHECK_EQUAL(manytomany_consumptions[1], via_route_consumptions_result[0]);
	BOOST_CHECK_EQUAL(consumption_normal_via_route, via_route_consumptions_result[0]);
}


BOOST_AUTO_TEST_CASE(manytomany_same_result_as_via_route_germany) {
	auto engine = getEngine("/home/kenspeckle/git/data/germany_elevation_double/germany-latest.osrm");

	util::Coordinate start = util::Coordinate::FromDouble(6.43451, 50.43788);
	PhantomNode phantom_start = engine.GetPhantomNodePair(start);
	util::Coordinate end = util::Coordinate::FromDouble(6.4260007, 50.4402747);
	PhantomNode phantom_end = engine.GetPhantomNodePair(end);
	std::vector<util::Coordinate> point_vec{start, end};

	std::vector<std::vector<osrm::engine::guidance::LegGeometry>> via_route_legs;
	auto status_via_route_normal = engine.ViaRouteInternal(point_vec, via_route_legs);
	BOOST_CHECK(status_via_route_normal == Status::Ok);
	BOOST_CHECK_EQUAL(1, via_route_legs.size());
	RouteConsumption consumption_normal_via_route = 0;
	double distance_normal_via_route = 0;
	double duration_normal_via_route = 0;
	for (const auto & leg : via_route_legs[0]) {
		for (const auto & annotation : leg.annotations) {
			consumption_normal_via_route += annotation.consumption;
			distance_normal_via_route += annotation.distance;
			duration_normal_via_route += annotation.duration;
		}
	}

	std::vector<RouteConsumption> via_route_consumptions_result;
	std::vector<EdgeDistance> via_route_distances_result;
	auto status_via_route_consumptions = engine.ViaRouteInternalConsumptions(point_vec, via_route_consumptions_result, via_route_distances_result);
	BOOST_CHECK(status_via_route_consumptions == Status::Ok);
	BOOST_CHECK_EQUAL(via_route_consumptions_result.size(), 1);

	auto manytomany_result = engine.ManyToManyInternal({phantom_start, phantom_end}, {}, {}, false);
	auto manytomany_durations = std::get<0>(manytomany_result);
	auto manytomany_distances = std::get<1>(manytomany_result);
	BOOST_CHECK(manytomany_distances.empty());
	auto manytomany_consumptions = std::get<2>(manytomany_result);
	auto manytomany_weights = std::get<3>(manytomany_result);

	// Check that the routes (i,i) have all 0 values
	BOOST_CHECK_EQUAL(0, manytomany_consumptions[0]);
	BOOST_CHECK_EQUAL(0, manytomany_consumptions[3]);

	BOOST_CHECK_EQUAL(0, manytomany_durations[0]);
	BOOST_CHECK_EQUAL(0, manytomany_durations[3]);

	BOOST_CHECK_EQUAL(0, manytomany_weights[0]);
	BOOST_CHECK_EQUAL(0, manytomany_weights[3]);

	// Check duration with via_route
	BOOST_CHECK_EQUAL(manytomany_durations[1], std::lround(duration_normal_via_route*10));

	// Check that the consumption from the first manytomany result is equal to the via_route results
	BOOST_CHECK_EQUAL(manytomany_consumptions[1], consumption_normal_via_route);
	BOOST_CHECK_EQUAL(manytomany_consumptions[1], via_route_consumptions_result[0]);
	BOOST_CHECK_EQUAL(consumption_normal_via_route, via_route_consumptions_result[0]);
}



BOOST_AUTO_TEST_SUITE_END()
