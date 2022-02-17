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

BOOST_AUTO_TEST_SUITE(charger_graph_builder_tests)

using namespace osrm::enav;


ChargerGraphBuilderConfig getEngineConfig(const std::string& base_path) {
	ChargerGraphBuilderConfig config;
	boost::filesystem::path boost_base_path{base_path};
	boost::filesystem::path boost_charger_csv_path{TEST_DATA_DIR "/charger.csv"};
	config.base_path = boost_base_path;
	config.charger_csv_path = boost_charger_csv_path;
	config.engine_config.storage_config = {boost_base_path};
	config.engine_config.use_shared_memory = false;
	config.engine_config.algorithm = osrm::EngineConfig::Algorithm::CH;
	return config;
}


std::vector<Charger> getChargersInLine(unsigned int num, const double diff) {
	std::vector<Charger> ret;
	double lon = 10.0;
	double lat = 51.0;

	for (unsigned int i = 0; i < num; i++) {
		ret.emplace_back(i, util::Coordinate::FromDouble(lon, lat));
		lon += diff;
	}
	return ret;
}

BOOST_AUTO_TEST_CASE(get_phantom_node_test)
{

	const double lon = 7.4347555;
	const double lat = 43.7503471;
	const std::uint32_t node_id = 268167430;
	osrm::enav::ChargerGraphBuilder builder{getEngineConfig(TEST_DATA_DIR "/street/street.osrm")};
	//lat="" lon=""
	const auto coord = util::Coordinate::FromDouble(lon, lat);

//	auto hint_str = builder.getPhantomNodeForCoord(coord);
//
//	Hint hint = Hint::FromBase64(hint_str);
//	BOOST_CHECK_EQUAL((double)util::toFloating(hint.phantom.location.lon), lon);
//	BOOST_CHECK_EQUAL((double)util::toFloating(hint.phantom.location.lat), lat);
//	BOOST_CHECK_EQUAL(hint.phantom.forward_segment_id.id, node_id);
//	BOOST_CHECK_EQUAL(hint.phantom.reverse_segment_id.id, node_id);

	builder.parseChargerFile(TEST_DATA_DIR "/charger.csv", {});

//	BOOST_CHECK(data.GetPropertyIndexes(point_t(0, 0)).empty());
//	CHECK_EQUAL_RANGE(data.GetPropertyIndexes(point_t(1, 1)), 0);
//	CHECK_EQUAL_RANGE(data.GetPropertyIndexes(point_t(0, 1)), 0);
//	CHECK_EQUAL_RANGE(data.GetPropertyIndexes(point_t(0.5, -0.5)), 0);
//	CHECK_EQUAL_RANGE(data.GetPropertyIndexes(point_t(0, -3)), 0);
//	CHECK_EQUAL_RANGE(data.GetPropertyIndexes(point_t(-0.75, 0.75)), 0);
//	CHECK_EQUAL_RANGE(data.GetPropertyIndexes(point_t(2, 0)), 0);
//	CHECK_EQUAL_RANGE(data.GetPropertyIndexes(point_t(1, 7)), 1);
//	CHECK_EQUAL_RANGE(data.GetPropertyIndexes(point_t(-2, 6)), 1);
//
//	BOOST_CHECK_EQUAL(data.FindByKey({}, "answer").which(), 0);
//	BOOST_CHECK_EQUAL(data.FindByKey({0}, "foo").which(), 0);
//	BOOST_CHECK_EQUAL(boost::get<double>(data.FindByKey({0}, "answer")), 42);
//	BOOST_CHECK_EQUAL(boost::get<bool>(data.FindByKey({1}, "answer")), true);
}





BOOST_AUTO_TEST_CASE(dbscan_line_10_points_test) {
	const double diff = 0.00001; //Aprox 1.1m
	auto chagers_in_cluster = getChargersInLine(10, diff);
	ChargerGraphBuilder builder{chagers_in_cluster, 2, 2};
	builder.dBScan();

	BOOST_CHECK_EQUAL(builder.chargers[0].cluster_id, 0);
	BOOST_CHECK_EQUAL(builder.chargers[1].cluster_id, 0);
	BOOST_CHECK_EQUAL(builder.chargers[2].cluster_id, 0);
	BOOST_CHECK_EQUAL(builder.chargers[3].cluster_id, 0);
	BOOST_CHECK_EQUAL(builder.chargers[4].cluster_id, 0);
	BOOST_CHECK_EQUAL(builder.chargers[5].cluster_id, 0);
	for (size_t i = 0; i < 10; i++) {
		BOOST_CHECK_EQUAL(builder.chargers[i].cluster_id, 0);
	}

}


BOOST_AUTO_TEST_CASE(dbscan_line_with_noise_test) {
	const double diff = 0.00001; //Aprox 1.1m
	auto chagers_in_cluster = getChargersInLine(10, diff);
	auto charger_outside_top_right = Charger(10, util::Coordinate::FromDouble(51 + 20*diff, 10 + 20*diff));
	auto charger_outside_top_left = Charger(11, util::Coordinate::FromDouble(51 - 20*diff, 10 + 20*diff));

	std::vector<Charger> all_chargers{chagers_in_cluster};
	all_chargers.push_back(charger_outside_top_right);
	all_chargers.push_back(charger_outside_top_left);

	ChargerGraphBuilder builder{all_chargers, 2, 1};
	builder.dBScan();

	for (size_t i = 0; i < 10; i++) {
		BOOST_CHECK_EQUAL(builder.chargers[i].cluster_id, 0);
	}

	BOOST_CHECK_EQUAL(builder.chargers[10].cluster_id, NOISE);
	BOOST_CHECK_EQUAL(builder.chargers[11].cluster_id, NOISE);
}


BOOST_AUTO_TEST_CASE(dbscan_two_clusters_test) {
	const double diff = 0.000005; //Aprox 55cm
	auto chagers_in_cluster = getChargersInLine(10, diff);
	auto second_cluster1 = Charger(10, util::Coordinate::FromDouble(51 + 20*diff, 10 + 20*diff));
	auto second_cluster2 = Charger(11, util::Coordinate::FromDouble(51 + 20*diff, 10 + 21*diff));
	auto second_cluster3 = Charger(12, util::Coordinate::FromDouble(51 + 20*diff, 10 + 22*diff));
	auto second_cluster4 = Charger(13, util::Coordinate::FromDouble(51 + 21*diff, 10 + 21*diff));

	std::vector<Charger> all_chargers{chagers_in_cluster};
	all_chargers.push_back(second_cluster1);
	all_chargers.push_back(second_cluster2);
	all_chargers.push_back(second_cluster3);
	all_chargers.push_back(second_cluster4);

	ChargerGraphBuilder builder{all_chargers, 2, 1};
	builder.dBScan();

	for (size_t i = 0; i < 10; i++) {
		BOOST_CHECK_EQUAL(builder.chargers[i].cluster_id, 0);
	}

	BOOST_CHECK_EQUAL(builder.chargers[10].cluster_id, 1);
	BOOST_CHECK_EQUAL(builder.chargers[11].cluster_id, 1);
	BOOST_CHECK_EQUAL(builder.chargers[12].cluster_id, 1);
	BOOST_CHECK_EQUAL(builder.chargers[13].cluster_id, 1);
}


BOOST_AUTO_TEST_CASE(dbscan_compress_two_clusters_test) {
	const double diff = 0.000005; //Aprox 55cm
	auto chagers_in_cluster = getChargersInLine(10, diff);
	auto second_cluster1 = Charger(10, util::Coordinate::FromDouble(51 + 20*diff, 10 + 20*diff));
	auto second_cluster2 = Charger(11, util::Coordinate::FromDouble(51 + 20*diff, 10 + 21*diff));
	auto second_cluster3 = Charger(12, util::Coordinate::FromDouble(51 + 20*diff, 10 + 22*diff));
	auto second_cluster4 = Charger(13, util::Coordinate::FromDouble(51 + 21*diff, 10 + 21*diff));

	std::vector<Charger> all_chargers{chagers_in_cluster};
	all_chargers.push_back(second_cluster1);
	all_chargers.push_back(second_cluster2);
	all_chargers.push_back(second_cluster3);
	all_chargers.push_back(second_cluster4);

	ChargerGraphBuilder builder{all_chargers, 2, 1};
	builder.dBScan();
	builder.compressClusters();

	BOOST_CHECK_EQUAL(2, builder.chargers.size());
	BOOST_CHECK_EQUAL(9, builder.chargers[0].charger_in_cluster.size());
	for (size_t i = 0; i < 9; i++) {
		BOOST_CHECK_EQUAL(0, builder.chargers[0].charger_in_cluster[i].cluster_id);
		BOOST_CHECK_EQUAL(builder.chargers.size() + i, builder.chargers[0].charger_in_cluster[i].node_id);
	}

	BOOST_CHECK_EQUAL(3, builder.chargers[1].charger_in_cluster.size());
	for (size_t i = 0; i < 3; i++) {
		BOOST_CHECK_EQUAL(1, builder.chargers[1].charger_in_cluster[i].cluster_id);
		BOOST_CHECK_EQUAL(builder.chargers.size() + builder.chargers[0].charger_in_cluster.size() + i,
						  builder.chargers[1].charger_in_cluster[i].node_id);
	}
}


BOOST_AUTO_TEST_SUITE_END()
