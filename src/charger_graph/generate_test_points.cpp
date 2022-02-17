//
// Created by kenspeckle on 4/14/22.
//

#include "charger_graph/charger_graph_builder_config.hpp"
#include "osrm/osrm.hpp"
#include "engine/engine.hpp"
#include "charger_graph/charger_graph.hpp"
#include "charger_graph/files.hpp"
#include "util/timing_util.hpp"
#include "util/meminfo.hpp"

using namespace osrm::enav;
using namespace osrm::engine;
using namespace osrm;
using namespace std;

using CH = engine::routing_algorithms::ch::Algorithm;
using namespace util::coordinate_calculation;

//const std::string OUTPUT_FOLDER = "/tmp2/osrm_1_" + osrm::util::uuid::generate_uuid_v4();

bool checkIfLastPointIsReachable(const shared_ptr<engine::Engine<CH>> &engine_,
                                 const std::vector<PhantomNodePair> &generated_points, const std::vector<PhantomNodePair> & charger_points,
                                 const PhantomNodePair & new_point);


util::Coordinate generateNewPoint() {
	static std::random_device rd;
	static std::mt19937 g(rd());

	//5°52′01″ - 15°02′37″
	static std::uniform_real_distribution<double> dist_lon(5.866944444444445,15.043611111111112);

	//47°16′15″ - 55°03′33″
	static std::uniform_real_distribution<double> dist_lat(47.2708333,55.0591667);

	auto lon = dist_lon(g);
	auto lat = dist_lat(g);

	return util::Coordinate::FromDouble(lon, lat);
}


void print_usage(int argc, char ** argv) {
	cout << "Usage: " << endl;
	if (argc >= 1) {
		cout << "./" << argv[0];
	} else {
		cout << "./generate_test_points";
	}
	cout << " <Path to OSRM files> <Output path for test points> [--extend-existsing]" << endl;
}


int main(int argc, char ** argv) {
	if (argc < 3) {
		print_usage(argc, argv);
		return 1;
	}

	string osrm_path = string{argv[1]};
	string output_path = string{argv[2]};

	TIMER_START(ROUTE);
	util::LogPolicy::GetInstance().Unmute();

	ChargerGraphBuilderConfig config;
	config.base_path = osrm_path;
	config.engine_config.storage_config = {config.base_path};
	config.engine_config.algorithm = engine::EngineConfig::Algorithm::CH;
	config.engine_config.use_shared_memory = false;
	std::shared_ptr<engine::Engine<CH>> engine_ = std::make_shared<engine::Engine<CH>>(config.engine_config);

	std::vector<Charger> chargers;
	files::readChargers(osrm_path + ".charger_graph.charger.json", chargers);

	const unsigned int number_points_to_generate = 1000;
	std::vector<PhantomNodePair> chargers_points;
	for (auto & it : chargers) {
		chargers_points.push_back(it.phantom_node_pair);
	}

	std::vector<engine::PhantomNodePair> generated_points;
	std::vector<engine::PhantomNode> snapped_generated_points;
	unsigned int i = 0;
	if (argc == 4 && std::string{argv[3]} == "--extend-existsing") {

		std::ifstream in{output_path};
		std::string line, lon_str, lat_str;
		std::getline(in, line);
		while (std::getline(in, line)) {
			stringstream ss(line);
			getline(ss, lon_str, ',');
			getline(ss, lat_str, ',');
			auto tmp_phantom_node_pair = engine_->GetPhantomNodePair(
					{util::Coordinate::FromDouble(std::stod(lon_str), std::stod(lat_str))});
			auto tmp_snapped_phantom_node = engine_->SnapPhantomNodes({tmp_phantom_node_pair});
			BOOST_ASSERT(tmp_snapped_phantom_node.size() == 1);
			generated_points.emplace_back(tmp_phantom_node_pair);
			snapped_generated_points.emplace_back(tmp_snapped_phantom_node.at(0));
		}
		i = generated_points.size();
		util::Log() << "Using existing list: " << std::to_string(i) << " points already existing";
	}

	util::Log() << "Setup done. Starting to generate random points";



	unsigned int tmp_generated_counter = 0;
	auto time_start = std::chrono::steady_clock::now();
	while (i < number_points_to_generate) {
		std::cout << "tmp_generated_points: " << tmp_generated_counter << std::endl;
		auto new_point = generateNewPoint();
		auto new_phantom_node_pair = engine_->GetPhantomNodePair(new_point);

		if (haversineDistance(new_point, new_phantom_node_pair.first.location) > 1000
				|| haversineDistance(new_point, new_phantom_node_pair.second.location) > 1000) {
			tmp_generated_counter++;
			continue;
		}

		bool same_as_existing = std::any_of(snapped_generated_points.cbegin(), snapped_generated_points.cend(), [&new_phantom_node_pair](const PhantomNode & existing_coord) {
			return
					(existing_coord.location == new_phantom_node_pair.first.location || existing_coord.location == new_phantom_node_pair.second.location)
					|| haversineDistance(existing_coord.location, new_phantom_node_pair.first.location) < 10000
					|| haversineDistance(existing_coord.location, new_phantom_node_pair.second.location) < 10000;
		});
		if (same_as_existing) {
//			std::cerr << "A generated point with these coordinates already exists or is less than 10km from an existing point" << endl;
			tmp_generated_counter++;
			continue;
		}

		if (checkIfLastPointIsReachable(engine_, generated_points, chargers_points, new_phantom_node_pair)) {
			std::cout << "Found a new and valid point" << std::endl;
			auto snapped_point = engine_->SnapPhantomNodes({new_phantom_node_pair});
			if (snapped_point.size() != 1 || !snapped_point.at(0).IsValid()){
				continue;
			}
			generated_points.emplace_back(new_phantom_node_pair);
			snapped_generated_points.emplace_back(snapped_point.at(0));

		} else {
			tmp_generated_counter++;
			continue;
		}
		util::Log() << (i+1) << "/" << number_points_to_generate << "\t" << new_point.ToString()
			<< "\t This took "
			<< (0.000001 * std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - time_start).count()) << "secs"
			<< "\tand " << tmp_generated_counter << " random points were generated" << std::endl;
		time_start = std::chrono::steady_clock::now();
		i++;
		tmp_generated_counter = 0;

		{
			std::cerr << "Writing coord to " << output_path << std::endl;
			ofstream out{output_path, std::ofstream::trunc};
			out << "lon,lat" << endl;

			for (const auto & it : snapped_generated_points) {
				out << it.location.ToString() << endl;
			}

		}
	}

	ofstream out{output_path, std::ofstream::trunc};
	out << "lon,lat" << endl;

	for (const auto & it : snapped_generated_points) {
		out << it.location.ToString() << endl;
	}

	TIMER_STOP(ROUTE);
	util::Log() << "generating" << to_string(number_points_to_generate) << " random points took " << TIMER_SEC(ROUTE) << " seconds" << std::endl;
	util::DumpMemoryStats();
}

bool checkIfLastPointIsReachable(const shared_ptr<engine::Engine<CH>> &engine_,
                                 const std::vector<PhantomNodePair> &generated_points, const std::vector<PhantomNodePair> & charger_points,
                                 const PhantomNodePair & new_point) {
	std::vector<PhantomNodePair> all_points;
	all_points.emplace_back(new_point);
	std::copy(generated_points.begin(), generated_points.end(), std::back_inserter(all_points));
	std::copy(charger_points.begin(), charger_points.end(), std::back_inserter(all_points));
	{

		auto result = engine_->ManyToManyInternal(all_points, {0}, {}, true);
		auto weights = get<3>(result);
		auto distances = get<1>(result);

		// It is nessecary to be able to route to all other generated points and to at least one charger
		for (size_t i = 1; i < generated_points.size() + 1; i++) {
			if (weights[i] == INVALID_EDGE_WEIGHT) {
				return false;
			}
		}
		bool found = false;
		for (size_t i = 1 + generated_points.size(); i < generated_points.size() + charger_points.size() + 1; i++) {
			if (weights[i] == INVALID_EDGE_WEIGHT) {
				found = true;
				break;
			}
		}
		if (!found) {
			return false;
		}
	}
	{
		auto result = engine_->ManyToManyInternal(all_points, {}, {0}, true);
		auto weights = get<3>(result);
		auto distances = get<1>(result);

		// It is nessecary to be able to route to all other generated points and to at least one charger
		for (size_t i = 1; i < generated_points.size() + 1; i++) {
			if (weights[i] == INVALID_EDGE_WEIGHT) {
				return false;
			}
		}
		bool found = false;
		for (size_t i = 1 + generated_points.size(); i < generated_points.size() + charger_points.size() + 1; i++) {
			if (weights[i] == INVALID_EDGE_WEIGHT) {
				found = true;
				break;
			}
		}
		if (!found) {
			return false;
		}
	}
	return true;
}
