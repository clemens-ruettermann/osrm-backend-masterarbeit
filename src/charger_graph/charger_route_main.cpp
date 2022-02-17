//
// Created by kenspeckle on 3/26/22.
//

#include <memory>
#include <filesystem>
#include "charger_graph/charger_graph.hpp"
#include "charger_graph/files.hpp"
#include "charger_graph/charger_graph_builder_config.hpp"
#include "engine/engine.hpp"
#include "util/coordinate_calculation.hpp"
#include "util/timing_util.hpp"
#include "osrm/osrm.hpp"
#include "util/json_renderer.hpp"
#include "../../third_party/threadpool/threadpool.h"


using namespace osrm;
using namespace osrm::engine;
using namespace osrm::enav;
using namespace std;

using CH = engine::routing_algorithms::ch::Algorithm;

#define DIJKSTRA_PARALELL

const std::string OSRM_PATH = "/home/kenspeckle/git/data/germany_elevation_double/germany-latest.osrm";
const std::string CHARGER_GRAPH_PATH = OSRM_PATH + ".charger_graph";
const std::string CAR_PATH = OSRM_PATH + ".car.properties";
const std::string OUTPUT_FOLDER = "/tmp2/osrm_1_" + osrm::util::uuid::generate_uuid_v4();


int main(int argc, char **argv) {
	util::LogPolicy::GetInstance().Unmute();
	TIMER_START(GLOBAL);
	TIMER_START(SETUP);
	if (argc != 5) {
		cout << "Usage: ./" << argv[0] << " <lon1> <lat1> <lon2> <lat2>" << endl;
		return 1;
	}
	filesystem::path p{};

	if (filesystem::exists(OUTPUT_FOLDER)) {
		throw runtime_error{"Folder already exists"};
	} else {
		cout << "Create folder " << OUTPUT_FOLDER << endl;
		filesystem::create_directory(OUTPUT_FOLDER);
	}

	auto lon1 = std::stod(argv[1]);
	auto lat1 = std::stod(argv[2]);
	auto lon2 = std::stod(argv[3]);
	auto lat2 = std::stod(argv[4]);

	auto coords_start = util::Coordinate::FromDouble(lon1, lat1);
	auto coords_end = util::Coordinate::FromDouble(lon2, lat2);

	Car car{CAR_PATH};
	auto car_shared_ptr = make_shared<Car>(car);
	auto upper_capacity_limit = car.base_battery_capacity_milli_watt_h * 0.9;// * 0.75;
	auto lower_capacity_limit = car.base_battery_capacity_milli_watt_h * 0.5;// * 0.5;

	std::vector<Charger> chargers;
	files::readChargers(CHARGER_GRAPH_PATH, chargers);


	PhantomNode phantom_node_start, phantom_node_end;
	{
		ChargerGraphBuilderConfig config;
		config.base_path = OSRM_PATH;
		config.engine_config.storage_config = {config.base_path};
		config.engine_config.algorithm = engine::EngineConfig::Algorithm::CH;
		config.engine_config.use_shared_memory = false;
		std::shared_ptr<engine::Engine<CH>> engine_ = std::make_shared<engine::Engine<CH>>(config.engine_config);
		phantom_node_start = engine_->GetSnappedPhantomNode(coords_start);
		phantom_node_end = engine_->GetSnappedPhantomNode(coords_end);
	}


	TIMER_STOP(SETUP);
	cout << "Setup took " << TIMER_SEC(SETUP) << "s" << endl;

	TIMER_STOP(GLOBAL);
	util::Log(logINFO) << "All in all this took " << TIMER_SEC(GLOBAL) << "s" << endl;

}