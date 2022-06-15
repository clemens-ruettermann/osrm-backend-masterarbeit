////
//// Created by kenspeckle on 3/26/22.
////
//
//#include <memory>
//#include <filesystem>
//#include "charger_graph/charger_graph.hpp"
//#include "charger_graph/files.hpp"
//#include "charger_graph/charger_graph_builder_config.hpp"
//#include "engine/engine.hpp"
//#include "util/coordinate_calculation.hpp"
//#include "util/timing_util.hpp"
//#include "osrm/osrm.hpp"
//#include "util/json_renderer.hpp"
//#include "../../third_party/threadpool/threadpool.h"
//
//
//using namespace osrm;
//using namespace osrm::engine;
//using namespace osrm::enav;
//using namespace std;
//
//using CH = engine::routing_algorithms::ch::Algorithm;
//
//#define DIJKSTRA_PARALELL
//
//const std::string OSRM_PATH = "/home/kenspeckle/git/data/germany_elevation_double/germany-latest.osrm";
//const std::string CHARGER_GRAPH_PATH = OSRM_PATH + ".charger_graph";
//const std::string CAR_PATH = OSRM_PATH + ".car.properties";
//const std::string OUTPUT_FOLDER = "/tmp2/osrm_1_" + osrm::util::uuid::generate_uuid_v4();
////	auto coords_flensburg = util::Coordinate::FromDouble(9.472474,54.815449);
////	auto coords_freiburg = util::Coordinate::FromDouble(7.82578,47.966333);
//
//
//
//
//
//
//void calculate_final_route(Car &car, vector<Charger> &chargers,
//                           PhantomNode &phantom_node_start, PhantomNode &phantom_node_end, vector<NodeID> &best_path) {
//	TIMER_START(FINAL_ROUTE);
//	EngineConfig config;
//	config.storage_config = {OSRM_PATH};
//	config.algorithm = engine::EngineConfig::Algorithm::CH;
//	config.use_shared_memory = false;
//	std::shared_ptr<engine::Engine<CH>> engine_ = std::make_shared<engine::Engine<CH>>(config);
//
//	vector<util::Coordinate> best_path_coords;
//	best_path_coords.push_back(phantom_node_start.location);
//	for (const auto & it : best_path) {
//		best_path_coords.push_back(chargers[it].coordinate);
//	}
//	best_path_coords.push_back(phantom_node_end.location);
//
//	cout << "Final coords: " << endl;
//	for (const auto & it : best_path_coords) {
//		cout << it.ToString() << endl;
//	}
//	std::vector<std::vector<osrm::engine::guidance::LegGeometry>> routes;
//	auto status = engine_->ViaRouteInternal(best_path_coords, routes);
//	if (status == Status::Error) {
//		cerr << "Unable to build route" << endl;
//		exit(2);
//	}
//
//	std::string geojson_str = routesToGeojson(routes, (double ) car.base_battery_capacity_milli_watt_h);
//	auto geojson_path = "/tmp/test/charger_route_main.geojson";
//	ofstream geojson_out{geojson_path};
//	geojson_out << geojson_str;
//	cout << geojson_path << endl;
//
//
//	TIMER_STOP(FINAL_ROUTE);
//	std::cout << "calculating the final route took " << TIMER_SEC(FINAL_ROUTE) << " seconds" << std::endl;
//}
//
//
//void dump_reachable_chargers(const string &path, const vector<Charger> &all_chargers,
//                             const std::vector<ReachableStartNode> &start_nodes) {
//	ofstream out_from_start{path};
//	out_from_start << "lon,lat" << endl;
//	for (auto &it: start_nodes) {
//		out_from_start << all_chargers[it.id].coordinate.ToString() << endl;
//	}
//}
//
//
//vector<NodeID> calculate_best_path(const std::shared_ptr<Car> & car, vector<Charger> &chargers, ReachableChargers &reachable_chargers) {
//
//	std::shared_ptr<std::vector<ChargerGraphEdge>> edges = std::make_shared<std::vector<ChargerGraphEdge>>();
//	TIMER_START(SETUP_CHARGER_GRAPH);
//
//	files::readEdges(CHARGER_GRAPH_PATH, edges);
//	std::shared_ptr<ChargerGraph> chargerGraph = std::make_shared<ChargerGraph>(car, edges, chargers);
//	TIMER_STOP(SETUP_CHARGER_GRAPH);
//	cout << "Setup of the charger graph took " << TIMER_SEC(SETUP_CHARGER_GRAPH) << "s" << endl;
//
//	TIMER_START(ROUTE);
//	auto result = chargerGraph->shortestPath(reachable_chargers.from_start_reachable_chargers, reachable_chargers.to_end_reachable_chargers);
//	TIMER_STOP(ROUTE);
//	std::cout << "calculating the shortest charger route took " << TIMER_SEC(ROUTE) << " seconds" << std::endl;
//
//
//	std::vector<NodeID> best_path = std::get<0>(result);
//	std::cout << "Nodes: " << std::endl;
//	for (auto & it : best_path) {
//		std::cout << std::to_string(it) << std::endl;
//	}
//
//	std::cout << "Total consumption is: " << std::get<1>(result) << std::endl;
//
//
//
//	ofstream used_chargers_coords{OUTPUT_FOLDER + "/final_used_chagers_coords.csv"};
//	std::vector<util::Coordinate> coords;
//	used_chargers_coords << "lon,lat" << std::endl;
//	for (size_t i = 0; i < best_path.size(); i++) {
//		auto it = best_path[i];
//		coords.emplace_back(chargers[it].coordinate);
//		BOOST_ASSERT(it == chargers[it].node_id);
//		used_chargers_coords << chargers[it].coordinate.ToString() << "," << std::to_string(i) << std::endl;
//	}
//	return best_path;
//}
//
//
//int main(int argc, char **argv) {
//	util::LogPolicy::GetInstance().Unmute();
//	TIMER_START(GLOBAL);
//	TIMER_START(SETUP);
//	if (argc != 5) {
//		cout << "Usage: ./" << argv[0] << " <lon1> <lat1> <lon2> <lat2>" << endl;
//		return 1;
//	}
//	filesystem::path p{};
//
//	if (filesystem::exists(OUTPUT_FOLDER)) {
//		throw runtime_error{"Folder already exists"};
//	} else {
//		cout << "Create folder " << OUTPUT_FOLDER << endl;
//		filesystem::create_directory(OUTPUT_FOLDER);
//	}
//
//	auto lon1 = std::stod(argv[1]);
//	auto lat1 = std::stod(argv[2]);
//	auto lon2 = std::stod(argv[3]);
//	auto lat2 = std::stod(argv[4]);
//
//	auto coords_start = util::Coordinate::FromDouble(lon1, lat1);
//	auto coords_end = util::Coordinate::FromDouble(lon2, lat2);
//
//	Car car;
//	Car::read_from_file(car, CAR_PATH);
//	auto car_shared_ptr = make_shared<Car>(car);
//	auto upper_capacity_limit = car.base_battery_capacity_milli_watt_h * 0.9;// * 0.75;
//	auto lower_capacity_limit = car.base_battery_capacity_milli_watt_h * 0.5;// * 0.5;
//
//	std::vector<Charger> chargers;
//	files::readChargers(CHARGER_GRAPH_PATH, chargers);
//
//
//	PhantomNode phantom_node_start, phantom_node_end;
//	{
//		ChargerGraphBuilderConfig config;
//		config.base_path = OSRM_PATH;
//		config.engine_config.storage_config = {config.base_path};
//		config.engine_config.algorithm = engine::EngineConfig::Algorithm::CH;
//		config.engine_config.use_shared_memory = false;
//		std::shared_ptr<engine::Engine<CH>> engine_ = std::make_shared<engine::Engine<CH>>(config.engine_config);
//		phantom_node_start = engine_->GetSnappedPhantomNode(coords_start);
//		phantom_node_end = engine_->GetSnappedPhantomNode(coords_end);
//	}
//
//
//	TIMER_STOP(SETUP);
//	cout << "Setup took " << TIMER_SEC(SETUP) << "s" << endl;
//	auto reachable_chargers = get_reachable_chargers(chargers, phantom_node_start, phantom_node_end, upper_capacity_limit, lower_capacity_limit);
//	auto best_path = calculate_best_path(car_shared_ptr, chargers, reachable_chargers);
//	calculate_final_route(car, chargers, phantom_node_start, phantom_node_end, best_path);
//
//	TIMER_STOP(GLOBAL);
//	util::Log(logINFO) << "All in all this took " << TIMER_SEC(GLOBAL) << "s" << endl;
//
//}