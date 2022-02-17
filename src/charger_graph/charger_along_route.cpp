////
//// Created by kenspeckle on 4/19/22.
////
//#include <iostream>
//#include <execution>
//#include "charger_graph/charger_graph_builder_config.hpp"
//#include "osrm/osrm.hpp"
//#include "engine/engine.hpp"
//#include "charger_graph/charger_graph.hpp"
//#include "charger_graph/files.hpp"
//#include "util/timing_util.hpp"
//#include "util/meminfo.hpp"
//
//using namespace std;
//using namespace osrm::enav;
//using namespace osrm::engine;
//using namespace osrm;
//using CH = engine::routing_algorithms::ch::Algorithm;
//
////#define DIJKSTRA_PARALELL
//
//
//static std::string routesToGeojson(const std::vector<std::vector<osrm::engine::guidance::LegGeometry>> & routes, const double battery_capacity_in_milli_wh) {
//	RouteConsumption complete_consumption = 0;
//	std::stringstream geojson_string_stream;
//	geojson_string_stream << R"({)";
//	for (size_t k = 0; k < routes.size(); k++) {
//		const auto & route = routes[k];
//		geojson_string_stream << R"(
//	"type": "FeatureCollection",
//	"features": [)";
//		for (size_t i = 0; i < route.size(); i++) {
//			auto & subroute = route[i];
//			RouteConsumption battery_cap = battery_capacity_in_milli_wh;
//			double distance_of_subroute = 0;
//			BOOST_ASSERT(subroute.locations.size() == subroute.annotations.size() + 1);
//			for (size_t j = 0; j < subroute.annotations.size(); j++) {
//				distance_of_subroute += subroute.annotations[j].distance;
//
//
//				geojson_string_stream << R"(
//	{
//		"type": "Feature",
//		"geometry": {
//			"type": "LineString",
//			"coordinates": [)";
//				auto begin = subroute.locations[j];
//				geojson_string_stream << "[" << begin.ToString() << "], ";
//				auto end = subroute.locations[j+1];
//				geojson_string_stream << "[" << end.ToString() << "]";
//				geojson_string_stream << R"(]
//		},)";
//				geojson_string_stream << R"(
//		"properties": {
//			"consumption": )";
//				battery_cap -= subroute.annotations[j].consumption;
//				complete_consumption += subroute.annotations[j].consumption;
//				auto percentage = ((double)battery_cap / battery_capacity_in_milli_wh) * 100.0;
//				geojson_string_stream << to_string(percentage);
//				geojson_string_stream << R"(
//		}
//	})";
//
//				if (i != route.size() - 1 || j != subroute.annotations.size() - 1) {
//					geojson_string_stream << ",";
//				}
//			}
//			std::cout << "Distance of subroute is: " << distance_of_subroute << "\t Consumption is: " << battery_cap << std::endl;
//		}
//		geojson_string_stream << "]" << endl;
//		if (k != routes.size() - 1) {
//			geojson_string_stream << ",";
//		}
//	}
//
//	geojson_string_stream << R"(})";
//
//	cout << "Complete consumption: " << complete_consumption << endl;
//
//	return geojson_string_stream.str();
//}
//
//
//
//
//
//
//void print_usage(int argc, char ** argv) {
//	util::Log(logINFO) << "Usage: " << endl;
//	if (argc >= 1) {
//		util::Log(logINFO) << "." << argv[0];
//	} else {
//		util::Log(logINFO) << "./charger_along_route";
//	}
//	util::Log(logINFO) << " <Path to OSRM files> <lon1> <lat1> <lon2> <lat2>";
//}
//
//int main(int argc, char ** argv) {
//	if (argc != 6) {
//		print_usage(argc, argv);
//		return 1;
//	}
//
//	const string osrm_path = string{argv[1]};
//	const string lon1{argv[2]};
//	const string lat1{argv[3]};
//	const string lon2{argv[4]};
//	const string lat2{argv[5]};
//	util::LogPolicy::GetInstance().Unmute();
//	TIMER_START(GLOBAL);
//	util::LogPolicy::GetInstance().Unmute();
//
//	ChargerGraphBuilderConfig config;
//	config.base_path = osrm_path;
//	config.engine_config.storage_config = {config.base_path};
//	config.engine_config.algorithm = engine::EngineConfig::Algorithm::CH;
//	config.engine_config.use_shared_memory = false;
//	std::shared_ptr<engine::Engine<CH>> engine_ = std::make_shared<engine::Engine<CH>>(config.engine_config);
//
//	std::vector<Charger> chargers;
//	files::readChargers(osrm_path + ".charger_graph", chargers);
//
//	double distance_between_query_points = 10000;
//	double distance_between_query_points_half = distance_between_query_points/2;
//	auto coords_start = util::Coordinate::FromDouble(std::stof(lon1) , std::stof(lat1));
//	auto coords_end = util::Coordinate::FromDouble(std::stof(lon2), std::stof(lat2));
//
//	std::vector<std::vector<engine::guidance::LegGeometry>>  first_route_result;
//	auto status = engine_->ViaRouteInternal({coords_start, coords_end}, first_route_result);
//
//	if (status == Status::Error) {
//		cerr << "Could not calculate route" << endl;
//		return 2;
//	}
//	BOOST_ASSERT(first_route_result.size() == 1);
//	auto first_route = first_route_result[0];
//
//
//	TIMER_START(FILTER_COORDS);
//	vector<util::Coordinate> all_coords;
//	for (const auto & leg : first_route) {
//		for (const auto & coord : leg.locations) {
//			auto no_overlapping = std::none_of(all_coords.cbegin(), all_coords.cend(), [&coord, &distance_between_query_points_half](const util::Coordinate & existing_coord) {
//				return util::coordinate_calculation::haversineDistance(coord, existing_coord) < distance_between_query_points_half;
//			});
//			if (no_overlapping) {
//				all_coords.push_back(coord);
//			}
//		}
//	}
//	TIMER_STOP(FILTER_COORDS);
//	util::Log(logINFO) << "Filtering the coords took " << TIMER_SEC(FILTER_COORDS) << "s";
//	util::Log(logINFO) << "There are " << all_coords.size() << " points at which we will search for chargers";
//
//	TIMER_START(FILTER_CHARGERS);
//	vector<Charger> possible_chargers;
//	std::copy_if(chargers.begin(), chargers.end(), std::back_inserter(possible_chargers), [&all_coords, &distance_between_query_points] (const Charger & charger) {
//		return std::any_of(all_coords.begin(), all_coords.end(), [&charger, &distance_between_query_points](const util::Coordinate & coord) {
//			return util::coordinate_calculation::haversineDistance(charger.coordinate, coord) < distance_between_query_points;
//		});
//	});
//	TIMER_STOP(FILTER_CHARGERS);
//	util::Log(logINFO) << "Filtering the chargers took " << TIMER_SEC(FILTER_CHARGERS) << "s";
//	util::Log(logINFO) << "Possible chargers: " << possible_chargers.size();
//
//
//
//
//	Car car;
//	Car::read_from_file(car, osrm_path + ".car.properties");
//	auto car_shared_ptr = make_shared<Car>(car);
//
//	vector<ChargerId> possible_charger_ids;
//	transform(possible_chargers.cbegin(), possible_chargers.cend(), std::back_inserter(possible_charger_ids), [](const Charger & c) {return c.node_id;});
//
//	std::shared_ptr<std::vector<ChargerGraphEdge>> edges = std::make_shared<std::vector<ChargerGraphEdge>>();
//	files::readEdges(osrm_path + ".charger_graph", edges);
//
//
//	auto upper_capacity_limit = car.base_battery_capacity_milli_watt_h * 0.9;// * 0.75;
//	auto lower_capacity_limit = car.base_battery_capacity_milli_watt_h * 0.5;// * 0.5;
//
//	TIMER_START(GRAPH_SHRINKING);
//	auto new_edge_end_it = remove_if(
//			std::execution::par_unseq,
//			edges->begin(), edges->end(), [&](const ChargerGraphEdge & e){
//		if (e.consumption < lower_capacity_limit || e.consumption > upper_capacity_limit) {
//			return true;
//		}
//		auto start_it = find(possible_charger_ids.cbegin(), possible_charger_ids.cend(), e.start);
//		if (start_it != possible_charger_ids.cend()) {
//			auto end_it = find(possible_charger_ids.cbegin(), possible_charger_ids.cend(), e.end);
//			return end_it == possible_charger_ids.cend();
//		}
//		return true;
//	});
//	TIMER_STOP(GRAPH_SHRINKING);
//	util::Log(logINFO) << "Graph shrinking took " << TIMER_SEC(GRAPH_SHRINKING) << "secs";
//	auto dist = distance(new_edge_end_it, edges->end());
//	edges->erase(new_edge_end_it, edges->end());
//	util::Log(logINFO) << "Removed edges " << dist << " with start/end node filter. Now there are " << edges->size() << " left";
//
//
//
//	std::shared_ptr<ChargerGraph> chargerGraph = std::make_shared<ChargerGraph>(car_shared_ptr, edges, possible_chargers);
//	PhantomNode phantom_node_start, phantom_node_end;
//
//
//	// from start reachable chargers
//	std::vector<ReachableStartNode> from_start_reachable_chargers;
//	{
//		std::vector<PhantomNode> all_nodes;
//		for (const auto &it: possible_chargers) {
//			all_nodes.push_back(it.phantom_node);
//		}
//		phantom_node_start = engine_->GetSnappedPhantomNode(coords_start);
//
//		all_nodes.push_back(phantom_node_start);
//		auto result = engine_->ManyToManyInternal(all_nodes, {all_nodes.size() - 1}, {}, false);
//		auto consumptions = std::get<2>(result);
//		auto weights = std::get<3>(result);
//		for (size_t i = 0; i < all_nodes.size() - 1; i++) {
//			if (car.base_battery_capacity_milli_watt_h >= consumptions[i] && 0 <= consumptions[i]) {
//				from_start_reachable_chargers.emplace_back(chargers[i], consumptions[i], weights[i]);
//			}
//		}
//		util::Log(logINFO) << "Got " << from_start_reachable_chargers.size() << " from start reachable chargers";
//	}
//
//
//
//	// to end reachable chargers
//	std::vector<ReachableEndNode> to_end_reachable_chargers;
//	{
//		std::vector<PhantomNode> all_nodes;
//		for (const auto &it: possible_chargers) {
//			all_nodes.push_back(it.phantom_node);
//		}
//		phantom_node_end = engine_->GetSnappedPhantomNode(coords_end);
//		all_nodes.push_back(phantom_node_end);
//
//		auto result = engine_->ManyToManyInternal(all_nodes, {}, {all_nodes.size() - 1}, false);
//		auto consumptions = std::get<2>(result);
//		auto weights = std::get<3>(result);
//		for (size_t i = 0; i < all_nodes.size() - 1; i++) {
//			if (car.base_battery_capacity_milli_watt_h >= consumptions[i] && 0 <= consumptions[i]) {
//				to_end_reachable_chargers.emplace_back(chargers[i].node_id, consumptions[i], weights[i]);
//			}
//		}
//		util::Log(logINFO) << "Got " << to_end_reachable_chargers.size() << " to end reachable chargers";
//	}
//
//
//	// best path
//	auto best_path_result = chargerGraph->shortestPath(from_start_reachable_chargers, to_end_reachable_chargers);
//	std::vector<NodeID> best_path = std::get<0>(best_path_result);
//	util::Log(logINFO) << "Nodes: ";
//	for (auto & it : best_path) {
//		util::Log(logINFO) << std::to_string(it) << std::endl;
//	}
//
//	ofstream used_chargers_coords{"/tmp/test/final_used_chagers_coords.csv"};
//	std::vector<util::Coordinate> coords;
//	used_chargers_coords << "lon,lat" << std::endl;
//	for (size_t i = 0; i < best_path.size(); i++) {
//		auto it = best_path[i];
//		coords.emplace_back(chargerGraph->charger_list[it].coordinate);
//		BOOST_ASSERT(it == chargerGraph->charger_list[it].node_id);
//		BOOST_ASSERT(it < possible_charger_ids.size());
//		used_chargers_coords << chargerGraph->charger_list[it].coordinate.ToString() << "," << std::to_string(i) << std::endl;
//
////		std::cout << chargerGraph->charger_list[it].coordinate.ToString() << "," << std::to_string(i) << std::endl;
//	}
//
//
//	// final route
//	vector<util::Coordinate> best_path_coords;
//	best_path_coords.push_back(phantom_node_start.location);
//	for (const auto & it : best_path) {
//		best_path_coords.push_back(chargerGraph->charger_list[it].coordinate);
//	}
//	best_path_coords.push_back(phantom_node_end.location);
//
//	util::Log(logINFO) << "Final coords: ";
//	for (const auto & it : best_path_coords) {
//		std::cout << it.ToString() << std::endl;
//	}
//	std::vector<std::vector<osrm::engine::guidance::LegGeometry>> routes;
//	auto final_status = engine_->ViaRouteInternal(best_path_coords, routes);
//	if (final_status == Status::Error) {
//		cerr << "Unable to build route" << endl;
//		exit(2);
//	}
//
//	{
//		auto geojson_str = routesToGeojson(routes, car.base_battery_capacity_milli_watt_h);
//		ofstream geojson_out{"/tmp/test/charger_along_route.geojson"};
//		geojson_out << geojson_str;
//		cout << "/tmp/test/charger_along_route.geojson" << endl;
//	}
//
//	TIMER_STOP(GLOBAL);
//	util::Log(logINFO) << "All in all this took " << TIMER_SEC(GLOBAL) << "s";
//
//}