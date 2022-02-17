//
// Created by kenspeckle on 4/9/22.
//

#include <memory>
#include <threadpool.h>
#include <mutex>
#include "elevation/geotiff_value_matrix.h"
#include "util/car.hpp"
#include "charger_graph/charger_graph_edge.hpp"
#include "util/string_util.hpp"
#include "charger_graph/charger.hpp"
#include "charger_graph/charger_graph.hpp"
#include "charger_graph/files.hpp"
#include "util/timing_util.hpp"
#include "util/meminfo.hpp"
#include "osrm/osrm.hpp"
#include "util/json_renderer.hpp"

using namespace osrm;
using namespace osrm::enav;
const std::string TOWER_PREFIX = "/home/kenspeckle/git/";
const std::string LAPTOP_PREFIX = "/home/kenspeckle/git/fh-aachen/masterarbeit/";
const std::string OSRM_PATH_ID3 = "data/osrm_data_id3_60_90/germany-220519.osrm";
const std::string OSRM_PATH_CAR_INDEPENDENT = "data/osrm_data_split_consumption/germany-220519.osrm";



std::string json_to_string(const util::json::Object & obj) {
	std::stringstream ss;
	util::json::render(ss, obj);
	return ss.str();
}


void performance_test_along(
		OSRM & osrm,
		std::vector<std::pair<util::Coordinate, util::Coordinate>> & test_point_pairs,
		const std::string & output_path,
		const size_t num_points) {
	util::Log() << "[performance Test] Starting AlongRoute";
	// Along 100 Routes
	{
		std::string path = output_path + "along_route_" + std::to_string(num_points) + ".txt";
		std::ofstream out{path};
		std::cerr << "Writing to " << path << std::endl;
		size_t sum_time = 0;
		util::json::Object j_res;
		for (size_t i = 0; i < num_points; i++) {
			const auto & it = test_point_pairs[i];
			auto start = std::chrono::high_resolution_clock::now();
			engine::api::EVRouteParameters parameters;
			parameters.lower_capacity_limit_percent = 0;
			parameters.upper_capacity_limit_percent = 100;
			parameters.start = it.first;
			parameters.end = it.second;
			parameters.output_format = osrm::engine::api::EVRouteParameters::OutputFormat::OSRM;
			osrm.EVRouteAlongRoute(parameters, j_res);
			auto end = std::chrono::high_resolution_clock::now();
			auto diff = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()/1000.0;
			out << i << ": " << diff << "ms" << std::endl;
//			if (i % 100 == 0) {
				util::Log(logINFO) << i << ": " << diff << "ms";
//			}

			sum_time += diff;
		}

		out << "==================================" << std::endl;
		out << "This took an avg of " << (sum_time/(double)test_point_pairs.size()) << " ms per route" << std::endl;
	}
	util::Log() << "[performance Test] Done AlongRoute";
}

void performance_test_dijkstra_along(
	OSRM & osrm,
	std::vector<std::pair<util::Coordinate, util::Coordinate>> & test_point_pairs,
	const std::string & output_path,
	const double lower_limit_percent,
	const double upper_limit_percent,
	const size_t num_points) {
	util::Log() << "[performance Test] Starting AlongRouteDijkstra";
	// Dijkstra Along 100 Routes
	{
		std::string path = output_path + "dijstra_along_" + std::to_string(num_points) + "_points_" + std::to_string(lower_limit_percent) + "-" + std::to_string(upper_limit_percent) + ".txt";
		std::cout << "Writing to " << path << std::endl;
		std::ofstream out{path};
		size_t sum_time = 0;
		util::json::Object j_res;
		for (size_t i = 0; i < num_points; i++) {
			const auto & it = test_point_pairs[i];
			auto start = std::chrono::high_resolution_clock::now();
			engine::api::EVRouteParameters parameters;
			parameters.lower_capacity_limit_percent = lower_limit_percent;
			parameters.upper_capacity_limit_percent = upper_limit_percent;
			parameters.start = it.first;
			parameters.end = it.second;
			parameters.output_format = osrm::engine::api::EVRouteParameters::OutputFormat::OSRM;
			osrm.EVRouteDijkstraAlongRoute(parameters, j_res);
			auto end = std::chrono::high_resolution_clock::now();
			auto diff = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()/1000.0;
			out << i << ": " << diff << "ms" << std::endl;
			util::Log(logINFO) << i << ": " << diff << "ms";

			sum_time += diff;
		}

		out << "==================================" << std::endl;
		out << "This took an avg of " << (sum_time/(double)test_point_pairs.size()) << " ms per route" << std::endl;

	}

	util::Log() << "[performance Test] Done AlongRouteDijkstra";
}

void performance_test_dijkstra(
	OSRM & osrm,
	std::vector<std::pair<util::Coordinate, util::Coordinate>> & test_point_pairs,
	const std::string & output_path,
	const double lower_limit_percent,
	const double upper_limit_percent,
	const size_t num_points) {
	util::Log() << "[performance Test] Starting Dijkstra";
	// Dijkstra 100 Routes
	{
		std::string path = output_path + "dijstra_" + std::to_string(num_points) +"_points_" + std::to_string(lower_limit_percent) + "-" + std::to_string(upper_limit_percent) + "percent.txt";
		std::cout << "Writing to " << path << std::endl;
		std::ofstream out{path};
		util::json::Object j_res;
		size_t sum_time = 0;
		for (size_t i = 0; i < num_points; i++) {

			const auto & it = test_point_pairs[i];
			auto start = std::chrono::high_resolution_clock::now();
			engine::api::EVRouteParameters parameters;
			parameters.lower_capacity_limit_percent = lower_limit_percent;
			parameters.upper_capacity_limit_percent = upper_limit_percent;
			parameters.start = it.first;
			parameters.end = it.second;
			parameters.output_format = osrm::engine::api::EVRouteParameters::OutputFormat::OSRM;
			osrm.EVRouteDijkstra(parameters, j_res);
			auto end = std::chrono::high_resolution_clock::now();
			auto diff = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()/1000.0;
			out << i << ": " << diff << "ms" << std::endl;
			util::Log(logINFO) << i << ": " << diff << "ms";
			sum_time += diff;
		}
		out << "==================================" << std::endl;
		out << "This took an avg of " << (sum_time/(double)test_point_pairs.size()) << " ms per route" << std::endl;
	}

	util::Log() << "[performance Test] Done with Dijkstra";
}




//
//void check_grade_of_result(
//	OSRM & osrm,
//	std::vector<std::pair<util::Coordinate, util::Coordinate>> & test_point_pairs,
//	const std::string & output_path,
//	const size_t num_points,
//	const bool full_graph) {
//
//
//	std::string path = output_path + "grade_of_result_" + std::to_string(num_points) + (full_graph?"full_graph":"sub_graph") + ".txt";
//	std::ofstream out{path};
//	out << "Dijkstra-Weight,Dijkstra-Consumption,Along-Dijkstra-Weight,Along-Dijkstra-Consumption,Along-Weight,Along-Consumption" << std::endl;
//	std::cout << "Writing to " << path << std::endl;
//
//	double diff_percent_weight_along_route = 0;
//	double diff_percent_consumption_along_route = 0;
//	double diff_percent_weight_dijkstra_along_route = 0;
//	double diff_percent_consumption_dijkstra_along_route = 0;
//
//	size_t counter = 0;
//	for (size_t i = 0; counter < 100; i++) {
//		auto & it = test_point_pairs[i];
//
//		engine::api::EVRouteParameters parameters;
//		parameters.lower_capacity_limit_percent = 60;
//		parameters.upper_capacity_limit_percent = 90;
//		parameters.start = it.first;
//		parameters.end = it.second;
//		parameters.output_format = osrm::engine::api::EVRouteParameters::OutputFormat::IP_FRONTEND;
//
//
//		util::json::Object res_along{};
//		auto status_along = osrm.EVRouteAlongRoute(parameters, res_along);
//		if (status_along == osrm::engine::Status::Error) {
//			std::cerr << "Unable to calc along route" << std::endl;
//			out << "Unable to route" << std::endl;
//			continue;
//		}
//
//
//		util::json::Object res_dijkstra{};
//		auto status_dijkstra = osrm.EVRouteDijkstra(parameters, res_dijkstra);
//		if (status_dijkstra == osrm::engine::Status::Error) {
//			std::cerr << "Unable to calc dijkstra route" << std::endl << std::endl;
////			out << "Unable to route" << std::endl;
//			continue;
//		}
//
//		auto ref_weight = res_dijkstra.values["weight"].get<util::json::Number>().value;
//		auto ref_consumption = res_dijkstra.values["consumption"].get<util::json::Number>().value;
//
//		util::json::Object res_dijkstra_along{};
//		auto status_dijkstra_along = osrm.EVRouteAlongRouteDijkstra(parameters, res_dijkstra_along);
//		if (status_dijkstra_along == osrm::engine::Status::Error) {
//			std::cerr << "Unable to calc dijkstra along route" << std::endl;
//			out << "Unable to route" << std::endl;
//			continue;
//		}
//		auto weight_dijkstra_along = res_dijkstra_along.values["weight"].get<util::json::Number>().value;
//		auto consumption_dijkstra_along = res_dijkstra_along.values["consumption"].get<util::json::Number>().value;
//
//		auto weight_diff_dijkstra_along = std::abs(ref_weight - weight_dijkstra_along);
//		diff_percent_weight_dijkstra_along_route += weight_diff_dijkstra_along/ref_weight;
//
//		auto consumption_diff_dijkstra_along = std::abs(ref_consumption - consumption_dijkstra_along);
//		diff_percent_consumption_dijkstra_along_route += consumption_diff_dijkstra_along/ref_consumption;
//
//
//
//
//
//
//		auto weight_along = res_along.values["weight"].get<util::json::Number>().value;
//		auto consumption_along = res_along.values["consumption"].get<util::json::Number>().value;
//
//		auto weight_diff_along = std::abs(ref_weight - weight_along);
//		diff_percent_weight_along_route += weight_diff_along/ref_weight;
//
//		auto consumption_diff_along = std::abs(ref_consumption - consumption_along);
//		diff_percent_consumption_along_route += consumption_diff_along/ref_consumption;
//
//
//		std::string current_line =
//			  "Dijkstra-Weight:                       " + std::to_string(ref_weight)
//			+ "Dijkstra-Along-Weight:                 " + std::to_string(weight_dijkstra_along)
//			+ "Along-Weight:                          " + std::to_string(weight_diff_along)
//			+ "Dijkstra-Consumption:                  " + std::to_string(ref_consumption)
//			+ "Dijkstra-Along-Consumption:            " + std::to_string(consumption_dijkstra_along)
//			+ "Along-Consumption:                     " + std::to_string(consumption_diff_along)
//			+ "Dijkstra-Along-Weight-Percent:         " + std::to_string(weight_diff_dijkstra_along/ref_weight)
//			+ "Dijkstra-Along-Consumption-Percent:    " + std::to_string(consumption_diff_dijkstra_along/ref_consumption)
//			+ "Along-Weight-Percent:                  " + std::to_string(weight_diff_along/ref_weight)
//			+ "Along-Consumption-Percent:             " + std::to_string(consumption_diff_along/ref_consumption) + "\n\n";
//
//		std::cout << current_line << std::endl;
//
////		out << "Dijkstra-Weight,Dijkstra-Consumption,Along-Dijkstra-Weight,Along-Dijkstra-Consumption,Along-Weight,Along-Consumption" << std::endl;
////		out << current_line << std::endl;
//		out
//			<< std::to_string(ref_weight) << "," << std::to_string(ref_consumption) << ","
//			<< std::to_string(weight_dijkstra_along) << "," << std::to_string(consumption_dijkstra_along) << ","
//			<< std::to_string(weight_along) << "," << std::to_string(consumption_along) << std::endl;
//
//		counter++;
//	}
//
//	out << "Diff Weight Percent Along: " << std::to_string(diff_percent_weight_along_route/100.0) << std::endl;
//	out << "Diff Consumption Percent Along: " << std::to_string(diff_percent_consumption_along_route/100.0) << std::endl;
//
//	out << "Diff Weight Percent Dijkstra Along: " << std::to_string(diff_percent_weight_dijkstra_along_route/100.0) << std::endl;
//	out << "Diff Consumption Percent Dijkstra Along: " << std::to_string(diff_percent_consumption_dijkstra_along_route/100.0) << std::endl;
//
//
//
//}
//





void check_grade_of_result_all_options(
	OSRM & osrm_full_graph,
	OSRM & osrm_sub_graph,
	std::vector<std::pair<util::Coordinate, util::Coordinate>> & test_point_pairs,
	const std::string & output_path,
	const size_t num_points,
	const double upper_limit) {


	std::string path = output_path + "grade_of_result_" + std::to_string(num_points) + "_all_options_combined.txt";
	bool need_to_write_header = false;
	if (!std::filesystem::is_regular_file(path)) {
		need_to_write_header = true;
	}
	std::ofstream out{path, std::ios_base::app};
	if (need_to_write_header) {
		out << "Dijkstra-Weight-Full,Dijkstra-Consumption-Full,Along-Dijkstra-Weight-Full,Along-Dijkstra-Consumption-Full,Dijkstra-Weight-Sub,Dijkstra-Consumption-Sub,Along-Dijkstra-Weight-Sub,Along-Dijkstra-Consumption-Sub,Along-Weight,Along-Consumption,lon0,lat0,lon1,lat1" << std::endl;
	}
	std::cout << "Writing to " << path << std::endl;



	double diff_percent_weight_along_route = 0;
	double diff_percent_consumption_along_route = 0;
	double diff_percent_weight_dijkstra_along_route = 0;
	double diff_percent_consumption_dijkstra_along_route = 0;

	size_t counter = 0;
	for (size_t i = 0; counter < num_points; i++) {
		auto & it = test_point_pairs[i];

		engine::api::EVRouteParameters parameters_sub;
		parameters_sub.lower_capacity_limit_percent = 60;
		parameters_sub.upper_capacity_limit_percent = 90;
		parameters_sub.start = it.first;
		parameters_sub.end = it.second;
		parameters_sub.output_format = osrm::engine::api::EVRouteParameters::OutputFormat::IP_FRONTEND;

		engine::api::EVRouteParameters parameters_full;
		parameters_full.lower_capacity_limit_percent = 0;
		parameters_full.upper_capacity_limit_percent = 100;
		parameters_full.start = it.first;
		parameters_full.end = it.second;
		parameters_full.output_format = osrm::engine::api::EVRouteParameters::OutputFormat::IP_FRONTEND;


		// Calc along
		util::json::Object res_along{};
		auto status_along = osrm_full_graph.EVRouteAlongRoute(parameters_sub, res_along);
		if (status_along == osrm::engine::Status::Error) {
			std::cerr << "Unable to calc along route" << std::endl << std::endl;
			continue;
		}
		auto weight_along = res_along.values["weight"].get<util::json::Number>().value;
		auto consumption_along = res_along.values["consumption"].get<util::json::Number>().value;
		if (consumption_along < upper_limit) {
			std::cerr << "End point is reachable without stop" << std::endl << std::endl;
			continue;
		}



		//Calc sub graph
		util::json::Object res_dijkstra_along_sub_graph{};
		auto status_dijkstra_along_sub_graph = osrm_sub_graph.EVRouteDijkstraAlongRoute(
				parameters_sub, res_dijkstra_along_sub_graph);
		if (status_dijkstra_along_sub_graph == osrm::engine::Status::Error) {
			std::cerr << "Unable to calc dijkstra along route" << std::endl << std::endl;
			continue;
		}
		auto weight_dijkstra_along_sub_graph = res_dijkstra_along_sub_graph.values["weight"].get<util::json::Number>().value;
		auto consumption_dijkstra_along_sub_graph = res_dijkstra_along_sub_graph.values["consumption"].get<util::json::Number>().value;




		util::json::Object res_dijkstra_along_full_graph{};
		auto status_dijkstra_along_full_graph = osrm_full_graph.EVRouteDijkstraAlongRoute(
				parameters_full, res_dijkstra_along_full_graph);
		if (status_dijkstra_along_full_graph == osrm::engine::Status::Error) {
			std::cerr << "Unable to calc dijkstra along route" << std::endl << std::endl;
			continue;
		}
		auto weight_dijkstra_along_full_graph = res_dijkstra_along_full_graph.values["weight"].get<util::json::Number>().value;
		auto consumption_dijkstra_along_full_graph = res_dijkstra_along_full_graph.values["consumption"].get<util::json::Number>().value;





		util::json::Object res_dijkstra_full{};
		auto status_dijkstra = osrm_full_graph.EVRouteDijkstra(parameters_full, res_dijkstra_full);
		if (status_dijkstra == osrm::engine::Status::Error) {
			std::cerr << "Unable to calc dijkstra route" << std::endl << std::endl;
			continue;
		}

		auto weight_dijkstra_full_graph = res_dijkstra_full.values["weight"].get<util::json::Number>().value;
		auto consumption_dijkstra_full_graph = res_dijkstra_full.values["consumption"].get<util::json::Number>().value;



		util::json::Object res_dijkstra_sub_graph{};
		auto status_dijkstra_sub_graph = osrm_sub_graph.EVRouteDijkstra(parameters_sub, res_dijkstra_sub_graph);
		if (status_dijkstra_sub_graph == osrm::engine::Status::Error) {
			std::cerr << "Unable to calc dijkstra route" << std::endl << std::endl;
			continue;
		}

		auto weight_dijkstra_sub_graph = res_dijkstra_sub_graph.values["weight"].get<util::json::Number>().value;
		auto consumption_dijkstra_sub_graph = res_dijkstra_sub_graph.values["consumption"].get<util::json::Number>().value;



		std::string current_line =
				  "Dijkstra-Weight-Full:                       " + std::to_string(weight_dijkstra_full_graph) + "\n"
	            + "Dijkstra-Along-Weight-Full:                 " + std::to_string(weight_dijkstra_along_full_graph) + "\n"
                + "Dijkstra-Weight-Sub:                        " + std::to_string(weight_dijkstra_sub_graph) + "\n"
				+ "Dijkstra-Along-Weight-Sub:                  " + std::to_string(weight_dijkstra_along_sub_graph) + "\n"
				+ "Along-Weight:                               " + std::to_string(weight_along) + "\n"
				+ "Dijkstra-Consumption-Full:                  " + std::to_string(consumption_dijkstra_full_graph)  + "\n"
				+ "Dijkstra-Along-Consumption-Full:            " + std::to_string(consumption_dijkstra_along_full_graph) + "\n"
				+ "Dijkstra-Consumption-Sub:                   " + std::to_string(consumption_dijkstra_sub_graph)  + "\n"
				+ "Dijkstra-Along-Consumption-Sub:             " + std::to_string(consumption_dijkstra_along_sub_graph) + "\n"

				+ "Along-Consumption:                          " + std::to_string(consumption_along) + "\n"
				+ "Dijkstra-Along-Weight-Percent:              " + std::to_string((weight_dijkstra_along_sub_graph-weight_dijkstra_full_graph)/weight_dijkstra_full_graph) + "\n"
				+ "Dijkstra-Along-Consumption-Percent:         " + std::to_string((consumption_dijkstra_along_sub_graph-consumption_dijkstra_full_graph)/consumption_dijkstra_full_graph) + "\n"
				+ "Along-Weight-Percent:                       " + std::to_string((weight_along-weight_dijkstra_full_graph)/weight_dijkstra_full_graph) + "\n"
				+ "Along-Consumption-Percent:                  " + std::to_string((consumption_along-consumption_dijkstra_full_graph)/consumption_dijkstra_full_graph) + "\n\n";

		std::cout << current_line << std::endl;


		if (
				//weight_dijkstra_full_graph > weight_dijkstra_along_full_graph
			//|| weight_dijkstra_full_graph > weight_dijkstra_sub_graph
			//|| weight_dijkstra_full_graph > weight_dijkstra_along_sub_graph
			//|| weight_dijkstra_full_graph > weight_along
			weight_dijkstra_full_graph * 2 <= weight_dijkstra_along_sub_graph
			|| weight_dijkstra_full_graph * 2 <= weight_dijkstra_along_full_graph) {
			std::cout << "+++++++++++++++++++++++++ Found a route where the full route is worse +++++++++++++++++++++++++" << std::endl;
			std::cout << "Start: " << it.first.ToString() << std::endl << "End: " << it.second.ToString() << std::endl << std::endl;


			std::string uuid = osrm::util::uuid::generate_uuid_v4();
			std::string tmp_path = "/tmp/test/wrong_paths_" + uuid;
			std::filesystem::create_directories(tmp_path);
			{
				std::ofstream tmp_out{tmp_path + "/" + uuid + "_dijkstra_full.json"};
				tmp_out << res_dijkstra_full.values["geojson"].get<util::json::String>().value;
			}
			{
				std::ofstream tmp_out{tmp_path + "/" + uuid + "_dijkstra_along_full.json"};
				tmp_out << res_dijkstra_along_full_graph.values["geojson"].get<util::json::String>().value;
			}			{
				std::ofstream tmp_out{tmp_path + "/" + uuid + "_dijkstra_sub.json"};
				tmp_out << res_dijkstra_sub_graph.values["geojson"].get<util::json::String>().value;
			}			{
				std::ofstream tmp_out{tmp_path + "/" + uuid + "_dijkstra_along_sub.json"};
				tmp_out << res_dijkstra_along_sub_graph.values["geojson"].get<util::json::String>().value;
			}			{
				std::ofstream tmp_out{tmp_path + "/" + uuid + "_along.json"};
				tmp_out << res_along.values["geojson"].get<util::json::String>().value;
			}

			std::cout << "Wrote all geojsons to " << tmp_path << std::endl;


		}


		out
			<< std::to_string(weight_dijkstra_full_graph) << "," << std::to_string(consumption_dijkstra_full_graph) << ","
			<< std::to_string(weight_dijkstra_along_full_graph) << "," << std::to_string(consumption_dijkstra_along_full_graph) << ","
			<< std::to_string(weight_dijkstra_sub_graph) << "," << std::to_string(consumption_dijkstra_sub_graph) << ","
			<< std::to_string(weight_dijkstra_along_sub_graph) << "," << std::to_string(consumption_dijkstra_along_sub_graph) << ","
			<< std::to_string(weight_along) << "," << std::to_string(consumption_along) << ","
			<< it.first.ToString() << "," << it.second.ToString() << std::endl;

		counter++;
	}

	out << "Diff Weight Percent Along: " << std::to_string(diff_percent_weight_along_route/100.0) << std::endl;
	out << "Diff Consumption Percent Along: " << std::to_string(diff_percent_consumption_along_route/100.0) << std::endl;

	out << "Diff Weight Percent Dijkstra Along: " << std::to_string(diff_percent_weight_dijkstra_along_route/100.0) << std::endl;
	out << "Diff Consumption Percent Dijkstra Along: " << std::to_string(diff_percent_consumption_dijkstra_along_route/100.0) << std::endl;



}






int main(int argc, char ** argv) {
	if (argc == 1) {
		std::cout << "please specify device: latop or tower" << std::endl;
		return 1;
	}
	std::string PREFIX;
	std::string TEST_DATA;
	if (std::string{argv[1]} == "laptop") {
		PREFIX = LAPTOP_PREFIX;
		TEST_DATA = PREFIX + "/thesis/data/test_points_2022_05.csv";
	} else if (std::string{argv[1]} == "tower") {
		PREFIX = TOWER_PREFIX;
		TEST_DATA = PREFIX + "masterarbeit/thesis/data/test_points_2022_05.csv";
	}


	const std::string OUTPUT_FOLDER = PREFIX + "masterarbeit/thesis/data/performance_test_charger_routing/";


	auto flensburg = util::Coordinate::FromDouble(9.4317400,54.7955575);
	auto freiburg = util::Coordinate::FromDouble(7.8459293,47.9907766);
	auto rosenheim = util::Coordinate::FromDouble(12.13663639,47.85434634);



	util::LogPolicy::GetInstance().Unmute();
	util::LogPolicy::GetInstance().SetLevel(logDEBUG);

	TIMER_START(SETUP);
	util::Log() << "Loading the test points ... ";
	std::vector<util::Coordinate> coords;
	{
		std::cout << TEST_DATA << std::endl;
		std::ifstream in{TEST_DATA};
		std::string line, lon_str, lat_str;
		std::getline(in, line);
		while (std::getline(in, line)) {
			std::stringstream ss{line};
			std::getline(ss, lon_str, ',');
			std::getline(ss, lat_str, ',');
			coords.emplace_back(util::Coordinate::FromDouble(std::stod(lon_str), std::stod(lat_str)));
		}
		std::cout << "Number coords: " << coords.size() << std::endl;
	}


	std::vector<std::pair<util::Coordinate, util::Coordinate>> test_point_pairs;
	util::Log() << "Generating test point pairs" ;
	{
		for (size_t i = 0; i < coords.size(); i++) {
			for (size_t j = 0; j < coords.size(); j++) {
				if (i == j) {
					continue;
				}

				test_point_pairs.emplace_back(coords[i], coords[j]);
			}
		}

		util::Log() << "Shuffeling test point pairs" ;
		auto random_device = std::random_device{};
		auto rng = std::default_random_engine{random_device()};
		std::shuffle(test_point_pairs.begin(), test_point_pairs.end(), rng);
	}

/*	{
		engine::EngineConfig engine_config_full_graph;
		engine_config_full_graph.storage_config = storage::StorageConfig("/home/kenspeckle/git/data/osrm_data_id3_0_100/germany-220519.osrm");
		engine_config_full_graph.use_shared_memory = false;
		engine_config_full_graph.use_mmap = false;

		osrm::OSRM osrm_full_graph{engine_config_full_graph};
		TIMER_STOP(SETUP);
		util::Log() << "Setup took " << TIMER_SEC(SETUP) << "s";

		// Check performance
//		performance_test_along(osrm_full_graph, test_point_pairs, OUTPUT_FOLDER + "0_100", 100000);
		performance_test_dijkstra(osrm_full_graph, test_point_pairs, OUTPUT_FOLDER + "0_100", 0, 100, 100);
		performance_test_dijkstra_along(osrm_full_graph, test_point_pairs, OUTPUT_FOLDER + "0_100", 0, 100, 50000);
		// Check grade of result
		check_grade_of_result(osrm_full_graph, test_point_pairs, OUTPUT_FOLDER + "0_100", 1000, true);

	}


	{
		engine::EngineConfig engine_config_sub_graph;
		engine_config_sub_graph.storage_config = storage::StorageConfig("/home/kenspeckle/git/data/osrm_data_id3_60_90/germany-220519.osrm");
		engine_config_sub_graph.use_shared_memory = false;
		engine_config_sub_graph.use_mmap = false;

		osrm::OSRM osrm_sub_graph{engine_config_sub_graph};
		TIMER_STOP(SETUP);
		util::Log() << "Setup took " << TIMER_SEC(SETUP) << "s";

		// Check performance
		performance_test_along(osrm_sub_graph, test_point_pairs, OUTPUT_FOLDER + "60_90", 100000);
		performance_test_dijkstra(osrm_sub_graph, test_point_pairs, OUTPUT_FOLDER + "60_90", 60, 90, 100);
		performance_test_dijkstra_along(osrm_sub_graph, test_point_pairs, OUTPUT_FOLDER + "60_90", 60, 90, 50000);
		// Check grade of result
		check_grade_of_result(osrm_sub_graph, test_point_pairs, OUTPUT_FOLDER + "60_90", 1000, false);

	}
*/

	{

		engine::EngineConfig engine_config_full_graph;
		engine_config_full_graph.storage_config = storage::StorageConfig("/home/kenspeckle/git/data/osrm_data_id3_0_100/germany-220519.osrm");
		engine_config_full_graph.use_shared_memory = false;
		engine_config_full_graph.use_mmap = false;

		osrm::OSRM osrm_full_graph{engine_config_full_graph};
		TIMER_STOP(SETUP);
		util::Log() << "Setup Full Graph took " << TIMER_SEC(SETUP) << "s";


		engine::EngineConfig engine_config_sub_graph;
		engine_config_sub_graph.storage_config = storage::StorageConfig("/home/kenspeckle/git/data/osrm_data_id3_60_90/germany-220519.osrm");
		engine_config_sub_graph.use_shared_memory = false;
		engine_config_sub_graph.use_mmap = false;

		osrm::OSRM osrm_sub_graph{engine_config_sub_graph};
		TIMER_STOP(SETUP);
		util::Log() << "Setup Sub Graph took " << TIMER_SEC(SETUP) << "s";


		Car car{"/home/kenspeckle/git/data/osrm_data_id3_60_90/germany-220519.osrm.car.properties"};

		check_grade_of_result_all_options(osrm_full_graph, osrm_sub_graph, test_point_pairs, OUTPUT_FOLDER + "everything_combined_with_points_", 100000, car.base_battery_capacity_milli_watt_h);
	}

	util::Log() << "Done filling routes_to_calculate";
	util::Log() << "====================================";
	util::Log() << "Starting to calculate all routes";



	TIMER_START(ROUTE);
	TIMER_STOP(ROUTE);
	util::Log() << "calculating the shortest charger route took " << TIMER_SEC(ROUTE) << " seconds";

	util::DumpMemoryStats();
}
