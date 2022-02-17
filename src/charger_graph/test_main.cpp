//
// Created by kenspeckle on 4/9/22.
//

#include <memory>
#include <mutex>
#include "charger_graph/charger_graph_builder.hpp"
#include "util/string_util.hpp"
#include "engine/engine.hpp"
#include "csvstream.hpp"
#include "util/timing_util.hpp"
#include "charger_graph/files.hpp"


#include "util/car.hpp"
#include "osrm/osrm.hpp"
#include "charger_graph/charger_graph_edge.hpp"
#include "util/string_util.hpp"
#include "charger_graph/charger_graph.hpp"
#include "util/meminfo.hpp"
#include "engine/api/route_parameters.hpp"
#include "util/json_renderer.hpp"
#include "storage/tar.hpp"
#include "storage/serialization.hpp"
#include "util/consumption_utils.hpp"

using namespace osrm;
using namespace osrm::enav;

using namespace std;

constexpr bool full = true;
const std::string OSRM_PATH_ID3 = std::string{"/home/kenspeckle/git/data/osrm_data_id3_"} + (full ? "0_100" : "60_90") + "/germany-220519.osrm";

static std::string routesToGeojson(const std::vector<MyLegGeometry> &routes,
                                   const double battery_capacity_in_milli_wh) {
	RouteConsumption complete_consumption = 0;
	std::stringstream geojson_string_stream;
	geojson_string_stream << R"({)";
	geojson_string_stream << R"(
	"type": "FeatureCollection",
	"features": [)";
	for (size_t k = 0; k < routes.size(); k++) {
		const auto &route = routes[k];
		RouteConsumption battery_cap = battery_capacity_in_milli_wh;
		double distance_of_subroute = 0;
		BOOST_ASSERT(route.locations.size() == route.annotations.size() + 1);
		for (size_t j = 0; j < route.annotations.size(); j++) {
			distance_of_subroute += route.annotations[j].distance;


			geojson_string_stream << R"(
	{
		"type": "Feature",
		"geometry": {
			"type": "LineString",
			"coordinates": [)";
			auto begin = route.locations[j];
			geojson_string_stream << "[" << begin.ToString() << "], ";
			auto end = route.locations[j + 1];
			geojson_string_stream << "[" << end.ToString() << "]";
			geojson_string_stream << R"(]
		},)";
			geojson_string_stream << R"(
		"properties": {
			"consumption": )";
			battery_cap -= route.annotations[j].consumption;
			complete_consumption += route.annotations[j].consumption;
			auto percentage = ((double) battery_cap / battery_capacity_in_milli_wh) * 100.0;
			geojson_string_stream << std::to_string(percentage);
			geojson_string_stream << R"(
		}
	})";

			if (k != routes.size() - 1 || j != route.annotations.size() - 1) {
				geojson_string_stream << ",";
			}
		}
		std::cout << "Distance of subroute is: " << distance_of_subroute << "\t Consumption is: " << battery_cap
		          << std::endl;
	}
	geojson_string_stream << "]" << std::endl;

	geojson_string_stream << R"(})";

	std::cout << "Complete consumption: " << complete_consumption << std::endl;

	return geojson_string_stream.str();
}



std::string route_to_geojson_string(util::json::Object & route) {
	std::stringstream ss;
	util::json::render(ss, route);
	return ss.str();
}



int main() {
	util::LogPolicy::GetInstance().Unmute();

//	const std::string OSRM_PATH = OSRM_PATH_CAR_INDEPENDENT;
	const std::string OSRM_PATH = OSRM_PATH_ID3;


	osrm::enav::Car car{OSRM_PATH + ".car.properties"};
	auto car_shared_ptr = make_shared<enav::Car>(car);

	osrm::EngineConfig config;
	config.storage_config = {OSRM_PATH};
	config.use_shared_memory = false;
	config.algorithm = osrm::EngineConfig::Algorithm::CH;
	osrm::OSRM osrm{config};


	// hinfahrt
	std::vector<util::Coordinate> coords;
//	coords.emplace_back(util::Coordinate::FromDouble(6.08237582,50.76835207));
//	coords.emplace_back(util::Coordinate::FromDouble(6.3021966,50.6080009));



	// Rückfahrt
//	std::vector<util::Coordinate> coords;
//	coords.emplace_back(util::Coordinate::FromDouble(6.3014386,50.6074284));
//	coords.emplace_back(util::Coordinate::FromDouble(6.30142392,50.60899424));
//	coords.emplace_back(util::Coordinate::FromDouble(6.28675986,50.61295126));
//	coords.emplace_back(util::Coordinate::FromDouble(6.08124970,50.77133443));
//	coords.emplace_back(util::Coordinate::FromDouble(6.07810959,50.77325498));




	double lon0 = 13.463365;
	double lat0 = 52.139925;
	double lon1 = 12.985062;
	double lat1 = 50.574944;



	{
		engine::api::EVRouteParameters ev_route_parameters;
		ev_route_parameters.lower_capacity_limit_percent = 0;
		ev_route_parameters.upper_capacity_limit_percent = 100;
		ev_route_parameters.start = util::Coordinate::FromDouble(lon0,lat0);
		ev_route_parameters.end = util::Coordinate::FromDouble(lon1,lat1);
		ev_route_parameters.output_format = osrm::engine::api::EVRouteParameters::OutputFormat::IP_FRONTEND;

		json::Object result_dijkstra;
		json::Object result_dijkstra_along;
		json::Object result_along;


		std::cout << "Dijkstra" << std::endl << std::endl;
		auto status_dijkstra = osrm.EVRouteDijkstra(ev_route_parameters, result_dijkstra);
		auto dijkstra_string = route_to_geojson_string(result_dijkstra);
		std::ofstream out_dijkstra{std::string{"/tmp/test2/dijkstra_"} + (full ? "full" : "sub") + ".json"};
		out_dijkstra << dijkstra_string;
		std::ofstream out_dijkstra_geojson{std::string{"/tmp/test2/dijkstra_"} + (full ? "full" : "sub") + ".geojson"};
		out_dijkstra_geojson << result_dijkstra.values["geojson"].get<util::json::String>().value;



		std::cout << "Dijkstra Along" << std::endl << std::endl;
		auto status_dijkstra_along = osrm.EVRouteDijkstraAlongRoute(ev_route_parameters, result_dijkstra_along);
		auto dijkstra_along_string = route_to_geojson_string(result_dijkstra_along);
		std::ofstream out_dijkstra_along{std::string{"/tmp/test2/dijkstra_along_"} + (full ? "full" : "sub") + ".json"};
		out_dijkstra_along << dijkstra_along_string;
		std::ofstream out_dijkstra_along_geojson{std::string{"/tmp/test2/dijkstra_along_"} + (full ? "full" : "sub") + ".geojson"};
		out_dijkstra_along_geojson << result_dijkstra_along.values["geojson"].get<util::json::String>().value;


		std::cout << "Along" << std::endl << std::endl;
		auto status_along = osrm.EVRouteAlongRoute(ev_route_parameters, result_along);
		auto along_string = route_to_geojson_string(result_along);
		std::ofstream out_along{std::string{"/tmp/test2/along_"} + (full ? "full" : "sub") + ".json"};
		out_along << along_string;
		std::ofstream out_along_geojson{std::string{"/tmp/test2/along_"} + (full ? "full" : "sub") + ".geojson"};
		out_along_geojson << result_along.values["geojson"].get<util::json::String>().value;

		std::cout << (full ? "full" : "sub") << std::endl;
	}



	util::DumpMemoryStats();
}
