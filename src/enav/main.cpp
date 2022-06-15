//
// Created by kenspeckle on 3/14/22.
//

#include "osrm/match_parameters.hpp"
#include "osrm/nearest_parameters.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/table_parameters.hpp"
#include "osrm/trip_parameters.hpp"

#include "osrm/coordinate.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/json_container.hpp"

#include "osrm/osrm.hpp"
#include "osrm/status.hpp"
#include <iostream>

using namespace osrm;

using namespace std;

int main() {
	const string osrm_base_path = "/home/kenspeckle/git/masterarbeit/routing_engines/osrm-backend/test/data/monaco.osrm";

	EngineConfig config;
	config.storage_config = {osrm_base_path};
	config.use_shared_memory = false;
	config.max_alternatives = 0;
	config.algorithm = EngineConfig::Algorithm::CH;
	const OSRM osrm{config};

	pair<double, double> start = make_pair(7.43209,43.747812);
	pair<double, double> end = make_pair(7.414323,43.727894);

	RouteParameters params;
	params.exclude.emplace_back("ferry");
	params.coordinates.emplace_back(util::FloatLongitude{start.first}, util::FloatLatitude{start.second});
	params.coordinates.emplace_back(util::FloatLongitude{end.first}, util::FloatLatitude{end.second});
	engine::api::ResultT result = json::Object();
	const auto status = osrm.Route(params, result);

	auto &json_result = result.get<json::Object>();
	if (status == Status::Ok) {
		auto &routes = json_result.values["routes"].get<json::Array>();
		auto &route = routes.values.at(0).get<json::Object>();
		const auto consumption = route.values["consumption"].get<json::Number>().value;
		cout << "Consumption: " << to_string(consumption) << endl;
	} else {
		const auto code = json_result.values["code"].get<json::String>().value;
		const auto message = json_result.values["message"].get<json::String>().value;

		cout << "Code: " << code << endl;
		cout << "Message: " << code << endl;
		cout << "Points: " << endl;
		cout << to_string(start.second) << " " << to_string(start.first) << endl;
		cout << to_string(end.second) << " " << to_string(end.first) << endl << endl;
		throw runtime_error{"Unable to route"};
	}

}