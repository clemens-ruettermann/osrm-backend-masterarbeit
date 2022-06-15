#include "server/service/ev_route_service.hpp"
#include "server/service/utils.hpp"

#include "server/api/parameters_parser.hpp"
#include "engine/api/route_parameters.hpp"

#include "util/json_container.hpp"

namespace osrm
{
namespace server
{
namespace service
{


engine::Status osrm::server::service::EVRouteService::RunQuery(std::size_t prefix_length, std::string &query, osrm::engine::api::ResultT &result)
{
    result = util::json::Object();
    auto &json_result = result.get<util::json::Object>();

    auto query_iterator = query.begin();
    auto opt_parameters = api::parseParameters<engine::api::EVRouteParameters>(query_iterator, query.end());
    if (!opt_parameters || query_iterator != query.end())
    {
        const auto position = std::distance(query.begin(), query_iterator);
        json_result.values["code"] = "InvalidQuery";
        json_result.values["message"] = "Query string malformed close to position " + std::to_string(prefix_length + position);
        return engine::Status::Error;
    }
    BOOST_ASSERT(opt_parameters);

    if (!opt_parameters->IsValid())
    {
        json_result.values["code"] = "InvalidOptions";
//        json_result.values["message"] = getWrongOptionHelp(*parameters);
		json_result.values["message"] = "Parameters are invalid";
        return engine::Status::Error;
    }
    BOOST_ASSERT(opt_parameters->IsValid());

	util::json::Object tmp_result = util::json::Object();
	
	auto parameters = *opt_parameters;
	engine::Status status;
	switch (parameters.algo) {

		case EVRouteParameters::Algo::DIJKSTRA:
			status = BaseService::routing_machine.EVRouteDijkstra(parameters, tmp_result);
			break;
		case EVRouteParameters::Algo::DIJKSTRA_ALONG_ROUTE:
			status = BaseService::routing_machine.EVRouteDijkstraAlongRoute(parameters, tmp_result);
			break;
		case EVRouteParameters::Algo::ALONG_ROUTE_SMALLEST_CONSUMPTION:
		case EVRouteParameters::Algo::ALONG_ROUTE_SHORTEST_DISTANCE:
			status = BaseService::routing_machine.EVRouteAlongRoute(parameters, tmp_result);
			break;
	}
	result = tmp_result;
	return status;
}
} // namespace service
} // namespace server
} // namespace osrm
