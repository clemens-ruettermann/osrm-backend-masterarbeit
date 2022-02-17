#include "engine/plugins/nearest.hpp"
#include "engine/api/nearest_api.hpp"
#include "engine/api/nearest_parameters.hpp"
#include "engine/phantom_node.hpp"
#include "util/integer_range.hpp"

#include <cstddef>
#include <string>

#include <boost/assert.hpp>
#include <boost/numeric/conversion/cast.hpp>

namespace osrm
{
namespace engine
{
namespace plugins
{

NearestPlugin::NearestPlugin(const int max_results_) : max_results{max_results_} {}

Status NearestPlugin::HandleRequest(const RoutingAlgorithmsInterface &algorithms,
                                    const api::NearestParameters &params,
                                    osrm::engine::api::ResultT &result) const
{
    BOOST_ASSERT(params.IsValid());

    if (!CheckAlgorithms(params, algorithms, result))
        return Status::Error;

    const auto &facade = algorithms.GetFacade();

    if (max_results > 0 &&
        (boost::numeric_cast<std::int64_t>(params.number_of_results) > max_results))
    {
        return Error("TooBig",
                     "Number of results " + std::to_string(params.number_of_results) +
                         " is higher than current maximum (" + std::to_string(max_results) + ")",
                     result);
    }

    if (!CheckAllCoordinates(params.coordinates))
        return Error("InvalidOptions", "Coordinates are invalid", result);

    if (params.coordinates.size() != 1)
    {
        return Error("InvalidOptions", "Only one input coordinate is supported", result);
    }

    auto phantom_nodes = GetPhantomNodes(facade, params, params.number_of_results);

    if (phantom_nodes.front().size() == 0)
    {
        return Error("NoSegment", "Could not find a matching segments for coordinate", result);
    }
    BOOST_ASSERT(phantom_nodes.front().size() > 0);

    api::NearestAPI nearest_api(facade, params);
    nearest_api.MakeResponse(phantom_nodes, result);

    return Status::Ok;
}


std::vector<PhantomNodePair> NearestPlugin::GetSnappedPhantomNodes(const RoutingAlgorithmsInterface &algorithms, const std::vector<util::Coordinate> & coordinates) const {
	if (!CheckAllCoordinates(coordinates)) {
		throw std::runtime_error("Coordinates are invalid");
	}

	const auto &facade = algorithms.GetFacade();
	api::BaseParameters params;
	for (size_t i = 0; i < coordinates.size(); i++) {
		params.approaches.emplace_back(Approach::UNRESTRICTED);
	}
	params.snapping = api::BaseParameters::SnappingType::Any;
	params.coordinates = coordinates;
	std::vector<PhantomNodePair> phantom_nodes = GetPhantomNodes(facade, params);

	if (phantom_nodes.size() != coordinates.size())
	{
		auto mismatch = std::mismatch(phantom_nodes.begin(),
		                              phantom_nodes.end(),
		                              coordinates.begin(),
		                              coordinates.end(),
		                              [](const auto &phantom_node, const auto &coordinate) {
			                              return phantom_node.first.input_location == coordinate;
		                              });
		std::size_t missing_index = std::distance(phantom_nodes.begin(), mismatch.first);
		std::string msg = std::string("Could not find a matching segment for coordinate ") + std::to_string(missing_index);
		throw std::runtime_error{msg};
	}

//	auto snapped_phantoms = SnapPhantomNodes(phantom_nodes);
	return phantom_nodes;
}

PhantomNodePair NearestPlugin::GetPhantomNodePair(const RoutingAlgorithmsInterface &algorithms, const util::Coordinate & coordinate) const {
	if (!coordinate.IsValid()) {
		throw std::runtime_error{"Coordinate is invalid"};
	}
	const auto &facade = algorithms.GetFacade();
	api::BaseParameters params;
	params.approaches.emplace_back(Approach::UNRESTRICTED);
	params.snapping = api::BaseParameters::SnappingType::Any;
	params.coordinates.emplace_back(coordinate);
	std::vector<PhantomNodePair> phantom_nodes = GetPhantomNodes(facade, params);

	if (phantom_nodes.size() != 1)
	{
		std::string msg = std::string("Could not find a matching segment for coordinate ");
		throw std::runtime_error{msg};
	}
	return phantom_nodes.at(0);
}


std::vector<PhantomNode> NearestPlugin::SnapPhantomNodePairs(const std::vector<PhantomNodePair> & phantom_node_pairs) const {
	return SnapPhantomNodes(phantom_node_pairs);
}

PhantomNode NearestPlugin::SnapPhantomNodePair(const PhantomNodePair & phantom_node_pair) const {
	auto res = SnapPhantomNodes({phantom_node_pair});
	BOOST_ASSERT(res.size() == 1);
	return res[0];
}

} // namespace plugins
} // namespace engine
} // namespace osrm
