#ifndef SERVER_API_EVROUTE_PARAMETERS_GRAMMAR_HPP
#define SERVER_API_EVROUTE_PARAMETERS_GRAMMAR_HPP

#include "engine/api/ev_route_parameters.hpp"
#include "server/api/base_parameters_grammar.hpp"

#include "engine/hint.hpp"
#include "engine/polyline_compressor.hpp"

#include <boost/spirit/include/phoenix.hpp>
#include <boost/spirit/include/qi.hpp>

#include <string>

namespace osrm {
namespace server {
namespace api {

namespace {
namespace ph = boost::phoenix;
namespace qi = boost::spirit::qi;
} // namespace

template<typename Iterator = std::string::iterator,
		typename Signature = void(engine::api::EVRouteParameters & )>
struct EVRouteParametersGrammar final : boost::spirit::qi::grammar<Iterator, Signature> {
	using json_policy = no_trailing_dot_policy<double, 'j', 's', 'o', 'n'>;

	EVRouteParametersGrammar() : EVRouteParametersGrammar::base_type(root_rule) {
		location_rule = (double_ > qi::lit(',') >
		                 double_)[qi::_val = ph::bind(
				[](double lon, double lat) {
					return util::Coordinate(
							util::toFixed(util::UnsafeFloatLongitude{lon}),
							util::toFixed(util::UnsafeFloatLatitude{lat}));
				},
				qi::_1,
				qi::_2)];

		algos_type.add("along_route_shortest_distance", engine::api::EVRouteParameters::Algo::ALONG_ROUTE_SHORTEST_DISTANCE)(
				"along_route_smallest_consumption", engine::api::EVRouteParameters::Algo::ALONG_ROUTE_SMALLEST_CONSUMPTION)(
				"dijkstra", engine::api::EVRouteParameters::Algo::DIJKSTRA)(
				"dijkstra_along_route", engine::api::EVRouteParameters::Algo::DIJKSTRA_ALONG_ROUTE);

		const auto add_lower_capacity_limit = [](engine::api::EVRouteParameters &parameters,
		                                         boost::optional<double> limit) {
			auto val = limit.get();
			if (val > 100 || val < 0) {
				throw std::runtime_error{"Unsupported limit"};
			}
			parameters.lower_capacity_limit_percent = limit.get() / 100.0;
			if (parameters.upper_capacity_limit_percent < parameters.lower_capacity_limit_percent) {
				throw std::runtime_error{"Upper limit must be larger than lower limit"};
			}
		};

		const auto add_upper_capacity_limit = [](engine::api::EVRouteParameters &parameters,
		                                         boost::optional<double> limit) {
			auto val = limit.get();
			if (val > 100 || val < 0) {
				throw std::runtime_error{"Unsupported limit"};
			}
			parameters.upper_capacity_limit_percent = val / 100.0;
			if (parameters.upper_capacity_limit_percent < parameters.lower_capacity_limit_percent) {
				throw std::runtime_error{"Upper limit must be larger than lower limit"};
			}
		};

		ev_route_rule = (location_rule[ph::bind(&engine::api::EVRouteParameters::start, qi::_r1) = qi::_1]
		                 > qi::lit(';')
		                 > location_rule[ph::bind(&engine::api::EVRouteParameters::end, qi::_r1) = qi::_1]);

		lower_capacity_rule =qi::lit("lower_capacity_limit=") > (qi::double_)[ph::bind(add_lower_capacity_limit, qi::_r1, qi::_1)];

		upper_capacity_rule = qi::lit("upper_capacity_limit=") > (qi::double_)[ph::bind(add_upper_capacity_limit, qi::_r1, qi::_1)];


		search_radius_rule = qi::lit("search_radius=") >
		                     qi::double_[ph::bind(&engine::api::EVRouteParameters::search_radius, qi::_r1) = qi::_1];

		temperature_rule = qi::lit("temperature=") >
		                     qi::double_[ph::bind(&engine::api::EVRouteParameters::temperature, qi::_r1) = qi::_1];

		algo_rule = (qi::lit("algo=") >
							algos_type[ph::bind(&engine::api::EVRouteParameters::algo, qi::_r1) = qi::_1]);

		root_rule = ev_route_rule(qi::_r1) >
		            -('?' > (
				                    upper_capacity_rule(qi::_r1) |
				                    lower_capacity_rule(qi::_r1) |
									algo_rule(qi::_r1) |
									search_radius_rule(qi::_r1)
		                    ) % '&');

	}

private:
	qi::real_parser<double, json_policy> double_;
	qi::rule<Iterator, osrm::util::Coordinate()> location_rule;
	qi::rule<Iterator, Signature> root_rule;
	qi::rule<Iterator, Signature> ev_route_rule;
	qi::rule<Iterator, Signature> upper_capacity_rule;
	qi::rule<Iterator, Signature> lower_capacity_rule;
	qi::rule<Iterator, Signature> search_radius_rule;
	qi::rule<Iterator, Signature> temperature_rule;
	qi::rule<Iterator, Signature> algo_rule;

	qi::symbols<char, engine::api::EVRouteParameters::Algo> algos_type;
};
} // namespace api
} // namespace server
} // namespace osrm

#endif
