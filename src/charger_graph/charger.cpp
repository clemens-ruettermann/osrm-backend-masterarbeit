//
// Created by kenspeckle on 3/21/22.
//

#include <algorithm>
#include <utility>
#include "charger_graph/charger.hpp"
#include "util/coordinate_calculation.hpp"
#include "util/string_util.hpp"


namespace osrm {
namespace enav {

Charger::Charger(ChargerId node_id_, std::pair<engine::PhantomNode, engine::PhantomNode> phantom_node_pair_, Coordinate coordinate_, std::string _operator_, unsigned long  total_power_, bool has_fast_charger_, std::vector<Charger> charger_in_cluster_)
	: node_id(node_id_),
	  phantom_node_pair(std::move(phantom_node_pair_)),
	  coordinate(coordinate_),
	  _operator(util::trim_copy(std::move(_operator_))),
	  total_power_in_milli_w(total_power_),
	  has_fast_charger(has_fast_charger_),
	  cluster_size(charger_in_cluster_.size()),
	  is_cluster(charger_in_cluster_.empty()),
	  charger_in_cluster(std::move(charger_in_cluster_)),
	  min_power(0),
	  max_power(0),
	  plugs{}

{
	std::for_each(charger_in_cluster.cbegin(), charger_in_cluster.cend(), [this](const Charger & charger) {
		plugs.insert(plugs.cend(), charger.plugs.begin(), charger.plugs.end());
	});
//	std::accumulate(charger_in_cluster_.begin(), charger_in_cluster_.end(), decltype(plugs)::value_type{}, [this](auto& dest, auto& src) {
//		plugs.insert(dest.end(), src.begin(), src.end());
//		return dest;
//	});
	min_power = std::min_element(plugs.cbegin(), plugs.cend(), [](const Plug & lhs, const Plug & rhs){
		return lhs.power_in_milli_w < rhs.power_in_milli_w;
	})->power_in_milli_w;

	max_power = std::max_element(plugs.cbegin(), plugs.cend(), [](const Plug & lhs, const Plug & rhs){
		return lhs.power_in_milli_w < rhs.power_in_milli_w;
	})->power_in_milli_w;
	BOOST_ASSERT(!plugs.empty());
	BOOST_ASSERT(max_power > 0);
	BOOST_ASSERT(min_power > 0);
	BOOST_ASSERT(total_power_in_milli_w > 0);
}

Charger::Charger(ChargerId node_id_, std::pair<engine::PhantomNode, engine::PhantomNode> phantom_node_pair_, Coordinate coordinate_, std::string _operator_, unsigned long  total_power_, bool has_fast_charger_, std::vector<Plug> plugs_)
	: node_id(node_id_),
	  phantom_node_pair(std::move(phantom_node_pair_)),
	  coordinate(coordinate_),
	  _operator(util::trim_copy(std::move(_operator_))),
	  total_power_in_milli_w(total_power_),
	  has_fast_charger(has_fast_charger_),
	  plugs(std::move(plugs_)),
	  is_cluster(false),
	  charger_in_cluster{},
	  min_power(0),
	  max_power(0),
	  cluster_size(0)

{
	if (!plugs.empty()) {
		min_power = std::min_element(plugs.cbegin(), plugs.cend(), [](const Plug &lhs, const Plug &rhs) {
			return lhs.power_in_milli_w < rhs.power_in_milli_w;
		})->power_in_milli_w;

		max_power = std::max_element(plugs.cbegin(), plugs.cend(), [](const Plug & lhs, const Plug & rhs){
			return lhs.power_in_milli_w < rhs.power_in_milli_w;
		})->power_in_milli_w;
	}
	BOOST_ASSERT(!plugs.empty());
	BOOST_ASSERT(max_power > 0);
	BOOST_ASSERT(min_power > 0);
	BOOST_ASSERT(total_power_in_milli_w > 0);
}

bool Charger::has_plug_type(const PlugType &p) const {
	for (const auto & it : plugs) {
		for (const auto & itt : it.plug_types) {
			if (p == itt) {
				return true;
			}
		}
	}
	return false;
}

Charger::Charger(ChargerId node_id, std::pair<engine::PhantomNode, engine::PhantomNode> phantom_node_pair_, Coordinate coord, bool is_cluster, unsigned long total_power_in_milli_w,
                 std::string _operator, std::vector<Plug> plugs, std::vector<Charger> charger_in_cluster,
                 unsigned long cluster_size, bool has_fast_charger, unsigned long min_power, unsigned long max_power):
				 node_id(node_id),
				 phantom_node_pair(std::move(phantom_node_pair_)),
				 coordinate(coord),
				 is_cluster(is_cluster),
				 total_power_in_milli_w(total_power_in_milli_w),
				 _operator(std::move(_operator)),
				 plugs(std::move(plugs)),
				 charger_in_cluster(std::move(charger_in_cluster)),
				 cluster_size(cluster_size),
				 has_fast_charger(has_fast_charger),
				 min_power(min_power),
				 max_power(max_power)
				 {
					 BOOST_ASSERT(this->max_power > 0);
					 BOOST_ASSERT(this->min_power > 0);
					 BOOST_ASSERT(this->total_power_in_milli_w > 0);
					 BOOST_ASSERT(!this->plugs.empty());
}

Charger::Charger(ChargerId node_id, Coordinate coord) : node_id(node_id), coordinate(coord) {}


}
}