//
// Created by kenspeckle on 4/6/22.
//

#ifndef OSRM_CHARGER_FILTER_HPP
#define OSRM_CHARGER_FILTER_HPP

#include <string>
#include <utility>
#include <vector>
#include "charger_graph/charger.hpp"
#include "charger_graph/charger_graph_edge.hpp"
#include "charger.hpp"


namespace osrm {
namespace enav {

struct ChargerPropertyFilter {
	virtual bool matches(const Charger & c) const {return true;};
};

struct PlugTypeFilter : ChargerPropertyFilter {
	inline bool matches(const Charger &c) const override {
		return std::any_of(plug_types.cbegin(), plug_types.cend(), [&c](const PlugType & _plug_type) {
			return c.has_plug_type(_plug_type);
		});
	}
	explicit PlugTypeFilter(std::vector<PlugType>  plug_types_) : plug_types(std::move(plug_types_)) {}
	explicit PlugTypeFilter(const PlugTypeFilter & o) : plug_types(o.plug_types) {}
//	explicit PlugTypeFilter(PlugType plug_type_) : plug_types{plug_type_} {}
	std::vector<PlugType> plug_types;
};

//struct MinChargingFilter : ChargerPropertyFilter {
//	explicit MinChargingFilter(const unsigned long charging_power_milli_w) : min_charging_power(charging_power_milli_w) {}
//	bool matches(const Charger &c) const override;
//	unsigned long min_charging_power;
//};
//
//struct HasFastChargingFilter : ChargerPropertyFilter {
//	bool matches(const Charger &c) const override;
//};

struct EdgePropertyFilter {
	[[nodiscard]] virtual bool matches(const ChargerGraphEdge & e) const {return true;};
};

//struct ChargerIdMatchesEdgeFilter : EdgePropertyFilter {
//	ChargerId charger_id_1;
//	ChargerId charger_id_2;
//	explicit ChargerIdMatchesEdgeFilter(const ChargerId charger_id_1_, const ChargerId charger_id_2_) : charger_id_1(charger_id_1_), charger_id_2(charger_id_2_) {}
//
//	bool matches(const ChargerGraphEdge & e) const override;
//
//};



//inline bool MinChargingFilter::matches(const Charger &c) const {
//	return std::any_of(c.plugs.cbegin(), c.plugs.cend(), [this](const Plug & _plug) {
//		return _plug.power_in_milli_w >= this->min_charging_power;
//	});
//}
//
//inline bool HasFastChargingFilter::matches(const Charger &c) const {
//	return c.has_fast_charger;
//}
//
//inline bool ChargerIdMatchesEdgeFilter::matches(const ChargerGraphEdge &e) const {
//	return e.start == charger_id_1 && e.end == charger_id_2 || e.start == charger_id_2 && e.end == charger_id_1;
//}
}
}



#endif //OSRM_CHARGER_FILTER_HPP
