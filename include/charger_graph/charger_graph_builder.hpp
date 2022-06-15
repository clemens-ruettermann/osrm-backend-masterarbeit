//
// Created by kenspeckle on 3/21/22.
//

#ifndef OSRM_CHARGER_GRAPH_BUILDER_H
#define OSRM_CHARGER_GRAPH_BUILDER_H


#include <string>
#include <vector>
#include "charger.hpp"
#include "charger_graph.hpp"

#include "charger_graph/charger_graph_builder_config.hpp"

#include "engine/plugins/viaroute.hpp"
#include "engine/plugins/match.hpp"
#include "engine/plugins/table.hpp"
#include "engine/plugins/nearest.hpp"
#include "storage/storage_config.hpp"
#include "engine/datafacade_provider.hpp"
#include "osrm/osrm.hpp"
#include "engine/engine.hpp"

namespace osrm {
namespace enav {

using CH = engine::routing_algorithms::ch::Algorithm;

class ChargerGraphBuilder {
public:
	explicit ChargerGraphBuilder(ChargerGraphBuilderConfig config);
	ChargerGraphBuilder(std::vector<Charger> charger, const unsigned int cluerst_size, const double epsilon);
	~ChargerGraphBuilder() = default;
	void Run();
	void parseChargerFile(const std::string& file, const std::vector<PlugType> & plug_type_filters, const unsigned long max_charging_power);
	void buildChargerGraph(double d, double d1);

	std::vector<Charger> chargers;
	void dBScan();
	void compressClusters();
private:

	std::unique_ptr<engine::Engine<CH>> engine;
	ChargerGraphBuilderConfig config;
	std::string filename;

	enav::Car car;
	double min_epsilon = 30;
	unsigned int min_cluster_size = 2;

	void renumberChargers();
	std::vector<size_t> regionQuery(const size_t index);
};

}
}

#endif //OSRM_CHARGER_GRAPH_BUILDER_H
