//
// Created by kenspeckle on 3/22/22.
//

#ifndef OSRM_CHARGER_GRAPH_BUILDER_CONFIG_H
#define OSRM_CHARGER_GRAPH_BUILDER_CONFIG_H

#include <string>
#include "engine/engine_config.hpp"

namespace osrm {
namespace enav {
class ChargerGraphBuilderConfig {
public:
	boost::filesystem::path base_path;
	boost::filesystem::path charger_csv_path;
	engine::EngineConfig engine_config;
	double upper_limit_percent = 90;
	double lower_limit_percent = 50;
	bool no_filter_plug_type = false;
};

}
}

#endif //OSRM_CHARGER_GRAPH_BUILDER_CONFIG_H
