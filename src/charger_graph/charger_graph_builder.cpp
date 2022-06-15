//
// Created by kenspeckle on 3/21/22.
//

#include <sstream>
#include <set>
#include <unordered_set>

#include "charger_graph/charger_graph_builder.hpp"
#include "util/string_util.hpp"
#include "engine/engine.hpp"
#include "csvstream.hpp"
#include "util/timing_util.hpp"
#include "charger_graph/files.hpp"


namespace osrm {
namespace enav {

ChargerGraphBuilder::ChargerGraphBuilder(ChargerGraphBuilderConfig config_) : config(std::move(config_)), car(config.engine_config.storage_config.GetPath(".osrm.car.properties").string()) {
	engine = std::make_unique<engine::Engine<CH>>(config.engine_config);
}

void ChargerGraphBuilder::parseChargerFile(const std::string &file, const std::vector<PlugType> & plug_type_filters, const unsigned long max_charging_power) {
	BOOST_ASSERT(chargers.empty());
	std::ifstream in{file};
	std::stringstream ss;
	if (in) {
		std::string line;
		//skip the first 10 lines
		for (size_t i = 0; i < 10; i++) {
			getline(in, line);
		}
		while(getline(in, line)) {
			ss << util::iso_8859_1_to_utf8(line) << '\n';
		}
	}
	csvstream csvin(ss, ';', false);
	std::map<std::string, std::string> row;

	ChargerId counter = 0;
	// Read file
	while (csvin >> row) {
		auto lon_str = row["Längengrad"];
		std::replace( lon_str.begin(), lon_str.end(), ',', '.');

		auto lat_str = row["Breitengrad"];
		std::replace( lat_str.begin(), lat_str.end(), ',', '.');
		auto coord = Coordinate::FromDouble(std::stod(lon_str), std::stod(lat_str));
		auto betreiber = row["Betreiber"];
		unsigned long total_power = std::stof(row["Anschlussleistung"]) * 1000000;
		bool is_fast_charger;
		if ("Normalladeeinrichtung" == row["Art der Ladeeinrichung"]) {
			is_fast_charger = false;
		} else if ("Schnellladeeinrichtung" == row["Art der Ladeeinrichung"]) {
			is_fast_charger = true;
		} else {
			util::Log(logERROR) << "Unknown charger type: " + row["Art der Ladeeinrichung"] << " skipping line";
			continue;
		}


		auto splt_string = [](std::string str, char delim) {
			std::vector<std::string> ret;
			size_t pos = str.find(delim);
			if (pos == std::string::npos) {
				ret.push_back(str);
				return ret;
			}
			std::string token;
			while ((pos = str.find(delim)) != std::string::npos) {
				token = str.substr(0, pos);
				ret.emplace_back(token);
				str.erase(0, pos + 1);
			}
			return ret;
		};

		std::vector<Plug> plugs;
		auto nbr_plugs = std::stoul(row["Anzahl Ladepunkte"]);
		if (nbr_plugs == 0) {
			util::Log(logWARNING) << "Number of plugs is 0. Skipping line";
		}

		for (size_t i = 0; i < nbr_plugs; i++) {
			auto plug_type = row[std::string{"Steckertypen" + std::to_string(i+1)}];
			if (!plug_type.empty()) {
				std::vector<PlugType> plug_types;
				for (auto & it : splt_string(plug_type, ';')) {
					for (auto & itt : splt_string(it, ',')) {
						plug_types.emplace_back(PLUG_TYPE_MAP.at(util::trim_copy(itt)));
					}
				}
				BOOST_ASSERT(!plug_types.empty());
				auto plug_power = std::stof(row[std::string{"P" + std::to_string(i+1) + " [kW]"}]);
				plugs.emplace_back(Plug{std::min(static_cast<unsigned long>(plug_power * 1000000), max_charging_power), plug_types});
			}
		}

		if (!plug_type_filters.empty()) {
			bool found_matching_filter = false;
			for (const auto & plug : plugs) {
				for (const auto & plug_type : plug.plug_types) {
					auto matches = std::any_of(plug_type_filters.cbegin(), plug_type_filters.cend(), [&plug_type](const auto & f_plug_type) {return plug_type == f_plug_type;});
					if (matches) {
						found_matching_filter = true;
						break;
					}
				}
				if (found_matching_filter) {
					break;
				}
			}
			if (!found_matching_filter) {
				continue;
			}

		}

		if (nbr_plugs != plugs.size()) {
			util::Log(logWARNING) << "The Charger at " << coord.ToString() << " should have " << std::to_string(nbr_plugs) << " but we could only extract " << std::to_string(plugs.size());
		}
		BOOST_ASSERT(!plugs.empty());

		auto phantom = engine->GetPhantomNodePair(coord);

		auto new_charger = Charger(counter, phantom, coord, betreiber, total_power, is_fast_charger, plugs);
		counter++;
		chargers.emplace_back(new_charger);
	}

	dBScan();
	compressClusters();
	renumberChargers();

	util::Log() << "Got " << std::to_string(chargers.size()) << " chargers";
	files::writeChargers(config.engine_config.storage_config.GetPath(".osrm.charger_graph").string(), chargers);
}

void ChargerGraphBuilder::renumberChargers() {
	// Update the node id to match the vector index to speed up calculations later on
	for (size_t i = 0; i < chargers.size(); i++) {
		chargers[i].node_id = i;
	}
	NodeID current_id = chargers.size();
	for (auto & charger : chargers) {
		for (auto & it : charger.charger_in_cluster) {
			it.node_id = current_id++;
		}
	}
}



void ChargerGraphBuilder::dBScan() {
	// make sure that chargers[i].node_id == i
	renumberChargers();

	std::uint32_t cluster_counter = 0;
	for (auto & it : chargers) {
		if (it.cluster_id != UNCLASSIFIED) {
			continue;
		}
		std::vector<size_t> neighbors = regionQuery(it.node_id);
		if (neighbors.size() + 1 < min_cluster_size) {  // neighbors does not include the current node
			it.cluster_id = NOISE;
			continue;
		}

		it.cluster_id = cluster_counter;

		bool neighbors_expanded = false;
		for (size_t i = 0; i < neighbors.size(); i++) {
			if (neighbors_expanded) {
				i = 0;
				neighbors_expanded = false;
			}
			auto n = neighbors[i];
			if (chargers[n].cluster_id == NOISE) {
				chargers[n].cluster_id = cluster_counter;
			} else if (chargers[n].cluster_id != UNCLASSIFIED) {
				continue;
			}
			chargers[n].cluster_id = cluster_counter;

			std::vector<size_t> sub_neighbors = regionQuery(n);
			if (sub_neighbors.size() + 1 >= min_cluster_size) {

				for (unsigned long & sub_neighbor : sub_neighbors) {
					if (std::find(neighbors.cbegin(), neighbors.cend(), sub_neighbor) == neighbors.cend()) {
						neighbors_expanded = true;
						neighbors.push_back(sub_neighbor);
					}
				}
			}
		}
		cluster_counter++;
	}
}

std::vector<size_t> ChargerGraphBuilder::regionQuery(const size_t index) {
	std::vector<size_t> ret;
	for (size_t i = 0; i < chargers.size(); i++) {
		if (i == index) {
			continue;
		}
		if (util::coordinate_calculation::haversineDistance(chargers[index].coordinate, chargers[i].coordinate) < min_epsilon) {
			ret.emplace_back(i);
		}
	}
	return ret;
}


void ChargerGraphBuilder::compressClusters() {
	const auto prev_number_chargers = chargers.size();
	renumberChargers();
	std::unordered_map<size_t, size_t> cluster_id_charger_mapping;
	std::vector<NodeID> charger_ids_to_delete;
	for (auto & it : chargers) {
		if (it.cluster_id == NOISE) {
			continue;
		}
		auto idx = cluster_id_charger_mapping.find(it.cluster_id);
		if (idx == cluster_id_charger_mapping.end()) {
			cluster_id_charger_mapping.emplace(it.cluster_id, it.node_id);
		} else {
			chargers[idx->second].charger_in_cluster.push_back(chargers[it.node_id]);
			charger_ids_to_delete.push_back(it.node_id);
		}
	}

	chargers.erase(std::remove_if(chargers.begin(), chargers.end(), [&](const Charger & c) {
		return std::any_of(charger_ids_to_delete.cbegin(), charger_ids_to_delete.cend(), [&](const auto & id) {
			return id == c.node_id;
		});
	}), chargers.end());

	renumberChargers();

	util::Log() << "Compressed " << prev_number_chargers << " to " << chargers.size() << " chargers" << std::endl;
}


void ChargerGraphBuilder::buildChargerGraph(const double lower_limit_percent, const double upper_limit_percent) {
	BOOST_ASSERT(!chargers.empty());

	std::vector<EdgeConsumption> consumptions;
	std::vector<EdgeWeight> weights;
	std::vector<EdgeDistance> distance;

	const auto upper_capacity_limit = car.base_battery_capacity_milli_watt_h * (upper_limit_percent/100.0);
	const auto lower_capacity_limit = car.base_battery_capacity_milli_watt_h * (lower_limit_percent/100.0);

	std::vector<engine::PhantomNodePair> all_phantom_nodes;
	unsigned long chargers_size = chargers.size();
	{
		for (size_t i = 0; i < chargers_size; i++) {
			all_phantom_nodes.emplace_back(chargers[i].phantom_node_pair);
		}

		util::Log() << "Starting many-to-many request ...";
		TIMER_START(TMP);
		auto result = engine->ManyToManyInternal(all_phantom_nodes, {}, {}, true);
		TIMER_STOP(TMP);
		util::Log() << "Calculating the many-to-many request took " << TIMER_SEC(TMP);

		consumptions = std::get<2>(result);
		weights = std::get<3>(result);
	}

	{
		// This will result in an implicitly sorted list
		// This feature is very important for the later usage and should be keep at all times
		std::vector<ChargerGraphEdge> edges;
		edges.reserve(chargers_size * (chargers_size - 1));     // In the worst case we have a fully connected graph (without loops on the same node)
		unsigned long skipped_edges = 0;
		size_t counter_wrong_consumption = 0;

		for (size_t i = 0; i < chargers_size; i++) {
			auto tmp_start_charger_id = i;
			BOOST_ASSERT(tmp_start_charger_id == chargers[i].node_id);
			for (size_t j = 0; j < chargers_size; j++) {
				if (i != j) {
					auto vec_index = i * chargers_size + j;
					auto tmp_consumption = consumptions[vec_index];
					if (tmp_consumption == INVALID_EDGE_CONSUMPTION) {
						skipped_edges++;
						continue;
					}

					// We only allow Edges with a positive consumption as it would just add redundancy
					if (tmp_consumption >= lower_capacity_limit && tmp_consumption <= upper_capacity_limit) {
						BOOST_ASSERT(chargers[j].node_id == j);
						edges.emplace_back(tmp_start_charger_id, chargers[j].node_id, weights[vec_index], tmp_consumption);
					} else {
						skipped_edges++;
					}
				}
			}
		}

		util::Log(logWARNING) << "There were " << counter_wrong_consumption << " wrong consumption routes" << std::endl;

		for (const auto & it : edges) {
			if (it.start >= chargers_size || it.end >= chargers_size) {
				throw std::runtime_error{"Invalid edges were created..."};
			}
		}

		files::writeEdges(config.engine_config.storage_config.GetPath(".osrm.charger_graph"), edges);

		util::Log() << "There are now " << std::to_string(edges.size()) << " edges in the charger graph";
		util::Log() << "Skipped " << std::to_string(skipped_edges) << " because they had a negative consumption or a consumption larger than the battery capacity";
	}
}

void ChargerGraphBuilder::Run() {
	TIMER_START(GLOBAL);

	auto plug_type_filters = std::vector<PlugType>();
	if (!config.no_filter_plug_type) {
		for (auto & plug_type : car.supported_plug_types) {
			plug_type_filters.push_back(plug_type);
		}
	}
	parseChargerFile(config.charger_csv_path.string(), plug_type_filters, car.max_charging_power);
	renumberChargers();
	buildChargerGraph(config.lower_limit_percent, config.upper_limit_percent);
	TIMER_STOP(GLOBAL);
	util::Log() << "This took " << TIMER_SEC(GLOBAL) << "s in total";
}


ChargerGraphBuilder::ChargerGraphBuilder(std::vector<Charger> chargers, const unsigned int cluster_size, const double epsilon) : chargers(std::move(chargers)), min_cluster_size(cluster_size), min_epsilon(epsilon) {
	std::cerr << "WARNING: This constructor of ChargerGraphBuilder should only be used for testing purposes" << std::endl;

}

}
}
