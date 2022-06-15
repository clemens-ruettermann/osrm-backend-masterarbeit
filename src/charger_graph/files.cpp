#include <vector>
#include <numeric>
#include <unordered_set>
#include <fstream>
#include <boost/filesystem.hpp>
#include <boost/iterator/function_output_iterator.hpp>
#include <boost/iterator/function_input_iterator.hpp>
#include "charger_graph/files.hpp"
#include "storage/tar.hpp"
#include <json.hpp>
#include "engine/hint.hpp"


namespace osrm {
namespace enav {
namespace files {

nlohmann::json chargerToJson(const Charger & charger) {
	nlohmann::json tmp_j = nlohmann::json::object();
	tmp_j["node_id"] = charger.node_id;
	const osrm::engine::Hint hint1{charger.phantom_node_pair.first, 0};
	const osrm::engine::Hint hint2{charger.phantom_node_pair.second, 0};
	tmp_j["phantom_node_1_str"] = hint1.ToBase64();
	tmp_j["phantom_node_2_str"] = hint2.ToBase64();
	tmp_j["lon"] = (int)charger.coordinate.lon;
	tmp_j["lat"] = (int)charger.coordinate.lat;
	tmp_j["is_cluster"] = charger.is_cluster;
	tmp_j["total_power_in_milli_w"] = charger.total_power_in_milli_w;
	tmp_j["_operator"] = charger._operator;
	tmp_j["plugs"] = nlohmann::json::array();
	for (auto & it_plug : charger.plugs) {
		auto tmp_plug_j = nlohmann::json::object();
		tmp_plug_j["power_in_milli_w"] = it_plug.power_in_milli_w;
		tmp_plug_j["plug_types"] = nlohmann::json::array();
		for (auto & it_all_plug_type : it_plug.plug_types) {
			for (auto & plug_type : PLUG_TYPE_MAP) {
				if (plug_type.second == it_all_plug_type) {
					tmp_plug_j["plug_types"].push_back(plug_type.first);
					break;
				}
			}
		}
		tmp_j["plugs"].push_back(tmp_plug_j);
	}
	tmp_j["charger_in_cluster"] = nlohmann::json::array();
	for (auto & it : charger.charger_in_cluster) {
		tmp_j["charger_in_cluster"].push_back(chargerToJson(it));
	}
	tmp_j["cluster_size"] = charger.cluster_size;
	tmp_j["has_fast_charger"] = charger.has_fast_charger;
	tmp_j["min_power"] = charger.min_power;
	tmp_j["max_power"] = charger.max_power;
	return tmp_j;
}


Charger chargerFromJson(const nlohmann::json & tmp_j) {
	NodeID node_id = tmp_j["node_id"];
	auto phantom_node_1 = osrm::engine::Hint::FromBase64(tmp_j["phantom_node_1_str"]).phantom;
	auto phantom_node_2 = osrm::engine::Hint::FromBase64(tmp_j["phantom_node_2_str"]).phantom;
	auto lon = util::FixedLongitude {tmp_j["lon"]};
	auto lat = util::FixedLatitude {tmp_j["lat"]};
	util::Coordinate coord{lon, lat };
	bool is_cluster = tmp_j["is_cluster"];

	unsigned long total_power_in_milli_w =tmp_j["total_power_in_milli_w"];
	std::string _operator = tmp_j["_operator"];
	auto plugs = std::vector<Plug>();
	for (auto & itt : tmp_j["plugs"]) {
		unsigned long power_in_milli_w = itt["power_in_milli_w"];
		std::vector<PlugType> new_plug_types;
		for (auto & ittt : itt["plug_types"]) {
			new_plug_types.push_back(PLUG_TYPE_MAP.at(ittt));
		}

		Plug tmp_plug{power_in_milli_w, new_plug_types};
		plugs.push_back(tmp_plug);
	}
	std::vector<Charger> charger_in_cluster;
	for (auto & charger_in_cluster_json : tmp_j["charger_in_cluster"]) {
		charger_in_cluster.push_back(chargerFromJson(charger_in_cluster_json));
	}

	unsigned long cluster_size = tmp_j["cluster_size"];
	bool has_fast_charger = tmp_j["has_fast_charger"];
	unsigned long min_power = tmp_j["min_power"];
	unsigned long max_power = tmp_j["max_power"];
	Charger ret = Charger(node_id, std::make_pair(phantom_node_1, phantom_node_2), coord, is_cluster, total_power_in_milli_w, _operator, plugs, charger_in_cluster, cluster_size, has_fast_charger, min_power, max_power);
	return ret;
}


void writeChargers(const boost::filesystem::path &path, const std::vector<Charger> & chargers) {
	nlohmann::json j = nlohmann::json::array();;
	for (auto & it : chargers) {
		j.push_back(chargerToJson(it));
	}

	std::string s = j.dump(4);
	std::ofstream out{path.string() + ".charger.json"};
	out << s;
}

void writeEdges(const boost::filesystem::path &path, const std::vector<ChargerGraphEdge> &edges) {
	storage::tar::FileWriter writer(path, storage::tar::FileWriter::GenerateFingerprint);

	writer.WriteElementCount64("/charger_graph/edges", edges.size());
	writer.WriteStreaming<ChargerGraphEdge>("/charger_graph/edges", edges.begin(), edges.size());
}

void writeChargerGraph(const boost::filesystem::path &path,
                       const std::vector<ChargerGraphEdge> &edges,
                       const std::vector<Charger> & chargers)
{
	writeEdges(path, edges);
	writeChargers(path, chargers);
}


void readChargers(const boost::filesystem::path & path, std::vector<Charger> & chargers) {
	std::ifstream in{path.string()};
	std::stringstream ss;
	std::string line;
	while(std::getline(in, line)) {
		ss << line;
	}

	std::string json_string = ss.str();
	nlohmann::json j = nlohmann::json::parse(json_string);
	for (auto & i : j) {
		chargers.push_back(chargerFromJson(i));
	}
	std::sort(chargers.begin(), chargers.end(), [](const auto & left, const auto & right) {
		return left.node_id < right.node_id;
	});
}


// reads .osrm.charger_graph file
void readChargerGraph(const boost::filesystem::path &path,
                      std::vector<ChargerGraphEdge> &edges,
                      std::vector<Charger> & chargers)
{
	storage::tar::FileReader reader(path, storage::tar::FileReader::VerifyFingerprint);
	auto number_of_nodes = reader.ReadElementCount64("/charger_graph/edges");
	edges.resize(number_of_nodes);
	auto index = 0;
	auto decode_edges = [&](const auto &current_edge) {
		edges[index] = current_edge;
		index++;
	};
	reader.ReadStreaming<ChargerGraphEdge>("/charger_graph/edges",boost::make_function_output_iterator(decode_edges));

	readChargers(path, chargers);
//
//	//writer.WriteElementCount64("/extractor/chargers", chargers.size());
//	auto number_of_chargers = reader.ReadElementCount64("/charger_graph/chargers");
//	chargers.reserve(number_of_chargers);
//	auto decode_charger_dtos = [&] (const auto & current_charger_dto) {
//		auto new_charger = Charger(
//				current_charger_dto.node_id,
//				current_charger_dto.phantom_node,
//				current_charger_dto.coordinate,
//				"",
//				current_charger_dto.total_power_in_milli_w,
//				current_charger_dto.has_fast_charging,
//				std::vector<Plug>());
//		chargers.push_back(new_charger);
//	};
//
//	reader.ReadStreaming<ChargerDto>("/charger_graph/chargers",boost::make_function_output_iterator(decode_charger_dtos));
}




void readEdges(const boost::filesystem::path & path, std::shared_ptr<std::vector<ChargerGraphEdge>> & edges) {
	storage::tar::FileReader reader(path, storage::tar::FileReader::VerifyFingerprint);
	auto number_of_nodes = reader.ReadElementCount64("/charger_graph/edges");
	edges->resize(number_of_nodes);
	auto index = 0;
	auto decode_edges = [&](const auto &current_edge) {
		(*edges)[index] = current_edge;
		index++;
	};
	reader.ReadStreaming<ChargerGraphEdge>("/charger_graph/edges",boost::make_function_output_iterator(decode_edges));

//	std::sort(edges->begin(), edges->end());
}
}
}
}
