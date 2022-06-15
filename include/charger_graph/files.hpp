//
// Created by kenspeckle on 3/28/22.
//

#ifndef OSRM_FILES_H
#define OSRM_FILES_H

#include <json.hpp>
#include "charger.hpp"
#include "charger_graph_edge.hpp"

namespace osrm {
namespace enav {
namespace files {

nlohmann::json chargerToJson(const Charger & charger);
Charger chargerFromJson(const nlohmann::json & tmp_j);
void writeChargers(const boost::filesystem::path &path, const std::vector<Charger> & chargers);
void writeEdges(const boost::filesystem::path &path, const std::vector<ChargerGraphEdge> &edges);
void writeChargerGraph(const boost::filesystem::path &path,const std::vector<ChargerGraphEdge> &edges,const std::vector<Charger> & chargers);
void readChargers(const boost::filesystem::path & path, std::vector<Charger> & chargers) ;

// reads .osrm.charger_graph file
void readChargerGraph(const boost::filesystem::path &path,std::vector<ChargerGraphEdge> &edges,std::vector<Charger> & chargers);
void readEdges(const boost::filesystem::path & path, std::shared_ptr<std::vector<ChargerGraphEdge>> & edges);
}
}
}


#endif //OSRM_FILES_H
