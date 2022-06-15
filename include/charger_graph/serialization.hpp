//
// Created by kenspeckle on 3/28/22.
//

#ifndef OSRM_SERIALIZATION_H
#define OSRM_SERIALIZATION_H


#include "util/serialization.hpp"

#include "storage/serialization.hpp"
#include "storage/tar.hpp"
#include "charger_graph_edge.hpp"
#include <charger_graph/charger.hpp>

namespace osrm
{
namespace contractor
{
namespace serialization
{

//void write(storage::tar::FileWriter &writer,
//                  const std::string &name,
//                  const std::vector<ChargerGraphEdge> &edges, const std::vector<Charger> & chargers)
//{
//
//	storage::serialization::write(writer, name + "/edges", edges);
//	storage::serialization::write(writer, name + "/chargers", chargers);
//}
//
//
//void read(storage::tar::FileReader &reader,
//                 const std::string &name,
//                 const std::vector<ChargerGraphEdge> &edges, const std::vector<Charger> & chargers)
//{
//	edges.resize(reader.ReadElementCount64(name));
////	storage::serialization::read(reader, edges);
////	storage::serialization::read(reader, name + "/edges", edges);
////	storage::serialization::read(reader, name + "/nodes", chargers);
//}


} // namespace serialization
} // namespace contractor
} // namespace osrm

#endif //OSRM_SERIALIZATION_H
