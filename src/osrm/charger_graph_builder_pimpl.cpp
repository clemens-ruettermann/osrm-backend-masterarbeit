
#include "osrm/charger_graph_builder_pimpl.hpp"
#include "charger_graph/charger_graph_builder.hpp"
namespace osrm
{
namespace enav
{
struct ChargerGraphBuilderConfig;

} // namespace extractor


// Pimpl-like facade

void build_charger_graph(const enav::ChargerGraphBuilderConfig &config) {
	enav::ChargerGraphBuilder(config).Run();
}

} // namespace osrm
