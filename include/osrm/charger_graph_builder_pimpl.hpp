#ifndef OSRM_CHARGER_GRAPH_BUILDER_PIMPL_HPP
#define OSRM_CHARGER_GRAPH_BUILDER_PIMPL_HPP

namespace osrm
{
namespace enav
{
struct ChargerGraphBuilderConfig;

} // namespace extractor

/**
 * Build the charger graph.
 *
 * \param config The user-provided extraction configuration.
 * \throws osrm::util::exception, osmium::io_error
 * \see  ChargerGraphBuilderConfig
 */
void build_charger_graph(const enav::ChargerGraphBuilderConfig &config);

} // namespace osrm

#endif // OSRM_CHARGER_GRAPH_BUILDER_PIMPL_HPP
