#include "osrm/elevator.hpp"
#include "elevation/elevation_extractor.hpp"
#include "elevation/elevation_config.hpp"

namespace osrm
{

// Pimpl-like facade

void extract_elevation(const elevation::ElevationConfig &config)
{
    elevation::ElevationExtractor(config).run();
}

} // namespace osrm
