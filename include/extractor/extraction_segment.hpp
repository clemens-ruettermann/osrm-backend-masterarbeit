#ifndef OSRM_EXTRACTION_SEGMENT_HPP
#define OSRM_EXTRACTION_SEGMENT_HPP

#include <util/coordinate.hpp>

namespace osrm
{
namespace extractor
{

// Note consumption will not be calculated in lua and as such it is not necessary to add it here
struct ExtractionSegment
{
    ExtractionSegment(const osrm::util::Coordinate source_,
                      const osrm::util::Coordinate target_,
                      double distance_,
                      double weight_,
                      double duration_)
        : source(source_), target(target_), distance(distance_), weight(weight_),
          duration(duration_)
    {
    }

    const osrm::util::Coordinate source;
    const osrm::util::Coordinate target;
    const double distance;
    double weight;
    double duration;
};
} // namespace extractor
} // namespace osrm

#endif
