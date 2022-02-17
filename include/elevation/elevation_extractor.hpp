//
// Created by kenspeckle on 3/1/22.
//

#ifndef OSRM_HEIGHTEXTRACTOR_HPP
#define OSRM_HEIGHTEXTRACTOR_HPP

#include <string>
#include <gdal_priv.h>
#include "util/coordinate.hpp"
#include "elevation_config.hpp"

namespace osrm {
namespace elevation{
class ElevationExtractor {
	public:
		explicit ElevationExtractor(ElevationConfig elevation_config);
		~ElevationExtractor() = default;

		int run();
private:

	ElevationConfig config;

};
} // namespace extractor
} // namespace osrm

#endif //OSRM_HEIGHTEXTRACTOR_HPP
