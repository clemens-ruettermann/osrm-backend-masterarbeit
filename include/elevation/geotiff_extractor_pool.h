//
// Created by kenspeckle on 3/7/22.
//

#ifndef OSRM_GEOTIFF_EXTRACTOR_POOL_H
#define OSRM_GEOTIFF_EXTRACTOR_POOL_H

#include <vector>
#include <memory>
#include <mutex>
#include "elevation/geotiff_extractor.hpp"

namespace osrm {

namespace elevation {


class GeotiffExtractorPool {
public:
	explicit GeotiffExtractorPool(const unsigned int pool_size, const std::vector<std::string> filenames);

	std::unique_ptr<GeotiffExtractor> get_free_extractor();
	void relese_back(std::unique_ptr<GeotiffExtractor> extractor);

private:
	const unsigned int pool_size;
	const std::vector<std::string> filenames;
	std::vector<std::unique_ptr<GeotiffExtractor>> extractors;
	std::mutex mutex_;
};


}
}

#endif //OSRM_GEOTIFF_EXTRACTOR_POOL_H

