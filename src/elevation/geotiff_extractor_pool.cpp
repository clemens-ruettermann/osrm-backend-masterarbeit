//
// Created by kenspeckle on 3/7/22.
//

#include "elevation/geotiff_extractor_pool.h"

namespace osrm {

namespace elevation {


osrm::elevation::GeotiffExtractorPool::GeotiffExtractorPool(const unsigned int pool_size_, const std::vector<std::string> filenames_) : pool_size(pool_size_), filenames(filenames_) {
	std::lock_guard<std::mutex> lg{mutex_};
	for(size_t i = 0; i < pool_size_; i++) {
		extractors.emplace_back(std::make_unique<GeotiffExtractor>(filenames));
	}
}

std::unique_ptr<GeotiffExtractor> osrm::elevation::GeotiffExtractorPool::get_free_extractor() {
	std::lock_guard<std::mutex> lg{mutex_};
	for (size_t i = 0; i < pool_size; i++) {
		if (extractors.at(i)) {
			return std::move(extractors[i]);
		}
	}
	throw std::runtime_error{"Unable to find a free geotiff extractor instance. Try increasing the pool size"};
}

void GeotiffExtractorPool::relese_back(std::unique_ptr<GeotiffExtractor> extractor) {
	std::lock_guard<std::mutex> lg{mutex_};
	for (size_t i = 0; i < pool_size; i++) {
		if (!extractors.at(i)) {
			extractors[i] = std::move(extractor);
			break;
		}
	}
}

}
}
