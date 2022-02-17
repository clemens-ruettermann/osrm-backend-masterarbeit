//
// Created by kenspeckle on 3/1/22.
//
#include "elevation/elevation_extractor.hpp"
#include "util/log.hpp"
#include "util/debug.hpp"
#include "util/timing_util.hpp"
#include "util/typedefs.hpp"
#include "util/exception_utils.hpp"
#include "storage/tar.hpp"
#include "storage/serialization.hpp"
#include "elevation/elevation_node.hpp"
#include "elevation/geotiff_extractor.hpp"
#include "elevation/geotiff_extractor_pool.h"

#include <gdal_priv.h>
#include <thread>
// Used for Version lookup
#include <tbb/tbb_stddef.h>
#if TBB_VERSION_MAJOR == 2020
#include <tbb/global_control.h>
#else
#include <tbb/task_scheduler_init.h>
#endif

#include <osmium/io/file.hpp>
#include <osmium/thread/pool.hpp>
#include <osmium/io/reader.hpp>
#include <osmium/handler/node_locations_for_ways.hpp>
#include <osmium/index/map/flex_mem.hpp>
#include <osmium/io/any_input.hpp>

#include <tbb/parallel_for.h>
#include <tbb/pipeline.h>



namespace osrm {
namespace elevation {

ElevationExtractor::ElevationExtractor(ElevationConfig elevation_config) :
		config(std::move(elevation_config)) {

}


int ElevationExtractor::run() {
	util::LogPolicy::GetInstance().Unmute();

	const unsigned recommended_num_threads = std::thread::hardware_concurrency();
	const int number_of_threads = std::min(recommended_num_threads, config.requested_num_threads);

#if TBB_VERSION_MAJOR == 2020
	tbb::global_control gc(tbb::global_control::max_allowed_parallelism,
	config.requested_num_threads);
#else
	tbb::task_scheduler_init init(config.requested_num_threads);
    BOOST_ASSERT(init.is_active());
#endif

	TIMER_START(extracting);

	util::Log() << "Input file: " << config.input_path.filename().string();
	util::Log() << "Threads: " << number_of_threads;

	const osmium::io::File input_file(config.input_path.string());
	osmium::thread::Pool pool(number_of_threads);

	util::Log() << "Reading in progress..";
	TIMER_START(reading);

	{ // Parse OSM header
		osmium::io::Reader reader(input_file, pool, osmium::osm_entity_bits::nothing);
		osmium::io::Header header = reader.header();

		std::string generator = header.get("generator");
		if (generator.empty()) {
			generator = "unknown tool";
		}
		util::Log() << "input file generated by " << generator;

		// write .timestamp data file
		std::string timestamp = header.get("osmosis_replication_timestamp");
		util::Log() << "timestamp: " << timestamp;
	}

	// OSM data reader
	using SharedBuffer = std::shared_ptr<osmium::memory::Buffer>;
	struct ParsedBuffer{
		SharedBuffer buffer;
		std::vector<std::pair<OSMNodeID, float>> resulting_nodes;
	};

	// Parse OSM elements with parallel transformer
	// Number of pipeline tokens that yielded the best speedup was about 1.5 * num_cores
	const unsigned int num_threads = std::lround(std::thread::hardware_concurrency() * 1.5);
	const auto read_meta = osmium::io::read_meta::no;

	osmium::io::Reader reader(input_file,
	                          pool,
	                          osmium::osm_entity_bits::node,
	                          read_meta);

	TIMER_STOP(reading);
	util::Log() << "Parsing finished after " << TIMER_SEC(reading) << " seconds";

	TIMER_START(elevation_time);
	const auto buffer_reader = [](osmium::io::Reader &reader) {
		return tbb::filter_t<void, SharedBuffer>(
				tbb::filter::serial_in_order, [&reader](tbb::flow_control &fc) {
					if (auto buffer = reader.read())
					{
						return std::make_shared<osmium::memory::Buffer>(std::move(buffer));
					}
					else
					{
						fc.stop();
						return SharedBuffer{};
					}
				});
	};

	std::mutex m;
	std::shared_ptr<GeotiffExtractor> geotiff_extractor = std::make_shared<GeotiffExtractor>(config.elevation_geotiffs_path_as_string);
//	GeotiffExtractorPool extractor_pool{std::thread::hardware_concurrency() * 2, config.elevation_geotiffs_path_as_string};
	tbb::filter_t<SharedBuffer, ParsedBuffer> buffer_transformer(
			tbb::filter::parallel, [&](const SharedBuffer& buffer) {
				ParsedBuffer parsed_buffer;
				parsed_buffer.buffer = buffer;
				for (auto entity = buffer->cbegin(), end = buffer->cend(); entity != end; ++entity){
					if (entity->type() == osmium::item_type::node) {
						const auto &node = static_cast<const osmium::Node &>(*entity);
						const auto lon = node.location().lon();
						const auto lat = node.location().lat();
						float elevation_val = 0;
						try {
							auto geotiff_coord = geotiff_extractor->get_pixel_coordinate(lon, lat);
							elevation_val = geotiff_extractor->get_value(geotiff_coord);
						} catch (std::runtime_error & e) {
//							util::Log(logINFO) << "Coords lie outside of the geotiff. Please provide additional geotiffs: " + std::to_string(lon) + ", " + std::to_string(lat);
						}
						auto node_id = OSMNodeID{static_cast<std::uint64_t>(node.id())};
						parsed_buffer.resulting_nodes.emplace_back(node_id, elevation_val);
					}
				}
				return parsed_buffer;
			});

	std::vector<ElevationNode> elevation_nodes;
	std::vector<OSMNodeID> id_vector;
	std::vector<float> value_vector;

	tbb::filter_t<ParsedBuffer, void> buffer_storage(
			tbb::filter::serial_in_order, [&](const ParsedBuffer &parsed_buffer) {
				for (const auto & it : parsed_buffer.resulting_nodes) {
					elevation_nodes.emplace_back(it.first, it.second);
					id_vector.emplace_back(it.first);
					value_vector.emplace_back(it.second);
				}
 			});
	auto pipeline =
			buffer_reader(reader)
			& buffer_transformer
			& buffer_storage;
	tbb::parallel_pipeline(num_threads,  pipeline);

	TIMER_STOP(elevation_time);
	util::Log() << "Done calculating the elevation for all nodes. This took " << TIMER_SEC(elevation_time) << "s" << std::flush;

	util::Log() << "Sort vector" << std::flush;
	tbb::parallel_sort(elevation_nodes.begin(), elevation_nodes.end());
	util::Log() << "Write elevation data" << std::flush;

	const auto fingerprint = storage::tar::FileWriter::GenerateFingerprint;
	storage::tar::FileWriter writer{config.GetPath(".osrm.elevation"), fingerprint};
	storage::serialization::write(writer, "/common/elevation/ids", id_vector);
	storage::serialization::write(writer, "/common/elevation/values", value_vector);

	TIMER_STOP(extracting);
	util::Log() << "extraction finished after " << TIMER_SEC(extracting) << "s";


	return 0;
}






}   // namespace osrm
}   // namespace elevation