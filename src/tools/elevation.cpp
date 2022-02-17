#include "osrm/exception.hpp"
#include "util/log.hpp"
#include "util/version.hpp"

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <cstdlib>
#include <exception>
#include <new>
#include <thread>
#include <gdal.h>
#include <gdal_priv.h>

#include "util/meminfo.hpp"
#include "elevation/elevation_config.hpp"
#include "osrm/elevator.hpp"

using namespace osrm;

enum class return_code : unsigned
{
	ok,
	fail,
	exit
};

return_code parseArguments(int argc,
                           char *argv[],
                           std::string &verbosity,
                           elevation::ElevationConfig &elevation_config) {
	// declare a group of options that will be allowed only on command line
	boost::program_options::options_description generic_options("Options");
	generic_options.add_options()("version,v", "Show version")("help,h", "Show this help message")(
		"verbosity,l",
		boost::program_options::value<std::string>(&verbosity)->default_value("INFO"),
		std::string("Log verbosity level: " + util::LogPolicy::GetLevels()).c_str());

	// declare a group of options that will be allowed both on command line
	boost::program_options::options_description config_options("Configuration");
	config_options.add_options()(
		"data_version,d",
		boost::program_options::value<std::string>(&elevation_config.data_version)
			->default_value(""),
		"Data version. Leave blank to avoid. osmosis - to get timestamp from file")(
		"threads,t",
		boost::program_options::value<unsigned int>(&elevation_config.requested_num_threads)
			->default_value(std::thread::hardware_concurrency()),
		"Number of threads to use")(
		"geotiff",
		boost::program_options::value<std::vector<boost::filesystem::path>>(&elevation_config.elevation_geotiffs),
		"Path to geotiff to be used to extract elevation data (This option can be given multiple times to specify multiple geotiffs)"
	);

	bool dummy;
	// hidden options, will be allowed on command line, but will not be
	// shown to the user
	boost::program_options::options_description hidden_options("Hidden options");
	hidden_options.add_options()(
		"input,i",
		boost::program_options::value<boost::filesystem::path>(&elevation_config.input_path),
		"Input file in .osm, .osm.bz2 or .osm.pbf format");

	// positional option
	boost::program_options::positional_options_description positional_options;
	positional_options.add("input", 1);

	// combine above options for parsing
	boost::program_options::options_description cmdline_options;
	cmdline_options.add(generic_options).add(config_options).add(hidden_options);

	const auto *executable = argv[0];
	boost::program_options::options_description visible_options(
		boost::filesystem::path(executable).filename().string() +
			" <input.osm/.osm.bz2/.osm.pbf> [options]");
	visible_options.add(generic_options).add(config_options);

	// parse command line options
	boost::program_options::variables_map option_variables;
	try
	{
		boost::program_options::store(boost::program_options::command_line_parser(argc, argv)
			                              .options(cmdline_options)
			                              .positional(positional_options)
			                              .run(),
		                              option_variables);
	}
	catch (const boost::program_options::error &e)
	{
		util::Log(logERROR) << e.what();
		return return_code::fail;
	}

	if (option_variables.count("version"))
	{
		std::cout << OSRM_VERSION << std::endl;
		return return_code::exit;
	}

	if (option_variables.count("help"))
	{
		std::cout << visible_options;
		return return_code::exit;
	}

	boost::program_options::notify(option_variables);

	if (!option_variables.count("input"))
	{
		std::cout << visible_options;
		return return_code::exit;
	}

	// Set elevation_geotiffs_string to avoid having to call to string more than once
	// Also we assume, that the values will not change during the execution
	elevation_config.set_geotiff_paths_as_string();
	return return_code::ok;
}



int main(int argc, char *argv[]) {
	try {
		util::LogPolicy::GetInstance().Unmute();
		// We avoid using GDALALLRegister() as we only need GeoTiff, so hopefully this will speed up things a little bit
		GetGDALDriverManager();
		GDALDriverManager::AutoLoadDrivers();
		GDALRegister_GTiff();

		elevation::ElevationConfig elevation_config;
		std::string verbosity;

		const auto result = parseArguments(argc, argv, verbosity, elevation_config);
		if (return_code::fail == result) {
			return EXIT_FAILURE;
		}
		if (return_code::exit == result) {
			return EXIT_SUCCESS;
		}
		util::LogPolicy::GetInstance().SetLevel(verbosity);
		elevation_config.UseDefaultOutputNames(elevation_config.input_path);

		if (1 > elevation_config.requested_num_threads) {
			util::Log(logERROR) << "Number of threads must be 1 or larger";
			return EXIT_FAILURE;
		}

		if (!boost::filesystem::is_regular_file(elevation_config.input_path)) {
			util::Log(logERROR) << "Input file " << elevation_config.input_path.string() << " not found!";
			return EXIT_FAILURE;
		}

		for (auto &it : elevation_config.elevation_geotiffs) {
			if (!boost::filesystem::is_regular_file(it)) {
				util::Log(logERROR) << "Geotiff " << it.string() << " not found!";
				return EXIT_FAILURE;
			}
		}
		osrm::extract_elevation(elevation_config);
		GDALDumpOpenDatasets(stderr);
		GDALDestroyDriverManager();
		util::DumpMemoryStats();
		return EXIT_SUCCESS;
	}
	catch (const osrm::RuntimeError &e) {
		util::DumpMemoryStats();
		util::Log(logERROR) << e.what();
		return e.GetCode();
	}
	catch (const std::system_error &e) {
		util::DumpMemoryStats();
		util::Log(logERROR) << e.what();
		return e.code().value();
	}
	catch (const std::bad_alloc &e) {
		util::DumpMemoryStats();
		util::Log(logERROR) << "[exception] " << e.what();
		util::Log(logERROR) << "Please provide more memory or consider using a larger swapfile";
		return EXIT_FAILURE;
	}
}
