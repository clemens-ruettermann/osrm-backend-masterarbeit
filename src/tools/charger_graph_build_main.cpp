#include "osrm/exception.hpp"
#include "osrm/charger_graph_builder_pimpl.hpp"
#include "util/log.hpp"
#include "util/version.hpp"

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <cstdlib>
#include <new>
#include <thread>

#include "util/meminfo.hpp"
#include "charger_graph/charger_graph_builder_config.hpp"

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
                           osrm::enav::ChargerGraphBuilderConfig &config)
{
	// declare a group of options that will be a llowed only on command line
	boost::program_options::options_description generic_options("Options");
	generic_options.add_options()("version,v", "Show version")("help,h", "Show this help message")(
			"verbosity,l",
			boost::program_options::value<std::string>(&verbosity)->default_value("INFO"),
			std::string("Log verbosity level: " + util::LogPolicy::GetLevels()).c_str());

	// declare a group of options that will be allowed both on command line
	boost::program_options::options_description config_options("Configuration");
	config_options.add_options()
			(
					"upper-limit",
					boost::program_options::value<double>(&config.upper_limit_percent)
							->default_value(90),
					"Upper limit for edges in percent of battery capacity. Defaults is 90")
			(
					"lower-limit",
					boost::program_options::value<double>(&config.lower_limit_percent)
							->default_value(50),
					"Lower limit for edges in percent of battery capacity. Must be smaller than upper limit. Default is 50")
			(
					"no-filter-plug-type",
					boost::program_options::bool_switch(&config.no_filter_plug_type)
							->default_value(false),
					"If specified do not remove chargers which do not share a plug-type with the given car. Default false => Remove charger without matching plug type");
	// hidden options, will be allowed on command line, but will not be shown to the user
	boost::program_options::options_description hidden_options("Hidden options");
	hidden_options.add_options()(
			"input,i",
			boost::program_options::value<boost::filesystem::path>(&config.base_path),
			"Input file in .osm, .osm.bz2 or .osm.pbf format");

	hidden_options.add_options()(
			"charger,c",
			boost::program_options::value<boost::filesystem::path>(&config.charger_csv_path),
			"Charger CSV file");

	// positional option
	boost::program_options::positional_options_description positional_options;
	positional_options.add("input", 1);
	positional_options.add("charger", 1);

	// combine above options for parsing
	boost::program_options::options_description cmdline_options;
	cmdline_options.add(generic_options).add(config_options).add(hidden_options);

	const auto *executable = argv[0];
	boost::program_options::options_description visible_options(
			"Usage: " + boost::filesystem::path(executable).filename().string() +
			" <input.osrm> <charger.csv> [options]");
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

	if (!option_variables.count("charger"))
	{
		std::cout << visible_options;
		return return_code::exit;
	}

	config.engine_config.storage_config = {config.base_path};
	config.engine_config.algorithm = engine::EngineConfig::Algorithm::CH;
	config.engine_config.use_shared_memory = false;

	return return_code::ok;
}

int main(int argc, char *argv[])
try
{
	util::LogPolicy::GetInstance().Unmute();
	osrm::enav::ChargerGraphBuilderConfig config;
	std::string verbosity;

	const auto result = parseArguments(argc, argv, verbosity, config);

	if (return_code::fail == result)
	{
		return EXIT_FAILURE;
	}

	if (return_code::exit == result)
	{
		return EXIT_SUCCESS;
	}

	util::LogPolicy::GetInstance().SetLevel(verbosity);


	if (!boost::filesystem::is_regular_file(config.base_path))
	{
		util::Log(logERROR) << "Input file " << config.base_path.string()
		                    << " not found!";
		return EXIT_FAILURE;
	}


	osrm::build_charger_graph(config);

	util::DumpMemoryStats();

	return EXIT_SUCCESS;
}
catch (const osrm::RuntimeError &e)
{
	util::DumpMemoryStats();
	util::Log(logERROR) << e.what();
	return e.GetCode();
}
catch (const std::system_error &e)
{
	util::DumpMemoryStats();
	util::Log(logERROR) << e.what();
	return e.code().value();
}
catch (const std::bad_alloc &e)
{
	util::DumpMemoryStats();
	util::Log(logERROR) << "[exception] " << e.what();
	util::Log(logERROR) << "Please provide more memory or consider using a larger swapfile";
	return EXIT_FAILURE;
}
