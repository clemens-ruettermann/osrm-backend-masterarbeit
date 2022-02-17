#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include "util/timing_util.hpp"
#include "geotiff_extractor.cpp"

using namespace osrm;

void calc_diff_for_dataset(elevation::GeotiffExtractor & extractor, const std::string & dataset) {
	std::ofstream out{"/home/kenspeckle/git/fh-aachen/masterarbeit/thesis/data/Testfahrt/" + dataset +"/diff_elevation.csv"};
	out << "gps,calc,diff" << std::endl;


	std::ifstream in{"/home/kenspeckle/git/fh-aachen/masterarbeit/thesis/data/Testfahrt/" + dataset + "/GPS_ELEVATION.csv"};
	std::string line, lon_str, lat_str, sec_str, unit_str, value_str;
	std::getline(in, line);
	while(std::getline(in, line)) {
		std::stringstream ss{line};
		std::getline(ss, sec_str, ',');
		std::getline(ss, value_str, ',');
		std::getline(ss, unit_str, ',');
		std::getline(ss, lon_str, ',');
		std::getline(ss, lat_str, ',');

		auto coord_center = extractor.get_pixel_coordinate(std::stod(lon_str), std::stod(lat_str));
		auto val_center = extractor.get_value(coord_center);
		std::cout << "GPS Elevation: " << value_str << "\tCalculated Elevation: " << val_center << "\tdiff: " << (std::stod(value_str) - val_center) << std::endl;

		out << value_str << "," << val_center << "," << (std::stod(value_str) - val_center) << std::endl;
	}

}

int main() {

	util::LogPolicy::GetInstance().Unmute();
	GDALAllRegister();
	GDALDriverManager::AutoLoadDrivers();

	TIMER_START(global);

	std::vector<std::string> geotiff_paths{"/home/kenspeckle/git/fh-aachen/masterarbeit/data/eu_dem_v11_E40N30.TIF"};
	elevation::GeotiffExtractor extractor = elevation::GeotiffExtractor(geotiff_paths);
	TIMER_STOP(global);
	util::Log() << "Loading took " << TIMER_SEC(global) << " sec";

	calc_diff_for_dataset(extractor, "simmerath_aachen");
	calc_diff_for_dataset(extractor, "aachen_simmerath");

}