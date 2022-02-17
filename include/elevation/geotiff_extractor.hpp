//
// Created by kenspeckle on 3/7/22.
//

#ifndef OSRM_GEOTIFF_EXTRACTOR_HPP
#define OSRM_GEOTIFF_EXTRACTOR_HPP

#include <vector>
#include <string>
#include <gdal_priv.h>
#include "geotiff_value_matrix.h"


namespace osrm {

namespace elevation {



struct GeotiffPixelCoordinate {

	constexpr static const double INVALID_LON = 361;
	constexpr static const double INVALID_LAT = 91;
	size_t band;
	int y;
	int x;
	double lon;
	double lat;

	GeotiffPixelCoordinate() : band(0), x(-1), y(-1), lon(INVALID_LON), lat(INVALID_LAT) {}
	GeotiffPixelCoordinate(const size_t band, const int x, const int y) : band(band), x(x), y(y), lon(INVALID_LON), lat(INVALID_LAT) {}
	GeotiffPixelCoordinate(const double lon, const double lat, const size_t band, const int x, const int y) : lon(lon), lat(lat), band(band), x(x), y(y) {}

	GeotiffPixelCoordinate(const GeotiffPixelCoordinate & o) : band(o.band), y(o.y), x(o.x), lon(o.lon), lat(o.lat) {}

	std::string to_string() const {
		return "x: " + std::to_string(x) + "\ty: " + std::to_string(y) + "\tband: " + std::to_string(band) + "\tlon: " + std::to_string(lon) + "\tlat: " + std::to_string(lat);
	}

	bool geo_is_valid() const {
		return (lon != INVALID_LON && lon <=180 && lon >= -180) && (lat != INVALID_LAT && lat <= 90 && lat >= -90);
	}
};

class GeotiffExtractor {
public:
	explicit GeotiffExtractor(const std::vector<std::string> & filenames);
	~GeotiffExtractor();

	GeotiffPixelCoordinate get_pixel_coordinate(double lon, double lat) const;
	[[nodiscard]] bool is_within(const GeotiffPixelCoordinate & coordinate) const;
	[[nodiscard]] float get_value(const GeotiffPixelCoordinate & coordinate) const;

	void set_coords_to_pixel_center(GeotiffPixelCoordinate & coord) const;
	float get_pixel_width_for_band(size_t band) const;
	float get_pixel_height_for_band(size_t band) const;

	[[nodiscard]] float get_value_for_pixel(const GeotiffPixelCoordinate & coord) const;
	[[nodiscard]] GeotiffValueMatrix<float, 3> get_matrix_values_for_center_pixel(const GeotiffPixelCoordinate & coord) const;
	[[nodiscard]] bool pixel_coord_is_valid(const GeotiffPixelCoordinate & coord) const;

	std::pair<double, double> pixel_coord_to_wgs84(unsigned int x, unsigned int y, unsigned int band) const;
	std::pair<double, double> pixel_coord_to_internal_srs(unsigned int x, unsigned int y, unsigned int band) const;
private:
	std::vector<std::string> filenames;
	std::vector<GDALDataset *> datasets;
	std::vector<OGRSpatialReference *> source_spatial_reference_systems;
//	OGRCoordinateTransformation * inverse_coordinate_transformations;
	std::vector<double *> adfInvGeoTransforms;
	std::vector<double *> adfGeoTransforms;
	std::vector<OGRCoordinateTransformation *> coordinate_transformations;
	std::vector<OGRCoordinateTransformation *> inverse_coordinate_transformations;
	std::vector<GDALRasterBand *> bands{};
	size_t nbr_bands{0};

	float *** values;
	std::vector<std::pair<size_t, size_t>> dimensions;

};


}
}

#endif //OSRM_GEOTIFF_EXTRACTOR_HPP
