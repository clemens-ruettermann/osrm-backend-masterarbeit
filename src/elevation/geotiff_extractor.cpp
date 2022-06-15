//
// Created by kenspeckle on 3/7/22.
//

#include "elevation/geotiff_extractor.hpp"
#include "util/coordinate_calculation.hpp"
#include "util/log.hpp"
#include <cmath>

namespace osrm {

namespace elevation {

GeotiffExtractor::GeotiffExtractor(const std::vector<std::string> &filenames) {
	values = new float**[filenames.size()];
	for (size_t i = 0; i < filenames.size(); i++) {
		auto & filename = filenames[i];
		auto *new_dataset = (GDALDataset *) GDALOpen(filename.c_str(), GA_ReadOnly);
		if (new_dataset == nullptr) {
			throw std::runtime_error{"Unable to load geotiff"};
		}
		this->datasets.emplace_back(new_dataset);

		auto tmp_source_spatial_reference_system = new OGRSpatialReference{};
		tmp_source_spatial_reference_system->importFromEPSG(4326); //WGS84
//		tmp_source_spatial_reference_system->SetFromUserInput();
		tmp_source_spatial_reference_system->SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
		source_spatial_reference_systems.emplace_back(tmp_source_spatial_reference_system);

		const OGRSpatialReference *target_spacial_reference_system = new_dataset->GetSpatialRef();
		if (target_spacial_reference_system == nullptr) {
			throw std::runtime_error{"Target spacial reference system is nullptr"};
		}
		auto tmp_coordinate_transformation = OGRCreateCoordinateTransformation(tmp_source_spatial_reference_system,target_spacial_reference_system);
		if (tmp_coordinate_transformation == nullptr) {
			throw std::runtime_error{"Coordinate transformation is nullptr"};
		}
		coordinate_transformations.emplace_back(tmp_coordinate_transformation);

		auto tmp_inverse_coordinate_transformation = OGRCreateCoordinateTransformation(
			target_spacial_reference_system, tmp_source_spatial_reference_system);
		if (tmp_inverse_coordinate_transformation == nullptr) {
			throw std::runtime_error{"inverse coordinate transformation is nullptr"};
		}
		inverse_coordinate_transformations.emplace_back(tmp_inverse_coordinate_transformation);



		if (new_dataset->GetRasterCount() != 1) {
			throw std::runtime_error{"Currently only geotiffs with one band are supported"};
		}
		auto tmp_band = new_dataset->GetRasterBand(1);
		if (tmp_band == nullptr) {
			throw std::runtime_error{"Band is null"};
		}
		bands.emplace_back(tmp_band);

		auto * adfGeoTransform = new double[6];
		adfGeoTransforms.push_back(adfGeoTransform);
		if (new_dataset->GetGeoTransform(adfGeoTransform) != CE_None) {
			throw std::runtime_error{"Cannot get geotransform for dataset " + filename};
		}

		auto *tmp_adf_inv_geo_transform = new double[6];
		adfInvGeoTransforms.push_back(tmp_adf_inv_geo_transform);
		if (!GDALInvGeoTransform(adfGeoTransform, tmp_adf_inv_geo_transform)) {
			throw std::runtime_error{"Cannot invert geotransform"};
		}

		auto x_size = new_dataset->GetRasterXSize();
		auto y_size = new_dataset->GetRasterYSize();

		dimensions.emplace_back(x_size, y_size);
		values[i] = new float*[y_size];

		for (size_t j = 0; j < y_size; j++) {
			values[i][j] = new float[x_size];
		}

		util::Log() << "Loading geotiff values into RAM ...";
		auto no_data_value = new_dataset->GetRasterBand(1)->GetNoDataValue();
		float adfPixel[2];
		for (size_t j = 0; j < y_size; j++) {
			if (y_size > 10 && j % (y_size/10) == 0) {
				util::Log() << "Row: " << j << "/" << y_size;
			}
			for (size_t k = 0; k < x_size; k++) {
				if (bands[i]->RasterIO(GF_Read, k, j, 1, 1, adfPixel, 1, 1, GDT_CFloat32, 0, 0) != CE_None) {
					throw std::runtime_error{"Unable to get value for " + std::to_string(k) + "," + std::to_string(j)};
				}
				if (adfPixel[0] != no_data_value) {
					values[i][j][k] = adfPixel[0];
				} else {
					values[i][j][k] = 0;
				}
			}
		}

		nbr_bands++;
	}
}


GeotiffExtractor::~GeotiffExtractor() {
	for (size_t i = 0; i < nbr_bands; i++) {
		GDALClose(datasets[i]);
		delete source_spatial_reference_systems[i];
		delete coordinate_transformations[i];
		delete inverse_coordinate_transformations[i];
		delete[] adfInvGeoTransforms[i];
		delete[] adfGeoTransforms[i];
		auto dimension = dimensions[i];
		for (size_t j = 0; j < dimension.second; j++) {
			delete [] values[i][j];
		}
		delete [] values[i];
	}
	delete [] values;
}

bool GeotiffExtractor::is_within(const GeotiffPixelCoordinate & coordinate) const {
	return coordinate.x >= 0 && coordinate.y >= 0 && coordinate.x < datasets[coordinate.band]->GetRasterXSize() && coordinate.y < datasets[coordinate.band]->GetRasterYSize();
}


float GeotiffExtractor::get_value_for_pixel(const GeotiffPixelCoordinate & coord) const {
	return values[coord.band][coord.y][coord.x];
//	float adfPixel[2];
//	if (bands[coord.band]->RasterIO(GF_Read, coord.x, coord.y, 1, 1, adfPixel, 1, 1, GDT_CFloat32, 0, 0) != CE_None) {
//		throw std::runtime_error{"Unable to get value for " + std::to_string(coord.x) + "," + std::to_string(coord.y)};
//	}
//	auto ret = adfPixel[0];
//	auto no_data_value = datasets[coord.band]->GetRasterBand(1)->GetNoDataValue();
//	if (ret == no_data_value) {
//		return 0.0;
//	}
//	return ret;
}


bool GeotiffExtractor::pixel_coord_is_valid(const GeotiffPixelCoordinate & coord) const {
	return coord.band < this->nbr_bands && is_within(coord);
}

GeotiffValueMatrix<float, 3> GeotiffExtractor::get_matrix_values_for_center_pixel(const GeotiffPixelCoordinate & coord) const {
	if (!pixel_coord_is_valid(coord)) {
		throw std::runtime_error{"The center coordinate is invalid: " + coord.to_string()};
	}
	//TODO if there are multiple geotiffs specified and the value is at the edge try to get the value from the other geotiffs

	GeotiffValueMatrix<float, 3> ret{};
	ret.at(1, 1) = get_value_for_pixel(coord);

	// Is not at the edge or a corner
	if (coord.x > 0 && coord.x < datasets[coord.band]->GetRasterXSize() - 1 && coord.y > 0 && coord.y < datasets[coord.band]->GetRasterYSize() -1) {
		ret.at(0, 0) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x-1, coord.y-1));
		ret.at(1, 0) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x, coord.y-1));
		ret.at(2, 0) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x+1, coord.y-1));

		ret.at(0, 1) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x-1, coord.y));
		ret.at(2, 1) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x+1, coord.y));

		ret.at(0, 2) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x-1, coord.y+1));
		ret.at(1, 2) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x, coord.y+1));
		ret.at(2, 2) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x+1, coord.y+1));

	} else {
		if (coord.x == 0) {
			// Left edge/corner
			ret.at(2, 1) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x+1, coord.y));

			if (coord.y == 0) {
				// Upper left corner
				ret.at(2, 2) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x+1, coord.y+1));
				ret.at(1, 2) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x, coord.y+1));

				ret.at(0, 0) = ret.at(1, 1);
				ret.at(0, 1) = ret.at(1, 1);
				ret.at(0, 2) = ret.at(1, 2);
				ret.at(1, 0) = ret.at(1, 1);
				ret.at(2, 0) = ret.at(2, 1);

			} else if (coord.y == datasets[coord.band]->GetRasterYSize() -1) {
				// lower left corner
				ret.at(1, 0) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x, coord.y-1));
				ret.at(2, 0) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x+1, coord.y-1));

				ret.at(0, 0) = ret.at(1, 0);
				ret.at(0, 1) = ret.at(1, 1);
				ret.at(0, 2) = ret.at(1, 1);
				ret.at(1, 2) = ret.at(1, 1);
				ret.at(2, 2) = ret.at(2, 1);

			} else {
				// left edge
				ret.at(1, 0) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x, coord.y-1));
				ret.at(2, 0) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x+1, coord.y-1));
				ret.at(1, 2) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x, coord.y+1));
				ret.at(2, 2) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x+1, coord.y+1));

				ret.at(0, 0) = ret.at(1, 0);
				ret.at(0, 1) = ret.at(1, 1);
				ret.at(0, 2) = ret.at(1, 2);
			}


		} else if (coord.x == datasets[coord.band]->GetRasterXSize() - 1) {
			//right edge/corner
			ret.at(0, 1) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x-1, coord.y));


			if (coord.y == 0) {
				// upper right corner
				ret.at(0, 2) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x-1, coord.y+1));
				ret.at(1, 2) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x, coord.y+1));

				ret.at(0, 0) = ret.at(0, 1);
				ret.at(1, 0) = ret.at(1, 1);
				ret.at(2, 0) = ret.at(1, 1);
				ret.at(2, 1) = ret.at(1, 1);
				ret.at(2, 2) = ret.at(1, 2);


			} else if (coord.y == datasets[coord.band]->GetRasterYSize() - 1) {
				// lower right corner
				ret.at(0, 0) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x-1, coord.y-1));
				ret.at(1, 0) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x, coord.y-1));

				ret.at(2, 0) = ret.at(1, 0);
				ret.at(2, 1) = ret.at(1, 1);
				ret.at(2, 2) = ret.at(1, 1);
				ret.at(1, 2) = ret.at(1, 1);
				ret.at(0, 2) = ret.at(0, 1);

			} else {
				// right edge
				ret.at(0, 0) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x-1, coord.y-1));
				ret.at(1, 0) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x, coord.y-1));
				ret.at(0, 2) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x-1, coord.y+1));
				ret.at(1, 2) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x, coord.y+1));

				ret.at(2, 0) = ret.at(1, 0);
				ret.at(2, 1) = ret.at(1, 1);
				ret.at(2, 2) = ret.at(1, 2);
			}
		} else {
			// only upper or lower edge as x != 0 != xSize
			ret.at(0, 1) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x-1, coord.y));
			ret.at(2, 1) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x+1, coord.y));

			if (coord.y == 0) {
				// upper edge
				ret.at(0, 2) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x-1, coord.y+1));
				ret.at(1, 2) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x, coord.y+1));
				ret.at(2, 2) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x+1, coord.y+1));

				ret.at(0, 0) = ret.at(0, 1);
				ret.at(1, 0) = ret.at(1, 1);
				ret.at(2, 0) = ret.at(2, 1);

			} else if (coord.y == datasets[coord.band]->GetRasterYSize() -1) {
				// lower edge
				ret.at(0, 0) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x-1, coord.y-1));
				ret.at(1, 0) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x, coord.y-1));
				ret.at(2, 0) = get_value_for_pixel(GeotiffPixelCoordinate(coord.band, coord.x+1, coord.y-1));

				ret.at(0, 2) = ret.at(0, 1);
				ret.at(1, 2) = ret.at(1, 1);
				ret.at(2, 2) = ret.at(2, 1);
			} else {
				throw std::runtime_error{"You should not be here. Contact a programmer who can do something with this info"};
			}
		}
	}
	return ret;
}

void GeotiffExtractor::set_coords_to_pixel_center(GeotiffPixelCoordinate & coord) const {
	if (!pixel_coord_is_valid(coord)) {
		throw std::runtime_error{"The geo coords could not be set to the center of the pixel as the pixel coords are invalid: " + coord.to_string()};
	}
	auto point = pixel_coord_to_wgs84(coord.x, coord.y, coord.band);
	coord.lon = point.first;
	coord.lat = point.second;
}

static float get_gauss_value(const double & my_x, const double & my_y, const double & sigma_x, const double & sigma_y, const double & x, const double & y) {
	return exp(-(pow(x-my_x, 2) + pow(y-my_y, 2)) / (pow(sigma_x, 2) + pow(sigma_y, 2)));
}



float GeotiffExtractor::get_value(const GeotiffPixelCoordinate & coord) const {
	if (!pixel_coord_is_valid(coord) || !coord.geo_is_valid()) {
		throw std::runtime_error{"Unable to calculate gauss kernel as the pixel/geo coords are invalid: " + coord.to_string()};
	}

	const float pixel_width = get_pixel_width_for_band(coord.band);
	const float pixel_height = get_pixel_height_for_band(coord.band);

	double point_img_coord_x = coord.lon;
	double point_img_coord_y = coord.lat;
	if (!coordinate_transformations[coord.band]->Transform(1, &point_img_coord_x, &point_img_coord_y, nullptr)) {
		throw std::runtime_error{"Could not transform geo coords to image coords"};
	}
	auto tmp_gauss_func = [&](const double & x, const double & y) {
		return get_gauss_value(point_img_coord_x, point_img_coord_y, abs(pixel_width), abs(pixel_height), x, y);
	};

	auto geo_transf = adfGeoTransforms[coord.band];

	GeotiffValueMatrix<float, 3> gauss_kernel{};
	for (size_t i = 0; i < 3; i++) {
		for (size_t j = 0; j < 3; j++) {
			auto tmp_coord_pair = pixel_coord_to_internal_srs(coord.x -1 + i, coord.y - 1 + j, coord.band);
			auto pixel_center_lon = tmp_coord_pair.first;
			auto pixel_center_lat = tmp_coord_pair.second;
			gauss_kernel.at(i, j) = tmp_gauss_func(pixel_center_lon, pixel_center_lat);
		}
	}
	const auto sum_of_gauss_kernel = gauss_kernel.sum();
	gauss_kernel /= sum_of_gauss_kernel;

	auto value_matrix = get_matrix_values_for_center_pixel(coord);
	value_matrix.cellwise_multiply(gauss_kernel);
	return value_matrix.sum();
}


float GeotiffExtractor::get_pixel_width_for_band(const size_t band) const {
	return adfGeoTransforms[band][1];
}

float GeotiffExtractor::get_pixel_height_for_band(const size_t band) const {
	return adfGeoTransforms[band][5];
}

GeotiffPixelCoordinate GeotiffExtractor::get_pixel_coordinate(const double lon, const double lat) const {
	double tmp_lon, tmp_lat;
	GeotiffPixelCoordinate ret;
	for (size_t i = 0; i < nbr_bands; i++) {
		tmp_lon = lon;
		tmp_lat = lat;
		if (!coordinate_transformations[i]->Transform(1, &tmp_lon, &tmp_lat, nullptr)) {
			continue;
		}
		int iPixel = static_cast<int>(floor(adfInvGeoTransforms[i][0] + adfInvGeoTransforms[i][1] * tmp_lon + adfInvGeoTransforms[i][2] * tmp_lat));
		int iLine = static_cast<int>(floor(adfInvGeoTransforms[i][3] + adfInvGeoTransforms[i][4] * tmp_lon + adfInvGeoTransforms[i][5] * tmp_lat));
		ret = GeotiffPixelCoordinate{lon, lat, i, iPixel, iLine};
		if (is_within(ret)) {
			return ret;
		}
	}
	throw std::runtime_error{"Unable to build geotiff pixel coordinates"};
}

std::pair<double, double> GeotiffExtractor::pixel_coord_to_wgs84(unsigned int x, unsigned int y, unsigned int band) const {
	//https://gdal.org/tutorials/geotransforms_tut.html
	double current_pxl_x = x;
	double current_pxl_y = y;
	auto pixel_center_lon = (
		adfGeoTransforms[band][0]
		+ current_pxl_x * adfGeoTransforms[band][1]
		+ current_pxl_y * adfGeoTransforms[band][2]
		) + get_pixel_width_for_band(band)/2.0;
	auto pixel_center_lat = (
		adfGeoTransforms[band][3]
		+ current_pxl_x * adfGeoTransforms[band][4]
		+ current_pxl_y * adfGeoTransforms[band][5]
		) + get_pixel_height_for_band(band)/2.0;

	inverse_coordinate_transformations.at(band)->Transform(1, &pixel_center_lon, &pixel_center_lat);
	return std::make_pair(pixel_center_lon, pixel_center_lat);
}

std::pair<double, double> GeotiffExtractor::pixel_coord_to_internal_srs(
	unsigned int x, unsigned int y, unsigned int band) const {

	//https://gdal.org/tutorials/geotransforms_tut.html
	double current_pxl_x = x;
	double current_pxl_y = y;
	auto pixel_center_lon = (
		adfGeoTransforms[band][0]
			+ current_pxl_x * adfGeoTransforms[band][1]
			+ current_pxl_y * adfGeoTransforms[band][2]
	) + get_pixel_width_for_band(band)/2.0;
	auto pixel_center_lat = (
		adfGeoTransforms[band][3]
			+ current_pxl_x * adfGeoTransforms[band][4]
			+ current_pxl_y * adfGeoTransforms[band][5]
	) + get_pixel_height_for_band(band)/2.0;
	return std::make_pair(pixel_center_lon, pixel_center_lat);
}

}
}