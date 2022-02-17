//
// Created by kenspeckle on 4/5/22.
//

#ifndef OSRM_MATH_UTILS_H
#define OSRM_MATH_UTILS_H

#include <cmath>
#include <functional>

namespace osrm {
namespace utils {
namespace math {

static std::function<double(double, double)> gauss(const double sigma_x, const double sigma_y, const double x_0, const double y_0) {
	double a = 1.0;
	return [&](const double x, const double y) {
		return a * exp(-((pow(x-x_0, 2)/(2*pow(sigma_x, 2))) + (pow(y-y_0, 2)/(2*pow(sigma_y, 2)))));
	};
}

static double gauss(const double sigma_x, const double sigma_y, const double x_0, const double y_0, const double x, const double y) {
	auto a = 1.0;
	auto part_1 = (pow(x-x_0, 2)/(2*pow(sigma_x, 2)));
	auto part_2 = (pow(y-y_0, 2)/(2*pow(sigma_y, 2)));
	auto combined = a * exp(-(part_1 + part_2));
	return combined;
}
}
}
}

#endif //OSRM_MATH_UTILS_H
