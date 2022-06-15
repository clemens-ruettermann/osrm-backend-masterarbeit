//
// Created by kenspeckle on 4/9/22.
//

#ifndef OSRM_GEOTIFF_VALUE_MATRIX_H
#define OSRM_GEOTIFF_VALUE_MATRIX_H

#include <array>
#include <cstddef>
#include <iostream>
#include <cmath>
#include <functional>
#include <sstream>

namespace osrm {
namespace elevation {
template<typename T, size_t N>
struct GeotiffValueMatrix {
	static const size_t RADIUS = N / 2;
	std::array<std::array<T, N>, N> vals;

	GeotiffValueMatrix() {}

	explicit GeotiffValueMatrix(const std::function<T(size_t, size_t)>& f) {
		iterate([&](const size_t x, const size_t y) {
			vals[y][x] = f(x, y);
		});
	}

	explicit GeotiffValueMatrix(const T &init_val) {
		iterate([&](const size_t x, const size_t y) {
			vals[y][x] = init_val;
		});
	}

	inline T &at(size_t x, size_t y) {
		return vals[y][x];
	}

	std::array<T *, N> row(size_t y) {
		std::array<T *, N> ret;
		for (size_t x = 0; x < N; x++) {
			ret[x] = &vals[y][x];
		}
		return ret;
	}

	std::array<T *, N> col(size_t x) {
		std::array<T *, N> ret;
		for (size_t y = 0; y < N; y++) {
			ret[y] = &vals[y][x];
		}
		return ret;
	}

	T sum() const {
		T ret{};
		auto sum_func = [&](size_t x, size_t y) {
			ret += vals[y][x];
		};
		iterate(sum_func);
		return ret;
	}

	T as_vector_dot(const GeotiffValueMatrix &m) {
		T ret{};
		auto dot_func = [&](const size_t x, const size_t y) {
			ret += m.vals[y][x];
		};
		iterate(dot_func);
		return ret;
	}


	GeotiffValueMatrix &operator*=(const T &val) {
		auto mult_func = [&](const size_t x, const size_t y) {
			vals[x][y] *= val;
		};
		iterate(mult_func);
		return *this;
	}

	GeotiffValueMatrix & cellwise_multiply(const GeotiffValueMatrix &m) {
		auto mult_func = [&](const size_t x, const size_t y) {
			vals[x][y] *= m.vals[x][y];
		};
		iterate(mult_func);
		return *this;
	}

	GeotiffValueMatrix &operator/=(const T &val) {
		auto div_func = [&](const size_t x, const size_t y) {
			vals[x][y] /= val;
		};
		iterate(div_func);
		return *this;
	}

	[[nodiscard]] std::string to_string() const {
		std::stringstream ss;
		for (size_t y = 0; y < N; y++) {
			ss << "[";
			for (size_t x = 0; x < N; x++) {
				if (x == 0) {
					ss << std::to_string(vals[y][x]);
				} else {
					ss << ", " << std::to_string(vals[y][x]);
				}
			}
			ss << "]" << std::endl;
		}
		return ss.str();
	}

	friend std::ostream &operator<<(std::ostream &os, const GeotiffValueMatrix &matrix) {
		os << matrix.to_string();
		return os;
	}

	static GeotiffValueMatrix<double, N> get_mean_kernel() {
		return GeotiffValueMatrix < double, N > {(N * N) / 1.0};
	}


	static GeotiffValueMatrix<double, N> get_gauss_kernel(const double &sigma_x, const double &sigma_y) {
		GeotiffValueMatrix<double, N> ret{0};
		for (size_t y = 0; y < N; y++) {
			for (size_t x = 0; x < N; x++) {
				double i_exp_val = (double) x - RADIUS;
				double j_exp_val = (double) y - RADIUS;
				ret.at(x, y) = exp(-(pow(i_exp_val, 2) + pow(j_exp_val, 2)) / (pow(sigma_x, 2) + pow(sigma_y, 2)));
			}
		}
		ret /= ret.sum();
		return ret;
	}


private:

	void iterate(const std::function<void(size_t x, size_t y)>& f) const {
		for (size_t y = 0; y < N; y++) {
			for (size_t x = 0; x < N; x++) {
				f(x, y);
			}
		}
	}

};


}
}

#endif //OSRM_GEOTIFF_VALUE_MATRIX_H
