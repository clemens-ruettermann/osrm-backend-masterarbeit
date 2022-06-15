//
// Created by kenspeckle on 4/5/22.
//

#ifndef OSRM_TO_STRING_UTILS_HPP
#define OSRM_TO_STRING_UTILS_HPP

#include <iostream>
#include <array>

namespace std {
//static bool is_same(std::array<std::array<double, 3>, 3> left, std::array<std::array<double, 3>, 3> right) {
//	return left[0][0] == right[0][0] && left[1][0] == right[1][0] && left[2][0] == right[2][0]
//	       && left[0][1] == right[0][1] && left[1][1] == right[1][1] && left[2][1] == right[2][1]
//	       && left[0][2] == right[0][2] && left[1][2] == right[1][2] && left[2][2] == right[2][2];
//}

template<typename T>
inline std::ostream &operator<<(std::ostream &out, const std::array<T, 3> m) {
	out << "[";
	for (size_t j = 0; j < 3; j++) {
		if (j == 0) {
			out << std::to_string(m[j]);
		} else {
			out << "," << std::to_string(m[j]);
		}
	}
	out << "]";
	out << std::endl;
	return out;
}

template<typename T>
inline std::ostream &operator<<(std::ostream &out, const std::array<std::array<T, 3>, 3> &m) {
	out << "[" << std::endl;
	for (size_t i = 0; i < 3; i++) {
		out << m[i];
		if (i != 2) {
			out << ",";
		}
		out << std::endl;
	}
	out << "]";
	return out;
}
}



#endif //OSRM_TO_STRING_UTILS_HPP
