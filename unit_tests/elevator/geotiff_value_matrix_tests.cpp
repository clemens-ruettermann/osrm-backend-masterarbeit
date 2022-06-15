#include <boost/test/unit_test.hpp>

#include <algorithm>
#include <iterator>
#include <vector>

#include "elevation/geotiff_value_matrix.h"

BOOST_AUTO_TEST_SUITE(geotiff_value_matrix_test)

using namespace std;
using namespace osrm::elevation;


BOOST_AUTO_TEST_CASE(geotiff_value_matrix_constructor) {

	GeotiffValueMatrix<int, 3> m1{23};
	for (size_t x = 0; x < 3; x++) {
		for (size_t y = 0; y < 3; y++) {
			BOOST_CHECK_EQUAL(23, m1.vals[y][x]);
			BOOST_CHECK_EQUAL(23, m1.at(x, y));
		}
	}


	int counter1 = 0;
	auto generator1 = [&](const size_t x, const size_t y) -> int{
		return counter1++;
	};
	GeotiffValueMatrix<int, 1> m2{generator1};
	BOOST_CHECK_EQUAL(0, m2.at(0, 0));


	int counter2 = 0;
	auto generator2 = [&](const size_t x, const size_t y) -> int{
		return counter2++;
	};
	GeotiffValueMatrix<int, 3> m3{generator2};
	BOOST_CHECK_EQUAL(0, m3.at(0, 0));
	BOOST_CHECK_EQUAL(1, m3.at(1, 0));
	BOOST_CHECK_EQUAL(2, m3.at(2, 0));
	BOOST_CHECK_EQUAL(3, m3.at(0, 1));
	BOOST_CHECK_EQUAL(4, m3.at(1, 1));
	BOOST_CHECK_EQUAL(5, m3.at(2, 1));
	BOOST_CHECK_EQUAL(6, m3.at(0, 2));
	BOOST_CHECK_EQUAL(7, m3.at(1, 2));
	BOOST_CHECK_EQUAL(8, m3.at(2, 2));
}


BOOST_AUTO_TEST_CASE(geotiff_value_matrix_sum) {

	GeotiffValueMatrix<int, 3> m1{1};
	BOOST_CHECK_EQUAL(9, m1.sum());

	GeotiffValueMatrix<int, 5> m2{1};
	BOOST_CHECK_EQUAL(25, m2.sum());

	GeotiffValueMatrix<double, 5> m3{0.5};
	BOOST_CHECK_EQUAL(12.5, m3.sum());

	int counter = 0;
	auto generator = [&](const size_t x, const size_t y) -> int{
		return counter++;
	};
	GeotiffValueMatrix<int, 3> m4{generator};
	BOOST_CHECK_EQUAL(36, m4.sum());
}


BOOST_AUTO_TEST_CASE(geotiff_value_matrix_div) {

	GeotiffValueMatrix<int, 3> m1{2};
	m1 *= 3;
	BOOST_CHECK_EQUAL(6, m1.at(0, 0));
	BOOST_CHECK_EQUAL(6, m1.at(1, 0));
	BOOST_CHECK_EQUAL(6, m1.at(2, 0));
	BOOST_CHECK_EQUAL(6, m1.at(0, 1));
	BOOST_CHECK_EQUAL(6, m1.at(1, 1));
	BOOST_CHECK_EQUAL(6, m1.at(2, 1));
	BOOST_CHECK_EQUAL(6, m1.at(0, 2));
	BOOST_CHECK_EQUAL(6, m1.at(1, 2));
	BOOST_CHECK_EQUAL(6, m1.at(2, 2));

	int counter = 0;
	auto generator = [&](const size_t x, const size_t y) -> int{
		return counter++;
	};
	GeotiffValueMatrix<int, 3> m2{generator};
	m2 *= 2;
	BOOST_CHECK_EQUAL(0, m2.at(0, 0));
	BOOST_CHECK_EQUAL(2, m2.at(1, 0));
	BOOST_CHECK_EQUAL(4, m2.at(2, 0));
	BOOST_CHECK_EQUAL(6, m2.at(0, 1));
	BOOST_CHECK_EQUAL(8, m2.at(1, 1));
	BOOST_CHECK_EQUAL(10, m2.at(2, 1));
	BOOST_CHECK_EQUAL(12, m2.at(0, 2));
	BOOST_CHECK_EQUAL(14, m2.at(1, 2));
	BOOST_CHECK_EQUAL(16, m2.at(2, 2));
}





BOOST_AUTO_TEST_SUITE_END()
