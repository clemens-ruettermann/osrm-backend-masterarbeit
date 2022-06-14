//
// Created by kenspeckle on 17.02.22.
//

#include <string>
#include <fstream>
#include <iostream>
#include <chrono>
#include <sstream>
#include <tuple>
#include <limits>
#include "consumption_utils.hpp"

using namespace enav;
using namespace std;

int main() {
	car id4{"ID4", 18.3, 2124, 77};

	vector<tuple<double, double, double, double>> measurements;
	measurements.reserve(19000);
	string filename = "/tmp/test/segments.csv";
	string line;
	unsigned int number_lines = 0;
	ifstream filer(filename);
	if (filer.is_open()) {
		getline(filer,line);// Skip the first line
		string str_avg_speed, str_distance, str_slope, str_duration;
		double avg_speed, distance, slope, duration;
		while (getline(filer,line)) {
			number_lines++;
			stringstream ss(line);

			getline(ss, str_distance, ',');
			getline(ss, str_avg_speed, ',');
			getline(ss, str_slope, ',');
			getline(ss, str_duration, ',');

			distance = stod(str_distance);
			avg_speed = stod(str_avg_speed);
			slope = stod(str_slope);
			duration = stod(str_duration) * 3.6;

			measurements.emplace_back(make_tuple(distance,duration, avg_speed, slope));
		}
		filer.close();
	} else {
		cout << "could not read file" << endl;
	}

	const unsigned int repetitions = 5000;

	cout << "v3:" << endl;
	auto start = chrono::high_resolution_clock::now();
	long double sum = 0;
	//for (unsigned int i = 0; i < repetitions; i++) {
	//	for (auto & j : measurements) {
	//		double measurement = calculate_kwh_change_v3(&id4, get<0>(j), get<1>(j), get<2>(j), get<3>(j));
	//		sum += measurement;
	//	}
	//}
	auto end = chrono::high_resolution_clock::now();
	auto diff_ns = chrono::duration_cast<chrono::nanoseconds>(end - start).count();
	auto diff_my = chrono::duration_cast<chrono::microseconds>(end - start).count();
	auto diff_ms = chrono::duration_cast<chrono::milliseconds>(end - start).count();
	cout << "This took " << to_string(diff_ms) << "ms / " << to_string(diff_my) << "my (" << to_string(diff_ns / (repetitions * number_lines)) << "ns per call)" << endl;
	cout << "======================================================" << endl;




	cout << "v1:" << endl;
	start = chrono::high_resolution_clock::now();
	sum = 0;
	//for (unsigned int i = 0; i < repetitions; i++) {
	//	for (auto & j : measurements) {
	//		double measurement = calculate_kwh_change_v1(&id4, get<0>(j), get<1>(j), get<2>(j), get<3>(j));
	//		sum += measurement;
	//	}
	//}
	end = chrono::high_resolution_clock::now();
	diff_ns = chrono::duration_cast<chrono::nanoseconds>(end - start).count();
	diff_my = chrono::duration_cast<chrono::microseconds>(end - start).count();
	diff_ms = chrono::duration_cast<chrono::milliseconds>(end - start).count();
	cout << "This took " << to_string(diff_ms) << "ms / " << to_string(diff_my) << "my (" << to_string(diff_ns / (repetitions * number_lines)) << "ns per call)" << endl;
	cout << "======================================================" << endl;




	cout << "v2:" << endl;
	start = chrono::high_resolution_clock::now();
	sum = 0;
	//for (unsigned int i = 0; i < repetitions; i++) {
	//	for (auto & j : measurements) {
	//		double measurement = calculate_kwh_change_v2(&id4, get<0>(j), get<1>(j), get<2>(j), get<3>(j));
	//		sum += measurement;
	//	}
	//}
	end = chrono::high_resolution_clock::now();
	diff_ns = chrono::duration_cast<chrono::nanoseconds>(end - start).count();
	diff_my = chrono::duration_cast<chrono::microseconds>(end - start).count();
	diff_ms = chrono::duration_cast<chrono::milliseconds>(end - start).count();
	cout << "This took " << to_string(diff_ms) << "ms / " << to_string(diff_my) << "my (" << to_string(diff_ns / (repetitions * number_lines)) << "ns per call)" << endl;
	cout << "======================================================" << endl;




	typedef std::numeric_limits< long double > dbl;
	cout.precision(dbl::max_digits10 + 2);
	cout << sum << " (this is to force the compile to execute the method)" << endl;
}
