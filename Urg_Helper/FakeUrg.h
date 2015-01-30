#pragma once
#include <string>
#include <vector>
#include "Urg_driver.h"

class FakeUrg : qrk::Urg_driver
{
public:
	void close();
	double index2rad(int scanNo);
	bool open(std::string path);
	void start_measurement();
	bool get_distance(std::vector<long, std::allocator<long>> &data, long *timestamp);

	FakeUrg();
	~FakeUrg();
};

