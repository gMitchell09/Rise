#pragma once
#include <string>
#include <vector>
#include "Urg_driver.h"

class FakeUrg : public qrk::Urg_driver
{
public:
	MOCKABLE void close();
	MOCKABLE double index2rad(int scanNo);
	MOCKABLE bool open(const char* device_name, 
				   long baudrate = Default_baudrate,
                   connection_type_t type = Serial);
	MOCKABLE void start_measurement();
	MOCKABLE bool get_distance(std::vector<long, std::allocator<long>> &data, long *timestamp);

	MOCKABLE bool set_sensor_time_stamp(long time_stamp);

	FakeUrg();
	~FakeUrg();
};

