#pragma once
#include <string>
#include <vector>
#include <fstream>
#include "Urg_driver.h"

struct DataReading
{
	std::vector<long, std::allocator<long>> data;
	long timestamp;
};

class FakeUrg : public qrk::Urg_driver
{
public:
	MOCKABLE void close() override;
	MOCKABLE double index2rad(int scanNo) const override;
	MOCKABLE bool open(const char* device_name, 
				   long baudrate = Default_baudrate,
                   connection_type_t type = Serial) override;
	MOCKABLE bool start_measurement(measurement_type_t type = Distance,
                               int scan_times = Infinity_times,
                               int skip_scan = 0) override;
	MOCKABLE bool get_distance(std::vector<long, std::allocator<long>> &data, long *timestamp) override;

	MOCKABLE bool set_sensor_time_stamp(long time_stamp) override;

	FakeUrg();
	~FakeUrg();

private:
	std::vector<DataReading> _rangeData;
	std::vector<float> _angleValues;
	std::fstream _inFile;

	int _timeStamp;
};

