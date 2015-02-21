#include "FakeUrg.h"
#include <limits>
#include <algorithm>
#include <iterator>

#include <iostream>
#include <cassert>

namespace
{
	template <typename T> // T models Any
	struct static_cast_func
	{
	  template <typename T1> // T1 models type statically convertible to T
	  T operator()(const T1& x) const { return static_cast<T>(x); }
	};
}

// Todo: Close urg filehandle
void FakeUrg::close() 
{
	_inFile.close();
}

// Todo: search array of angles for this index and then return corresponding angle value
double FakeUrg::index2rad(int scanNo) const
{
	if (scanNo < _angleValues.size())
	{
		return _angleValues[scanNo];
	}
	else return std::numeric_limits<double>::quiet_NaN(); 
}

// open lidar file and parse out angle array
bool FakeUrg::open(const char* device_name, 
				   long baudrate,
                   connection_type_t type) 
{
	_inFile.open(device_name, std::ios::in | std::ios::binary);
	assert (_inFile.good());

	int nAngles = 1080;
	_angleValues.resize(nAngles);

	for (int i = 0; i < nAngles; ++i)
	{
		float angle = 0.0f;
		_inFile.read(reinterpret_cast<char*>(&angle), sizeof(angle));
		assert (_inFile.good());

		_angleValues[i] = angle;
		
	}

	int32_t shouldBeZero;
	_inFile.read(reinterpret_cast<char*>(&shouldBeZero), sizeof(shouldBeZero));

	assert(shouldBeZero == 0);

	//std::transform(std::istreambuf_iterator<char> (_inFile.rdbuf()), std::istreambuf_iterator<char> (_inFile.rdbuf()) + 4320, _angleValues.begin(), static_cast_func<float>());

	return true;
}

// parse remainder of file
bool FakeUrg::start_measurement(measurement_type_t,
                               int scan_times,
                               int skip_scan)
{
	return true;
}

// pop next distance measurement from list
bool FakeUrg::get_distance(std::vector<long, std::allocator<long>> &data, long *timestamp) 
{
	if (_inFile.eof()) return false;
	long ts = -1;
	_inFile.read(reinterpret_cast<char*>(&ts), sizeof(long));
	//assert (_inFile.good());

	*timestamp = ts + _timeStamp; // maintain our offset for whatever reason
	long next;

	_inFile.read(reinterpret_cast<char*>(&next), sizeof(next));

	while (next != 0 && !_inFile.eof())
	{
		data.push_back(next);
		_inFile.read(reinterpret_cast<char*>(&next), sizeof(next));
		//assert (_inFile.good());
	}

	if (_inFile.eof()) return false;

	return true; 
}

bool FakeUrg::set_sensor_time_stamp(long time_stamp) 
{
	_timeStamp = time_stamp;
	return true; 
}

// meh
FakeUrg::FakeUrg() : _timeStamp(0) {}

// meh
FakeUrg::~FakeUrg() {}
