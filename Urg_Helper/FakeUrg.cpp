#include "FakeUrg.h"


void FakeUrg::close() {}
double FakeUrg::index2rad(int scanNo) { return 0.0; }
bool FakeUrg::open(const char* device_name, 
				   long baudrate,
                   connection_type_t type) { return false; }
void FakeUrg::start_measurement() {}
bool FakeUrg::get_distance(std::vector<long, std::allocator<long>> &data, long *timestamp) { return false; }
bool FakeUrg::set_sensor_time_stamp(long time_stamp) { return false; }

FakeUrg::FakeUrg() {}
FakeUrg::~FakeUrg() {}
