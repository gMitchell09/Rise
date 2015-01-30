#include "FakeUrg.h"


void FakeUrg::close() {}
double FakeUrg::index2rad(int scanNo) { return 0.0; }
bool FakeUrg::open(std::string path) { return false; }
void FakeUrg::start_measurement() {}
bool FakeUrg::get_distance(std::vector<long, std::allocator<long>> &data, long *timestamp) { return false; }

FakeUrg::FakeUrg() {}
FakeUrg::~FakeUrg() {}
