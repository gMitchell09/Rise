#include "TravelGrid.h"

#include <boost/interprocess/mapped_region.hpp>

TravelGrid::TravelGrid()
{
	_pageSize = boost::interprocess::mapped_region::get_page_size();
	quadTree = vtkHyperOctree::New();
}


TravelGrid::~TravelGrid()
{
}
