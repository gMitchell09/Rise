#include "TravelGrid.h"

const size_t TravelGrid::cell_size = 200; // 200mm

TravelGrid::TravelGrid()
{

}

TravelGrid::Cell& TravelGrid::cell_at(float x, float y)
{
	CellPoint p;
	p.x = (int)(x / 200);
	p.y = (int)(y / 200);

	auto search = grid.find(p);
	bool exists = (search != grid.end());

	Cell& cell = grid[p];
	if (!exists) cell.cellType = Cell::kUndefined;

	return cell;
}

TravelGrid::~TravelGrid()
{
}

TravelGrid::Cell::CellTypes TravelGrid::Cell::classify_cell(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
	float floorHeightGuess = -3.0;
	float ceilingHeightGuess = 3.0;
	float mean = 0, median = 0, mode = 0;
	float min = std::numeric_limits<float>::max();
	float max = std::numeric_limits<float>::min();

	for (auto itr = cloud->begin(); itr != cloud->end(); ++itr)
	{
		(*itr).x;
		(*itr).y;
		(*itr).z;
	}

	return Cell::kUndefined;
}
