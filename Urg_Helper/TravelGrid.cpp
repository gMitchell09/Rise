#include "TravelGrid.h"
#include <algorithm>

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

pcl::PointIndices::Ptr TravelGrid::points_in_range(float x_start, float x_end, float y_start, float y_end)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud2 (new pcl::PointCloud<pcl::PointXYZRGB>());

	pcl::PassThrough<pcl::PointXYZRGB> filter(true);
	filter.setInputCloud(this->cloud);
	filter.setFilterFieldName("x");
	filter.setFilterLimits(x_start, x_end);
	filter.setNegative(true);
	filter.filter(*filteredCloud);

	pcl::PointIndices::Ptr pi (new pcl::PointIndices());
	filter.getRemovedIndices(*pi);

	pcl::IndicesPtr ind, ind2;
	
	ind = filter.getIndices();

	filter.setInputCloud(cloud);
	filter.setFilterFieldName("y");
	filter.setFilterLimits(y_start, y_end);
	filter.setNegative(true);
	filter.filter(*filteredCloud2);

	pcl::PointIndices::Ptr pi2 (new pcl::PointIndices());
	filter.getRemovedIndices(*pi2);
	ind2 = filter.getIndices();

	std::vector<int> inter_verts(pi->indices.size() + pi2->indices.size()); // meh, memory is cheap.
	
	std::set_intersection(pi->indices.begin(), pi->indices.end(), pi2->indices.begin(), pi2->indices.end(), inter_verts.begin());

	pi->indices = inter_verts;
	
	return pi;
}

TravelGrid::~TravelGrid()
{
}

TravelGrid::Cell::CellTypes TravelGrid::classify_cell(pcl::PointIndices::Ptr indices)
{
	float floorHeightGuess = -3.0;
	float ceilingHeightGuess = 3.0;
	float mean = 0, median = 0, mode = 0;
	float min = std::numeric_limits<float>::max();
	float max = std::numeric_limits<float>::min();
	TravelGrid::Cell::CellTypes x = TravelGrid::Cell::CellTypes::kFloor;
	/*for (auto itr = indices->indices.begin(); itr != indices->indices.end(); ++itr)
	{
		pcl::PointXYZRGB point = this->cloud->points[*itr];
		point.x;
		point.y;
		point.z;

		x = (TravelGrid::Cell::CellTypes)((int)(point.x * 1000) % 3);
		break;
	}*/

	return x;
}
