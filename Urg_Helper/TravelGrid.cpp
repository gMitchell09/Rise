#include "TravelGrid.h"
#include <pcl/common/common.h>

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
	double floorHeightGuess = -3.0;
	double ceilingHeightGuess = 3.0;
	double mean = 0, median = 0, mode = 0, stddev;
	double min = std::numeric_limits<double>::max();
	double max = std::numeric_limits<double>::lowest();

	if (indices->indices.size() < TravelGrid::MinIndices) return TravelGrid::Cell::CellTypes::kUndefined;

	TravelGrid::Cell::CellTypes x = TravelGrid::Cell::CellTypes::kFloor;


	std::vector<float> pointYVals(indices->indices.size());
	//auto extractY = [](pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int ind) { return cloud->points[ind].z; };
	for (auto itr = indices->indices.begin(); itr != indices->indices.end(); ++itr)
		pointYVals.push_back(this->cloud->points[*itr].z);

	pcl::getMeanStdDev(pointYVals, mean, stddev);

	for (auto itr = indices->indices.begin(); itr != indices->indices.end(); ++itr)
	{
		pcl::PointXYZRGB point = this->cloud->points[*itr];

		if (point.z < min) min = point.z;
		if (point.z > max) max = point.z;
	}

	std::cout << std::endl;;
	std::cout << "Mean: " << mean << std::endl << "Std Dev: " << stddev << std::endl;
	std::cout << "Min: " << min << std::endl << "Max: " << max << std::endl << "Median: " << (max - min)/2.0 << std::endl;
	std::cout << std::endl;

	return x;
}
