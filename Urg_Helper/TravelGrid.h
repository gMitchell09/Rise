#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include <memory> // for std:: smart pointers
#include <map>

class TravelGrid
{
public:
	struct Cell {
		enum CellTypes {
			kUndefined,
			kFloor,
			kWall,
			kStairs,
		};

		CellTypes cellType;

		CellTypes classify_cell(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
	};

	struct CellPoint {
		int x, y;

		bool operator() (const CellPoint& lhs, const CellPoint& rhs) const
		{
			return (lhs.x*lhs.x + lhs.y*lhs.y) < (rhs.x * rhs.x + rhs.y * rhs.y);
		}
	};

	static const size_t cell_size;

	TravelGrid();
	~TravelGrid();

	Cell& cell_at(float x, float y);

private:
	std::map<CellPoint, Cell, CellPoint> grid;
};

pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr points_in_range(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, float x_start, float x_end, float y_start, float y_end)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud2(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PassThrough<pcl::PointXYZRGB> filter;
	filter.setInputCloud(cloud);
	filter.setFilterFieldName("x");
	filter.setFilterLimits(x_start, x_end);
	filter.filter(*filteredCloud);

	filter.setInputCloud(filteredCloud);
	filter.setFilterFieldName("y");
	filter.setFilterLimits(y_start, y_end);
	filter.filter(*filteredCloud2);

	return filteredCloud2->makeShared();
}