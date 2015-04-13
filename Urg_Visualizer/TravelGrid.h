#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <memory> // for std:: smart pointers
#include <thread>
#include <mutex>
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
	};

	struct CellPoint {
		int x, y;

		bool operator() (const CellPoint& lhs, const CellPoint& rhs) const
		{
			return (lhs.x*lhs.x + lhs.y*lhs.y) < (rhs.x * rhs.x + rhs.y * rhs.y);
		}
	};

	static const size_t cell_size;
	pcl::PointIndices::Ptr points_in_range(float x_start, float x_end, float y_start, float y_end);
	Cell::CellTypes classify_cell(pcl::PointIndices::Ptr indices);
	
	void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
	{
		this->cloud = cloud;
	}

	TravelGrid();
	~TravelGrid();

	Cell& cell_at(float x, float y);

private:
	std::map<CellPoint, Cell, CellPoint> grid;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

	std::thread _classificationThread;
	std::unique_ptr<std::mutex> _mapLock;

	static const unsigned int MinIndices = 1000;
	static const unsigned long RoverHeight = 1000;
};
