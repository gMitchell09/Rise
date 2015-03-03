#pragma once
#include <vtkHyperOctree.h>

class TravelGrid
{
public:
	enum CellTypes {
		kUndefined,
		kFloor,
		kWall,
		kStairs,
	};

	struct Cell {
		CellTypes cellType;
	};

	struct CellEntry {
		Cell* cell;
		TravelGrid* xGrid;
		TravelGrid* yGrid;
	};

	TravelGrid();
	~TravelGrid();

	vtkHyperOctree *quadTree;

private:
	size_t _pageSize;
	size_t _width;
	size_t _height;

	long _posX;
	long _posY;

	CellEntry** _cells;
};