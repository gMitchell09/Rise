#pragma once

#include <memory>

#include "TravelGrid.h"

class MovementHandler
{
public:
	MovementHandler(void);
	~MovementHandler(void);

	enum Directions {
		RotateLeft90, RotateRight90, MoveForward, Stop
	};

	Directions WhereToGo();

private:
	std::shared_ptr<TravelGrid> _travelGrid;
};

