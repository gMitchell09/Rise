#pragma once

#include "Stdafx.h"
#include "urg_helper.h"
#include "FakeUrg.h"

class Urg_Helper_Mock :
	public Urg_Helper
{
public:
	Urg_Helper_Mock();
	~Urg_Helper_Mock();
	
	MOCKABLE bool ConnectToUrg() override;
	MOCKABLE void GetScanFromUrg() override;

};
