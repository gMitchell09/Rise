#include "StdAfx.h"
#include "ManagedUrgHelper.h"
#include "Urg_Helper.h"

namespace ManagedUrgHelper
{
	ManagedUrgHelper::ManagedUrgHelper()
	{
		_unUrg_Helper = new Urg_Helper();
	}

	ManagedUrgHelper::~ManagedUrgHelper()
	{
		if (_unUrg_Helper != NULL)
			delete _unUrg_Helper;
	}

	bool ManagedUrgHelper::ConnectToUrg()
	{
		return _unUrg_Helper->ConnectToUrg();
	}

	void ManagedUrgHelper::GetScanFromUrg()
	{
		_unUrg_Helper->GetScanFromUrg();
	}
	void ManagedUrgHelper::spawnIMUThread()
	{
		_unUrg_Helper->spawnIMUThread();
	}

	bool ManagedUrgHelper::StartCloudVisualization()
	{
		return _unUrg_Helper->StartCloudVisualization();
	}
}