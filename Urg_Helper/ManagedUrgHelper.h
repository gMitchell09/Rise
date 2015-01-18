#pragma once

#using <System.dll>
class Urg_Helper;

namespace ManagedUrgHelper {
	public ref class ManagedUrgHelper
	{
	public:
		ManagedUrgHelper();
		~ManagedUrgHelper();
	
		bool ConnectToUrg();
		void GetScanFromUrg();
		void spawnIMUThread();

		bool StartCloudVisualization();

	private:
		Urg_Helper *_unUrg_Helper;
	};
}