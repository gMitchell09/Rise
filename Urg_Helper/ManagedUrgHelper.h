#pragma once

#using <System.dll>

#pragma managed(push, off)
class Urg_Helper
{
public:
	// TODO: Add your methods for this class here.
	bool ConnectToUrg();
	void GetScanFromUrg();
	void spawnIMUThread();

	bool StartCloudVisualization();
	Urg_Helper();
	~Urg_Helper();
};
#pragma managed(pop)

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