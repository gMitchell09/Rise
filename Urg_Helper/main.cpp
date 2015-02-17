#include "Urg_Helper.h"
#include "Urg_Helper.Mock.h"

int main()
{
	//Urg_Helper h;
	//h.StartPCLVisualizer();
	Urg_Helper_Mock uhm;
	uhm.ConnectToUrg();
	uhm.StartPCLVisualizer();
	//uhm.StartCloudVisualization();

	system("PAUSE");
	return 0;
}