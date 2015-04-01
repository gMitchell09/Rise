#include "Urg_Helper.h"
#include "Urg_Helper.Mock.h"
#include <cstdio>
#include <cstdlib>

int main()
{
	//Urg_Helper h;
	//h.StartPCLVisualizer();
	Urg_Helper uhm;
	uhm.ConnectToUrg();
	uhm.StartPCLVisualizer();
	//uhm.StartCloudVisualization();

	return 0;
}