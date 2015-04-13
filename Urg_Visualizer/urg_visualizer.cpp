#include "urg_visualizer.h"
#include "Urg_Helper.h"
#include "Urg_Helper.Mock.h"

Urg_Visualizer::Urg_Visualizer(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	//Urg_Helper h;
	//h.StartPCLVisualizer();
	
	Urg_Helper_Mock uhm;
	uhm.ConnectToUrg();
	uhm.StartPCLVisualizer();
	uhm.StartCloudVisualization();
}

Urg_Visualizer::~Urg_Visualizer()
{

}
