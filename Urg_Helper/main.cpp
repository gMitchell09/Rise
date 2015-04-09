#include "Urg_Helper.h"
#include "Urg_Helper.Mock.h"
#include <cstdio>
#include <cstdlib>

#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkChartXY.h>
#include <vtkSmartPointer.h>
#include <vtkAxis.h>
#include <vtkContextView.h>
#include <vtkContextScene.h>
#include <vtkButtonWidget.h>

#include <memory>

int main()
{
	//Urg_Helper h;
	//h.StartPCLVisualizer();
	
	vtkSmartPointer<vtkContextView> view = 
		vtkSmartPointer<vtkContextView>::New();

	view->GetRenderer()->SetBackground(1.0, 1.0, 1.0);

	vtkSmartPointer<vtkChartXY> chart = vtkSmartPointer<vtkChartXY>::New();
	chart->GetAxis(0)->SetGridVisible(true);
	chart->GetAxis(1)->SetGridVisible(true);
	vtkSmartPointer<vtkButtonWidget> button = 
		vtkSmartPointer<vtkButtonWidget>::New();

	view->GetScene()->AddItem(chart);
	//view->GetScene()->AddItem(button);

	view->GetInteractor()->Initialize();
	view->GetInteractor()->Start();
	
	Urg_Helper_Mock uhm;
	uhm.ConnectToUrg();
	uhm.StartPCLVisualizer();
	//uhm.StartCloudVisualization();
	return 0;
}