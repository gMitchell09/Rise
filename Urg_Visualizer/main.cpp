#include "Urg_Helper.h"
#include "Urg_Helper.Mock.h"
#include "urg_visualizer.h"
#include <QtWidgets/QApplication>

int main(int argc, char** argv)
{
	QApplication a(argc, argv);
	Urg_Visualizer w;
	w.show();
	return a.exec();

	return 0;
}