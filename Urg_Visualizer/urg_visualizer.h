#ifndef URG_VISUALIZER_H
#define URG_VISUALIZER_H

#include <QtWidgets/QMainWindow>
#include "ui_urg_visualizer.h"

class Urg_Visualizer : public QMainWindow
{
	Q_OBJECT

public:
	Urg_Visualizer(QWidget *parent = 0);
	~Urg_Visualizer();

private:
	Ui::Urg_VisualizerClass ui;
};

#endif // URG_VISUALIZER_H
