/********************************************************************************
** Form generated from reading UI file 'urg_visualizer.ui'
**
** Created by: Qt User Interface Compiler version 5.4.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_URG_VISUALIZER_H
#define UI_URG_VISUALIZER_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Urg_VisualizerClass
{
public:
    QAction *actionAbout;
    QAction *actionSave_Cloud;
    QAction *actionImport_Cloud;
    QAction *actionExit;
    QAction *actionXY_Plane;
    QAction *actionXZ_Plane;
    QAction *actionYZ_Plane;
    QAction *actionVoxel_Grid;
    QAction *actionSmoothing;
    QAction *actionOutlier;
    QWidget *centralWidget;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QGraphicsView *graphicsView;
    QPushButton *pushButton;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuFilters;
    QMenu *menuPlane_Removal;
    QMenu *menuAbut;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *Urg_VisualizerClass)
    {
        if (Urg_VisualizerClass->objectName().isEmpty())
            Urg_VisualizerClass->setObjectName(QStringLiteral("Urg_VisualizerClass"));
        Urg_VisualizerClass->resize(864, 574);
        actionAbout = new QAction(Urg_VisualizerClass);
        actionAbout->setObjectName(QStringLiteral("actionAbout"));
        actionSave_Cloud = new QAction(Urg_VisualizerClass);
        actionSave_Cloud->setObjectName(QStringLiteral("actionSave_Cloud"));
        actionImport_Cloud = new QAction(Urg_VisualizerClass);
        actionImport_Cloud->setObjectName(QStringLiteral("actionImport_Cloud"));
        actionExit = new QAction(Urg_VisualizerClass);
        actionExit->setObjectName(QStringLiteral("actionExit"));
        actionXY_Plane = new QAction(Urg_VisualizerClass);
        actionXY_Plane->setObjectName(QStringLiteral("actionXY_Plane"));
        actionXZ_Plane = new QAction(Urg_VisualizerClass);
        actionXZ_Plane->setObjectName(QStringLiteral("actionXZ_Plane"));
        actionYZ_Plane = new QAction(Urg_VisualizerClass);
        actionYZ_Plane->setObjectName(QStringLiteral("actionYZ_Plane"));
        actionVoxel_Grid = new QAction(Urg_VisualizerClass);
        actionVoxel_Grid->setObjectName(QStringLiteral("actionVoxel_Grid"));
        actionSmoothing = new QAction(Urg_VisualizerClass);
        actionSmoothing->setObjectName(QStringLiteral("actionSmoothing"));
        actionOutlier = new QAction(Urg_VisualizerClass);
        actionOutlier->setObjectName(QStringLiteral("actionOutlier"));
        centralWidget = new QWidget(Urg_VisualizerClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        scrollArea = new QScrollArea(centralWidget);
        scrollArea->setObjectName(QStringLiteral("scrollArea"));
        scrollArea->setGeometry(QRect(-1, 69, 871, 471));
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QStringLiteral("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 869, 469));
        graphicsView = new QGraphicsView(scrollAreaWidgetContents);
        graphicsView->setObjectName(QStringLiteral("graphicsView"));
        graphicsView->setGeometry(QRect(-85, -59, 1101, 641));
        pushButton = new QPushButton(scrollAreaWidgetContents);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setGeometry(QRect(140, 120, 75, 23));
        scrollArea->setWidget(scrollAreaWidgetContents);
        Urg_VisualizerClass->setCentralWidget(centralWidget);
        scrollArea->raise();
        graphicsView->raise();
        menuBar = new QMenuBar(Urg_VisualizerClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 864, 21));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QStringLiteral("menuFile"));
        menuFilters = new QMenu(menuBar);
        menuFilters->setObjectName(QStringLiteral("menuFilters"));
        menuPlane_Removal = new QMenu(menuBar);
        menuPlane_Removal->setObjectName(QStringLiteral("menuPlane_Removal"));
        menuAbut = new QMenu(menuBar);
        menuAbut->setObjectName(QStringLiteral("menuAbut"));
        Urg_VisualizerClass->setMenuBar(menuBar);
        statusBar = new QStatusBar(Urg_VisualizerClass);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        Urg_VisualizerClass->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuFilters->menuAction());
        menuBar->addAction(menuPlane_Removal->menuAction());
        menuBar->addAction(menuAbut->menuAction());
        menuFile->addAction(actionSave_Cloud);
        menuFile->addAction(actionImport_Cloud);
        menuFile->addSeparator();
        menuFile->addAction(actionExit);
        menuFilters->addAction(actionVoxel_Grid);
        menuFilters->addAction(actionSmoothing);
        menuFilters->addAction(actionOutlier);
        menuPlane_Removal->addAction(actionXY_Plane);
        menuPlane_Removal->addAction(actionXZ_Plane);
        menuPlane_Removal->addAction(actionYZ_Plane);
        menuAbut->addAction(actionAbout);

        retranslateUi(Urg_VisualizerClass);

        QMetaObject::connectSlotsByName(Urg_VisualizerClass);
    } // setupUi

    void retranslateUi(QMainWindow *Urg_VisualizerClass)
    {
        Urg_VisualizerClass->setWindowTitle(QApplication::translate("Urg_VisualizerClass", "Urg_Visualizer", 0));
        actionAbout->setText(QApplication::translate("Urg_VisualizerClass", "About", 0));
        actionSave_Cloud->setText(QApplication::translate("Urg_VisualizerClass", "Save Cloud", 0));
        actionImport_Cloud->setText(QApplication::translate("Urg_VisualizerClass", "Import Cloud", 0));
        actionExit->setText(QApplication::translate("Urg_VisualizerClass", "Exit", 0));
        actionXY_Plane->setText(QApplication::translate("Urg_VisualizerClass", "XY Plane", 0));
        actionXZ_Plane->setText(QApplication::translate("Urg_VisualizerClass", "XZ Plane", 0));
        actionYZ_Plane->setText(QApplication::translate("Urg_VisualizerClass", "YZ Plane", 0));
        actionVoxel_Grid->setText(QApplication::translate("Urg_VisualizerClass", "Voxel Grid", 0));
        actionSmoothing->setText(QApplication::translate("Urg_VisualizerClass", "Smoothing", 0));
        actionOutlier->setText(QApplication::translate("Urg_VisualizerClass", "Outlier", 0));
        pushButton->setText(QApplication::translate("Urg_VisualizerClass", "PushButton", 0));
        menuFile->setTitle(QApplication::translate("Urg_VisualizerClass", "File", 0));
        menuFilters->setTitle(QApplication::translate("Urg_VisualizerClass", "Filters", 0));
        menuPlane_Removal->setTitle(QApplication::translate("Urg_VisualizerClass", "Plane Removal", 0));
        menuAbut->setTitle(QApplication::translate("Urg_VisualizerClass", "Help", 0));
    } // retranslateUi

};

namespace Ui {
    class Urg_VisualizerClass: public Ui_Urg_VisualizerClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_URG_VISUALIZER_H
