#pragma once

#include <QtWidgets/QMainWindow>
#include <QtWidgets>
#include "ui_ImageViewer.h"
#include "ViewerWidget.h"
#include "lighting.h"
#include "representation.h"

class ImageViewer : public QMainWindow
{
	Q_OBJECT

public:
	ImageViewer(QWidget* parent = Q_NULLPTR);

private:
	Ui::ImageViewerClass* ui;
	ViewerWidget* vW;

	QColor globalColor; QColor color1; QColor color2, color3D;
	QSettings settings;
	QMessageBox msgBox;

	bool polygonDone = false;
	bool lineDone = false;
	bool curveDone = false;

	bool objectLoaded = false;
	enum projTypes { parallel, perspective };
	double phi = 0.0, theta = 0.0;

	//Event filters
	bool eventFilter(QObject* obj, QEvent* event);

	//ViewerWidget Events
	bool ViewerWidgetEventFilter(QObject* obj, QEvent* event);
	void ViewerWidgetMouseButtonPress(ViewerWidget* w, QEvent* event);
	void ViewerWidgetMouseButtonRelease(ViewerWidget* w, QEvent* event);
	void ViewerWidgetMouseMove(ViewerWidget* w, QEvent* event);
	void ViewerWidgetLeave(ViewerWidget* w, QEvent* event);
	void ViewerWidgetEnter(ViewerWidget* w, QEvent* event);
	void ViewerWidgetWheel(ViewerWidget* w, QEvent* event);

	//ImageViewer Events
	void closeEvent(QCloseEvent* event);

	//Image functions
	bool openImage(QString filename);
	bool saveImage(QString filename);

	void linkSlidersWithSpinBoxes();
	void renderObject3D();
	Lighting setUpPrime();

private slots:
	void on_actionSave_as_triggered();
	void on_actionClear_triggered();
	void on_actionExit_triggered();

//-----------------------------------------
//		*** Tools 2D slots ***
//-----------------------------------------
	void on_pushButtonSetColor_clicked();
	void on_pushButtonColor1_clicked();
	void on_pushButtonColor2_clicked();
	void on_pushButtonTurn_clicked();
	void on_pushButtonScale_clicked();
	void on_checkBoxAxialSymmetry_toggled(bool checked);
	void on_pushButtonShear_clicked();

//-----------------------------------------
//		*** Tools 3D slots ***
//-----------------------------------------
	void on_pushButton3DTools_clicked();
	void on_pushButton2DTools_clicked();
	void on_pushButtonSaveVTK3D_clicked();
	void on_pushButton3DColor_clicked();
	void on_pushButtonOpenVTK3D_clicked();
	void on_horizontalSliderAzimuth_valueChanged(int value);
	void on_horizontalSliderZenith_valueChanged(int value);
	void on_pushButtonLighting_clicked();
	void on_pushButtonBack_clicked();
	void on_pushButtonApplyChanges_clicked();
};
