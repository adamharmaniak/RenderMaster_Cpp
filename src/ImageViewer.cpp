#include "ImageViewer.h"

ImageViewer::ImageViewer(QWidget* parent)
	: QMainWindow(parent), ui(new Ui::ImageViewerClass)
{
	ui->setupUi(this);
	vW = new ViewerWidget(QSize(500, 500));
	ui->scrollArea->setWidget(vW);

	ui->scrollArea->setBackgroundRole(QPalette::Dark);
	ui->scrollArea->setWidgetResizable(true);
	ui->scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
	ui->scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

	vW->setObjectName("ViewerWidget");
	vW->installEventFilter(this);

	globalColor = Qt::blue;
	color1 = Qt::green;
	color2 = Qt::red;
	color3D = Qt::darkCyan;
	QString style_sheet = QString("background-color: #%1;").arg(globalColor.rgba(), 0, 16);
	ui->pushButtonSetColor->setStyleSheet(style_sheet);
	ui->pushButtonColor1->setStyleSheet("background-color: green;");
	ui->pushButtonColor2->setStyleSheet("background-color: red;");
	ui->pushButton3DColor->setStyleSheet("background-color: cyan;");

	linkSlidersWithSpinBoxes();
}

// Event filters
bool ImageViewer::eventFilter(QObject* obj, QEvent* event)
{
	if (obj->objectName() == "ViewerWidget") {
		return ViewerWidgetEventFilter(obj, event);
	}
	return false;
}

//ViewerWidget Events
bool ImageViewer::ViewerWidgetEventFilter(QObject* obj, QEvent* event)
{
	ViewerWidget* w = static_cast<ViewerWidget*>(obj);

	if (!w) {
		return false;
	}

	if (event->type() == QEvent::MouseButtonPress) {
		ViewerWidgetMouseButtonPress(w, event);
	}
	else if (event->type() == QEvent::MouseButtonRelease) {
		ViewerWidgetMouseButtonRelease(w, event);
	}
	else if (event->type() == QEvent::MouseMove) {
		ViewerWidgetMouseMove(w, event);
	}
	else if (event->type() == QEvent::Leave) {
		ViewerWidgetLeave(w, event);
	}
	else if (event->type() == QEvent::Enter) {
		ViewerWidgetEnter(w, event);
	}
	else if (event->type() == QEvent::Wheel) {
		ViewerWidgetWheel(w, event);
	}

	return QObject::eventFilter(obj, event);
}
void ImageViewer::ViewerWidgetMouseButtonPress(ViewerWidget* w, QEvent* event)
{
	QMouseEvent* e = static_cast<QMouseEvent*>(event);
	if (e->button() == Qt::LeftButton && ui->toolButtonDrawLine->isChecked() && lineDone == false)
	{
		ui->toolButtonDrawCircle->setChecked(false);
		if (w->getDrawLineActivated()) {
			w->drawLine(w->getDrawLineBegin(), e->pos(), globalColor, ui->comboBoxLineAlg->currentIndex());
			w->setDrawLineActivated(false);
			lineDone = true;
		}
		else {
			w->setDrawLineBegin(e->pos());
			w->setDrawLineActivated(true);
			w->setPixel(e->pos().x(), e->pos().y(), globalColor);
			w->update();
		}
	}
	if (e->button() == Qt::LeftButton && ui->toolButtonDrawCircle->isChecked())
	{
		ui->toolButtonDrawLine->setChecked(false);
		ui->comboBoxLineAlg->setCurrentIndex(1);
		if (w->getDrawCircleActivated()) {
			w->drawCircle(w->getDrawCircleCenter(), e->pos(), globalColor);
			w->setDrawCircleActivated(false);
		}
		else {
			w->setDrawCircleCenter(e->pos());
			w->setDrawCircleActivated(true);
			w->setPixel(e->pos().x(),e->pos().y(), globalColor);
			w->update();
		}
	}
	if (e->button() == Qt::LeftButton && ui->toolButtonDrawPolygon->isChecked() && polygonDone == false) {
		w->setPixel(e->pos().x(), e->pos().y(), globalColor);
		w->addToVectorOfPoints(e->pos());
		w->update();
	}
	if (e->button() == Qt::RightButton && ui->toolButtonDrawPolygon->isChecked()) {
		w->drawPolygon(globalColor, color1, color2, ui->comboBoxLineAlg->currentIndex(), ui->comboBoxInterpolation->currentIndex());
		ui->toolButtonDrawPolygon->setDisabled(true);
		polygonDone = true;
	}
	if (e->button() == Qt::LeftButton && ui->toolButtonDrawCurve->isChecked() && curveDone == false) {
		w->setPixel(e->pos().x(), e->pos().y(), color2, ui->toolButtonDrawCurve->isChecked());
		w->addToVectorOfCurvePoints(e->pos());
		w->update();
	}
	if (e->button() == Qt::RightButton && ui->toolButtonDrawCurve->isChecked()) {
		w->calculateTangents();
		w->drawCurve(globalColor, color2, ui->comboBoxCurveType->currentIndex(), ui->comboBoxLineAlg->currentIndex());
		ui->toolButtonDrawCurve->setDisabled(true);
		curveDone = true;
	}

}
void ImageViewer::ViewerWidgetMouseButtonRelease(ViewerWidget* w, QEvent* event)
{
	QMouseEvent* e = static_cast<QMouseEvent*>(event);
}
void ImageViewer::ViewerWidgetMouseMove(ViewerWidget* w, QEvent* event)
{
	QMouseEvent* e = static_cast<QMouseEvent*>(event);

	if (ui->toolButtonDrawPolygon->isChecked()) {
		if (e->buttons() & Qt::LeftButton && ui->pushButtonMove->isChecked()) {
			QPoint offset = e->pos() -  w->getMoveStart();
			if (!w->getMoveStart().isNull()) {
				w->movePolygon(globalColor, color1, color2, offset, ui->comboBoxLineAlg->currentIndex(), ui->comboBoxInterpolation->currentIndex());
			}
			w->setMoveStart(e->pos());
		}
		else if (ui->pushButtonMove->isChecked()) {
			w->setMoveStart(QPoint());
		}
	}
	if (ui->toolButtonDrawLine->isChecked()) {
		if (e->buttons() & Qt::LeftButton && ui->pushButtonMove->isChecked()) {
			QPoint offset = e->pos() - w->getMoveStart();
			if (!w->getMoveStart().isNull()) {
				w->moveLine(globalColor, offset, ui->comboBoxLineAlg->currentIndex());
			}
			w->setMoveStart(e->pos());
		}
		else if (ui->pushButtonMove->isChecked()) {
			w->setMoveStart(QPoint());
		}
	}
	if (ui->toolButtonDrawCurve->isChecked()) {
		if (e->buttons() & Qt::LeftButton && ui->pushButtonMove->isChecked()) {
			QPoint offset = e->pos() - w->getMoveStart();
			if (!w->getMoveStart().isNull()) {
				w->moveCurve(globalColor, color2, offset, ui->comboBoxLineAlg->currentIndex(), ui->comboBoxCurveType->currentIndex(), ui->toolButtonDrawCurve->isChecked());
			}
			w->setMoveStart(e->pos());
		}
		else if (ui->pushButtonMove->isChecked()) {
			w->setMoveStart(QPoint());
		}
		
		if (e->buttons() & Qt::LeftButton && ui->checkBoxMoveVectors->isChecked()) {
			ui->pushButtonMove->setChecked(false);
			if (!w->getMoveStart().isNull()) {
				QPoint offset = e->pos() - w->getMoveStart();
				w->moveTheNearestPoint(globalColor, color2, offset, ui->comboBoxCurveType->currentIndex(), ui->comboBoxLineAlg->currentIndex());
			}
			w->setMoveStart(w->calculateTheNearestPoint(e->pos()));
		}
		else if (ui->checkBoxMoveVectors->isChecked()) {
			w->setMoveStart(QPoint());
		}
	}
}
void ImageViewer::ViewerWidgetLeave(ViewerWidget* w, QEvent* event)
{
}
void ImageViewer::ViewerWidgetEnter(ViewerWidget* w, QEvent* event)
{
}
void ImageViewer::ViewerWidgetWheel(ViewerWidget* w, QEvent* event)
{
	QWheelEvent* wheelEvent = static_cast<QWheelEvent*>(event);

	if (ui->checkBoxScale->isChecked()) {
		int deltaY = wheelEvent->angleDelta().y();
		double scale = 1.0;
		if (deltaY < 0) {
			scale = 0.75;
		}
		else if (deltaY > 0) {
			scale = 1.25;
		}
		if (ui->toolButtonDrawPolygon->isChecked()) {
			w->scalePolygon(globalColor, color1, color2, ui->comboBoxLineAlg->currentIndex(), scale, scale, ui->comboBoxInterpolation->currentIndex());
		}
		if (ui->toolButtonDrawLine->isChecked()) {
			w->scaleLine(globalColor, ui->comboBoxLineAlg->currentIndex(), scale, scale);
		}
		if (ui->toolButtonDrawCurve->isChecked()) {
			w->scaleCurve(globalColor, color2, ui->comboBoxLineAlg->currentIndex(), scale, scale, ui->comboBoxCurveType->currentIndex(), ui->toolButtonDrawCurve->isChecked());
		}
	}
}

//ImageViewer Events
void ImageViewer::closeEvent(QCloseEvent* event)
{
	if (QMessageBox::Yes == QMessageBox::question(this, "Close Confirmation", "Are you sure you want to exit?", QMessageBox::Yes | QMessageBox::No))
	{
		event->accept();
	}
	else {
		event->ignore();
	}
}

//Image functions
bool ImageViewer::openImage(QString filename)
{
	QImage loadedImg(filename);
	if (!loadedImg.isNull()) {
		return vW->setImage(loadedImg);
	}
	return false;
}
bool ImageViewer::saveImage(QString filename)
{
	QFileInfo fi(filename);
	QString extension = fi.completeSuffix();

	QImage* img = vW->getImage();
	return img->save(filename, extension.toStdString().c_str());
}

void ImageViewer::linkSlidersWithSpinBoxes() {
	connect(ui->horizontalSliderSourceX, &QSlider::valueChanged, ui->spinBoxSourceX, &QSpinBox::setValue);
	connect(ui->spinBoxSourceX, QOverload<int>::of(&QSpinBox::valueChanged), ui->horizontalSliderSourceX, &QSlider::setValue);

	connect(ui->horizontalSliderSourceY, &QSlider::valueChanged, ui->spinBoxSourceY, &QSpinBox::setValue);
	connect(ui->spinBoxSourceY, QOverload<int>::of(&QSpinBox::valueChanged), ui->horizontalSliderSourceY, &QSlider::setValue);

	connect(ui->horizontalSliderSourceZ, &QSlider::valueChanged, ui->spinBoxSourceZ, &QSpinBox::setValue);
	connect(ui->spinBoxSourceZ, QOverload<int>::of(&QSpinBox::valueChanged), ui->horizontalSliderSourceZ, &QSlider::setValue);

	connect(ui->horizontalSliderSourceColorR, &QSlider::valueChanged, ui->spinBoxSourceColorR, &QSpinBox::setValue);
	connect(ui->spinBoxSourceColorR, QOverload<int>::of(&QSpinBox::valueChanged), ui->horizontalSliderSourceColorR, &QSlider::setValue);

	connect(ui->horizontalSliderSourceColorG, &QSlider::valueChanged, ui->spinBoxSourceColorG, &QSpinBox::setValue);
	connect(ui->spinBoxSourceColorG, QOverload<int>::of(&QSpinBox::valueChanged), ui->horizontalSliderSourceColorG, &QSlider::setValue);

	connect(ui->horizontalSliderSourceColorB, &QSlider::valueChanged, ui->spinBoxSourceColorB, &QSpinBox::setValue);
	connect(ui->spinBoxSourceColorB, QOverload<int>::of(&QSpinBox::valueChanged), ui->horizontalSliderSourceColorB, &QSlider::setValue);

	connect(ui->horizontalSliderAmbientColorR, &QSlider::valueChanged, ui->spinBoxAmbientColorR, &QSpinBox::setValue);
	connect(ui->spinBoxAmbientColorR, QOverload<int>::of(&QSpinBox::valueChanged), ui->horizontalSliderAmbientColorR, &QSlider::setValue);

	connect(ui->horizontalSliderAmbientColorG, &QSlider::valueChanged, ui->spinBoxAmbientColorG, &QSpinBox::setValue);
	connect(ui->spinBoxAmbientColorG, QOverload<int>::of(&QSpinBox::valueChanged), ui->horizontalSliderAmbientColorG, &QSlider::setValue);

	connect(ui->horizontalSliderAmbientColorB, &QSlider::valueChanged, ui->spinBoxAmbientColorB, &QSpinBox::setValue);
	connect(ui->spinBoxAmbientColorB, QOverload<int>::of(&QSpinBox::valueChanged), ui->horizontalSliderAmbientColorB, &QSlider::setValue);
}

Lighting ImageViewer::setUpPrime() {
	int x = ui->spinBoxSourceX->value();
	int y = ui->spinBoxSourceY->value();
	int z = ui->spinBoxSourceZ->value();
	unsigned char r = static_cast<unsigned char>(ui->spinBoxSourceColorR->value());
	unsigned char g = static_cast<unsigned char>(ui->spinBoxSourceColorG->value());
	unsigned char b = static_cast<unsigned char>(ui->spinBoxSourceColorB->value());
	Light source(r, g, b, x, y, z);

	r = static_cast<unsigned char>(ui->spinBoxAmbientColorR->value());
	g = static_cast<unsigned char>(ui->spinBoxAmbientColorG->value());
	b = static_cast<unsigned char>(ui->spinBoxAmbientColorB->value());

	float coeffDiffR = ui->doubleSpinBoxDiffusionR->value();
	float coeffDiffG = ui->doubleSpinBoxDiffusionG->value();
	float coeffDiffB = ui->doubleSpinBoxDiffusionB->value();
	Light diffusion(coeffDiffR, coeffDiffG, coeffDiffB);

	float coeffRefR = ui->doubleSpinBoxReflectionR->value();
	float coeffRefG = ui->doubleSpinBoxReflectionG->value();
	float coeffRefB = ui->doubleSpinBoxReflectionB->value();
	Light reflection(coeffRefR, coeffRefG, coeffRefB);

	float coeffAmbR = ui->doubleSpinBoxAmbientR->value();
	float coeffAmbG = ui->doubleSpinBoxAmbientG->value();
	float coeffAmbB = ui->doubleSpinBoxAmbientB->value();
	Light ambient(r, g, b, 0, 0, 0, coeffAmbR, coeffAmbG, coeffAmbB);

	Lighting prime(source, diffusion, reflection, ambient);

	return prime;
}

void ImageViewer::renderObject3D() {
	Lighting prime = setUpPrime();

	if (ui->checkBoxParallelProjection->isChecked()) {
		if (ui->checkBoxCube->isChecked()) {
			if (ui->checkBoxWireFrame->isChecked()) {
				vW->renderParallelProjection(theta, phi, objectLoaded,ui->comboBoxLineAlg->currentIndex(), isCubeWireFrame, parallel, ui->doubleSpinBoxDistance->value(), ui->spinBoxCameraDistance->value(), prime, ui->comboBoxShading->currentIndex(), ui->spinBoxSharpness->value());
			}
			else {
				vW->renderParallelProjection(theta, phi, objectLoaded, ui->comboBoxLineAlg->currentIndex(), isCube, parallel, ui->doubleSpinBoxDistance->value(), ui->spinBoxCameraDistance->value(), prime, ui->comboBoxShading->currentIndex(), ui->spinBoxSharpness->value());
			}
		}
		else {
			if (ui->checkBoxWireFrame->isChecked()) {
				vW->renderParallelProjection(theta, phi, objectLoaded, ui->comboBoxLineAlg->currentIndex(), isSphereWireFrame, parallel, ui->doubleSpinBoxDistance->value(), ui->spinBoxCameraDistance->value(), prime, ui->comboBoxShading->currentIndex(), ui->spinBoxSharpness->value());
			}
			else {
				vW->renderParallelProjection(theta, phi, objectLoaded, ui->comboBoxLineAlg->currentIndex(), isSphere, parallel, ui->doubleSpinBoxDistance->value(), ui->spinBoxCameraDistance->value(), prime, ui->comboBoxShading->currentIndex(), ui->spinBoxSharpness->value());
			}
		}
	}	 
	else {
		if (ui->checkBoxCube->isChecked()) {
			if (ui->checkBoxWireFrame->isChecked()) {
				vW->renderPerspectiveProjection(theta, phi, objectLoaded, ui->comboBoxLineAlg->currentIndex(), isCubeWireFrame, perspective, ui->doubleSpinBoxDistance->value(), ui->spinBoxCameraDistance->value(), prime, ui->comboBoxShading->currentIndex(), ui->spinBoxSharpness->value());
			}
			else {
				vW->renderPerspectiveProjection(theta, phi, objectLoaded, ui->comboBoxLineAlg->currentIndex(), isCube, perspective, ui->doubleSpinBoxDistance->value(), ui->spinBoxCameraDistance->value(), prime, ui->comboBoxShading->currentIndex(), ui->spinBoxSharpness->value());
			}
		}
		else {
			if (ui->checkBoxWireFrame->isChecked()) {
				vW->renderPerspectiveProjection(theta, phi, objectLoaded, ui->comboBoxLineAlg->currentIndex(), isSphereWireFrame, perspective, ui->doubleSpinBoxDistance->value(), ui->spinBoxCameraDistance->value(), prime, ui->comboBoxShading->currentIndex(), ui->spinBoxSharpness->value());
			}
			else {
				vW->renderPerspectiveProjection(theta, phi, objectLoaded, ui->comboBoxLineAlg->currentIndex(), isSphere, perspective, ui->doubleSpinBoxDistance->value(), ui->spinBoxCameraDistance->value(), prime, ui->comboBoxShading->currentIndex(), ui->spinBoxSharpness->value());
			}
		}
	}
}

//-----------------------------------------
//		*** Tools 2D slots ***
//-----------------------------------------

void ImageViewer::on_actionSave_as_triggered()
{
	QString folder = settings.value("folder_img_save_path", "").toString();

	QString fileFilter = "Image data (*.bmp *.gif *.jpg *.jpeg *.png *.pbm *.pgm *.ppm .*xbm .* xpm);;All files (*)";
	QString fileName = QFileDialog::getSaveFileName(this, "Save image", folder, fileFilter);
	if (!fileName.isEmpty()) {
		QFileInfo fi(fileName);
		settings.setValue("folder_img_save_path", fi.absoluteDir().absolutePath());

		if (!saveImage(fileName)) {
			msgBox.setText("Unable to save image.");
			msgBox.setIcon(QMessageBox::Warning);
		}
		else {
			msgBox.setText(QString("File %1 saved.").arg(fileName));
			msgBox.setIcon(QMessageBox::Information);
		}
		msgBox.exec();
	}
}

void ImageViewer::on_actionClear_triggered()
{
	vW->clearVectorOfPoints();
	vW->clearVectorOfCurvePoints();
	vW->clearVectorOfTangents();
	polygonDone = false;
	lineDone = false;
	curveDone = false;
	objectLoaded = false;

	ui->toolButtonDrawPolygon->setDisabled(false);
	ui->toolButtonDrawCurve->setDisabled(false);
	ui->toolButtonDrawCurve->setChecked(false);
	ui->toolButtonDrawPolygon->setChecked(false);
	ui->pushButtonMove->setChecked(false);
	ui->checkBoxScale->setChecked(false);
	ui->checkBoxAxialSymmetry->setChecked(false);
	ui->checkBoxCube->setChecked(false);
	ui->checkBoxSphere->setChecked(false);
	ui->spinBoxParallels->setValue(0);
	ui->spinBoxMeridians->setValue(0);
	ui->doubleSpinBoxSphereRadius->setValue(0.0);
	ui->doubleSpinBoxCube->setValue(20.0);
	ui->horizontalSliderZenith->setValue(0.0);
	ui->horizontalSliderAzimuth->setValue(0.0);
	ui->checkBoxCentralProjection->setChecked(false);
	ui->checkBoxParallelProjection->setChecked(false);
	ui->checkBoxWireFrame->setChecked(false);
	ui->doubleSpinBoxDistance->setValue(0.0);
	vW->clear();
}

void ImageViewer::on_actionExit_triggered()
{
	this->close();
}

void ImageViewer::on_pushButtonSetColor_clicked()
{
	QColor newColor = QColorDialog::getColor(globalColor, this);
	if (newColor.isValid()) {
		QString style_sheet = QString("background-color: #%1;").arg(newColor.rgba(), 0, 16);
		ui->pushButtonSetColor->setStyleSheet(style_sheet);
		globalColor = newColor;
	}
}

void ImageViewer::on_pushButtonColor1_clicked()
{
	QColor newColor = QColorDialog::getColor(color1, this);
	if (newColor.isValid()) {
		QString style_sheet = QString("background-color: #%1;").arg(newColor.rgba(), 0, 16);
		ui->pushButtonColor1->setStyleSheet(style_sheet);
		color1 = newColor;
	}
}

void ImageViewer::on_pushButtonColor2_clicked()
{
	QColor newColor = QColorDialog::getColor(color2, this);
	if (newColor.isValid()) {
		QString style_sheet = QString("background-color: #%1;").arg(newColor.rgba(), 0, 16);
		ui->pushButtonColor2->setStyleSheet(style_sheet);
		color2 = newColor;
	}
}

void ImageViewer::on_pushButtonTurn_clicked() {
	if (ui->toolButtonDrawPolygon->isChecked()) {
		vW->turnPolygon(globalColor, color1, color2, ui->comboBoxLineAlg->currentIndex(), ui->spinBoxTurn->value(), ui->comboBoxInterpolation->currentIndex());
	}
	if (ui->toolButtonDrawLine->isChecked()) {
		vW->turnLine(globalColor, ui->comboBoxLineAlg->currentIndex(), ui->spinBoxTurn->value());
	}
	if (ui->toolButtonDrawCurve->isChecked()) {
		vW->turnCurve(globalColor, color2, ui->comboBoxLineAlg->currentIndex(), ui->spinBoxTurn->value(), ui->comboBoxCurveType->currentIndex(), ui->toolButtonDrawCurve->isChecked());
	}
}

void ImageViewer::on_pushButtonScale_clicked() {
	if (ui->toolButtonDrawPolygon->isChecked()) {
		vW->scalePolygon(globalColor, color1, color2, ui->comboBoxLineAlg->currentIndex(), ui->doubleSpinBoxScaleX->value(), ui->doubleSpinBoxScaleY->value(), ui->comboBoxInterpolation->currentIndex());
	}
	if (ui->toolButtonDrawLine->isChecked()) {
		vW->scaleLine(globalColor, ui->comboBoxLineAlg->currentIndex(), ui->doubleSpinBoxScaleX->value(), ui->doubleSpinBoxScaleY->value());
	}
	if (ui->toolButtonDrawCurve->isChecked()) {
		vW->scaleCurve(globalColor, color2, ui->comboBoxLineAlg->currentIndex(), ui->doubleSpinBoxScaleX->value(), ui->doubleSpinBoxScaleY->value(), ui->comboBoxCurveType->currentIndex(), ui->toolButtonDrawCurve->isChecked());
	}
}

void ImageViewer::on_checkBoxAxialSymmetry_toggled(bool checked) {
	int object;
	if (ui->toolButtonDrawLine->isChecked())
		object = 0;
	else if (ui->toolButtonDrawPolygon->isChecked())
		object = 1;
	else if (ui->toolButtonDrawCurve->isChecked())
		object = 2;
	
	if (checked)
	{
		vW->axialSymmetry(globalColor, color1, color2, ui->comboBoxLineAlg->currentIndex(), object, ui->comboBoxInterpolation->currentIndex(),ui->comboBoxCurveType->currentIndex());
	}
	else {
		vW->restoreOriginalState(globalColor, color1, color2, ui->comboBoxLineAlg->currentIndex(), object, ui->comboBoxInterpolation->currentIndex(), ui->comboBoxCurveType->currentIndex());
	}
}

void ImageViewer::on_pushButtonShear_clicked() {
	if (ui->toolButtonDrawPolygon->isChecked()) {
		vW->shearObject(globalColor, color1, color2, ui->comboBoxLineAlg->currentIndex(), ui->doubleSpinBoxShearCoefficient->value(), ui->comboBoxInterpolation->currentIndex());
	}
}

//-----------------------------------------
//		*** Tools 3D slots ***
//-----------------------------------------

void ImageViewer::on_pushButton3DTools_clicked() {
	int page3DIndex = 1;
	ui->stackedWidget->setCurrentIndex(page3DIndex);
}

void ImageViewer::on_pushButton2DTools_clicked() {
	int page2DIndex = 0;
	ui->stackedWidget->setCurrentIndex(page2DIndex);
}

void ImageViewer::on_pushButton3DColor_clicked() {
	QColor newColor = QColorDialog::getColor(color3D, this);
	if (newColor.isValid()) {
		QString style_sheet = QString("background-color: #%1;").arg(newColor.rgba(), 0, 16);
		ui->pushButtonSetColor->setStyleSheet(style_sheet);
		color3D = newColor;
	}
}

void ImageViewer::on_pushButtonSaveVTK3D_clicked() {
	if (ui->checkBoxCube->isChecked()) {
		vW->saveVTKCube(ui->doubleSpinBoxCube->value());
	}
	else if (ui->checkBoxSphere->isChecked()) {
		vW->saveSphereVTK(ui->spinBoxParallels->value(), ui->spinBoxMeridians->value(),ui->doubleSpinBoxSphereRadius->value());
	}
}

void ImageViewer::on_pushButtonOpenVTK3D_clicked() {
	if (ui->checkBoxCube->isChecked()) {
		vW->loadCubeVTK();
		objectLoaded = true;
	}
	else {
		vW->loadSphereVTK();
		objectLoaded = true;
	}
	renderObject3D();
}

void ImageViewer::on_horizontalSliderAzimuth_valueChanged(int value) {
	phi = vW->mapSliderValueToDegrees(value, ui->horizontalSliderAzimuth->minimum(), ui->horizontalSliderAzimuth->maximum());
	renderObject3D();
}

void ImageViewer::on_horizontalSliderZenith_valueChanged(int value) {
	theta = vW->mapSliderValueToDegrees(value, ui->horizontalSliderZenith->minimum(), ui->horizontalSliderZenith->maximum());
	renderObject3D();
}

void ImageViewer::on_pushButtonLighting_clicked() {
	ui->stackedWidget->setCurrentIndex(2);
}

void ImageViewer::on_pushButtonBack_clicked() {
	ui->stackedWidget->setCurrentIndex(1);
}

void ImageViewer::on_pushButtonApplyChanges_clicked() {
	if (objectLoaded) {
		renderObject3D();
	}
	else {
		QMessageBox::warning(this, "Nenacitane data", "Nie su nacitane ziadne data na vykreslenie.");
		return;
	}
}