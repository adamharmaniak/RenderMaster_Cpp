#include   "ViewerWidget.h"

ViewerWidget::ViewerWidget(QSize imgSize, QWidget* parent)
	: QWidget(parent)
{
	setAttribute(Qt::WA_StaticContents);
	setMouseTracking(true);
	if (imgSize != QSize(0, 0)) {
		img = new QImage(imgSize, QImage::Format_ARGB32);
		img->fill(Qt::white);
		resizeWidget(img->size());
		setPainter();
		setDataPtr();
	}
}
ViewerWidget::~ViewerWidget()
{
	delete painter;
	delete img;
}
void ViewerWidget::resizeWidget(QSize size)
{
	this->resize(size);
	this->setMinimumSize(size);
	this->setMaximumSize(size);
}

//-----------------------------------------
//		*** Image functions ***
//-----------------------------------------

bool ViewerWidget::setImage(const QImage& inputImg)
{
	if (img != nullptr) {
		delete painter;
		delete img;
	}
	img = new QImage(inputImg);
	if (!img) {
		return false;
	}
	resizeWidget(img->size());
	setPainter();
	setDataPtr();
	update();

	return true;
}
bool ViewerWidget::isEmpty()
{
	if (img == nullptr) {
		return true;
	}

	if (img->size() == QSize(0, 0)) {
		return true;
	}
	return false;
}

bool ViewerWidget::changeSize(int width, int height)
{
	QSize newSize(width, height);

	if (newSize != QSize(0, 0)) {
		if (img != nullptr) {
			delete painter;
			delete img;
		}

		img = new QImage(newSize, QImage::Format_ARGB32);
		if (!img) {
			return false;
		}
		img->fill(Qt::white);
		resizeWidget(img->size());
		setPainter();
		setDataPtr();
		update();
	}

	return true;
}

void ViewerWidget::clear()
{
	img->fill(Qt::white);
	update();
}

void ViewerWidget::paintEvent(QPaintEvent* event)
{
	QPainter painter(this);
	QRect area = event->rect();
	painter.drawImage(area, *img, area);
}

//-----------------------------------------
//		*** Point drawing functions ***
//-----------------------------------------

void ViewerWidget::setPixel(int x, int y, uchar r, uchar g, uchar b, uchar a)
{
	r = r > 255 ? 255 : (r < 0 ? 0 : r);
	g = g > 255 ? 255 : (g < 0 ? 0 : g);
	b = b > 255 ? 255 : (b < 0 ? 0 : b);
	a = a > 255 ? 255 : (a < 0 ? 0 : a);

	size_t startbyte = y * img->bytesPerLine() + x * 4;
	data[startbyte] = b;
	data[startbyte + 1] = g;
	data[startbyte + 2] = r;
	data[startbyte + 3] = a;
}
void ViewerWidget::setPixel(int x, int y, double valR, double valG, double valB, double valA)
{
	valR = valR > 1 ? 1 : (valR < 0 ? 0 : valR);
	valG = valG > 1 ? 1 : (valG < 0 ? 0 : valG);
	valB = valB > 1 ? 1 : (valB < 0 ? 0 : valB);
	valA = valA > 1 ? 1 : (valA < 0 ? 0 : valA);

	size_t startbyte = y * img->bytesPerLine() + x * 4;
	data[startbyte] = static_cast<uchar>(255 * valB);
	data[startbyte + 1] = static_cast<uchar>(255 * valG);
	data[startbyte + 2] = static_cast<uchar>(255 * valR);
	data[startbyte + 3] = static_cast<uchar>(255 * valA);
}
void ViewerWidget::setPixel(int x, int y, const QColor& color)
{
	if (!color.isValid() || x < 0 || y < 0 || x >= img->width() || y >= img->height()) {
		return;
	}

	size_t startbyte = y * img->bytesPerLine() + x * 4;

	data[startbyte] = color.blue();
	data[startbyte + 1] = color.green();
	data[startbyte + 2] = color.red();
	data[startbyte + 3] = color.alpha();
}

void ViewerWidget::setPixel(int x, int y, const QColor& color, bool isCurve) {
	if (!color.isValid() || x < 0 || y < 0 || x >= img->width() || y >= img->height()) {
		return;
	}

	if (isCurve) {
		int radius = 5;
		QPoint center(x, y);
		QPoint radiusPoint(x + radius, y);

		drawCircle(center, radiusPoint, color);
	}
	else {
		return;
	}
}

//-----------------------------------------
//		*** Drawing functions ***
//-----------------------------------------

void ViewerWidget::drawCircle(QPoint center, QPoint radiusPoint, QColor color) {
	painter->setPen(QPen(color));

	int r = sqrt(pow(radiusPoint.x() - center.x(), 2) + pow(radiusPoint.y() - center.y(), 2));
	int x = 0;
	int y = r;
	int p = 1 - r;

	drawSymmetricPoints(center, x, y);

	while (x < y) {
		x++;
		if (p < 0) {
			p += 2 * x + 1;
		}
		else {
			y--;
			p += 2 * (x - y) + 1;
		}
		drawSymmetricPoints(center, x, y);
	}

	update();
}

void ViewerWidget::drawSymmetricPoints(const QPoint& center, int x, int y) {
	QPoint points[8] = {
		QPoint(x, y),
		QPoint(y, x),
		QPoint(-x, y),
		QPoint(-y, x),
		QPoint(-x, -y),
		QPoint(-y, -x),
		QPoint(x, -y),
		QPoint(y, -x)
	};

	for (auto& point : points) {
		painter->drawPoint(center.x() + point.x(), center.y() + point.y());
	}
}

void ViewerWidget::drawLine(QPoint start, QPoint end, QColor color, int algType)
{
	linePoints.clear();
	painter->setPen(QPen(color));

	QVector<QPoint> lineToClip;
	lineToClip.append(start);
	lineToClip.append(end);

	clipLineWithPolygon(lineToClip);

	if (lineToClip.size() == 2) {
		linePoints.append(lineToClip[0]);
		linePoints.append(lineToClip[1]);
	}
	else {
		linePoints.append(start);
		linePoints.append(end);
	}

	if (algType == 0) {
		drawLineDDA(linePoints);
	}
	else {
		drawLineBresenham(linePoints);
	}

	update();
}

void ViewerWidget::drawLineDDA(QVector<QPoint>& linePoints) {
	int dx = linePoints.last().x() - linePoints.first().x();
	int dy = linePoints.last().y() - linePoints.first().y();

	if (abs(dx) > abs(dy)) {
		double m = static_cast<double>(dy) / static_cast<double>(dx);
		double y = linePoints.first().y();

		if (dx > 0) {
			for (int x = linePoints.first().x(); x <= linePoints.last().x(); x++) {
				painter->drawPoint(x, static_cast<int>(round(y)));
				y += m;
			}
		}
		else {
			for (int x = linePoints.first().x(); x >= linePoints.last().x(); x--) {
				painter->drawPoint(x, static_cast<int>(round(y)));
				y -= m;
			}
		}
	}
	else {
		double m = static_cast<double>(dx) / static_cast<double>(dy);
		double x = linePoints.first().x();
		if (dy > 0) {
			for (int y = linePoints.first().y(); y <= linePoints.last().y(); y++) {
				painter->drawPoint(static_cast<int>(round(x)),y);
				x += m;
			}
		}
		else {
			for (int y = linePoints.first().y(); y >= linePoints.last().y(); y--) {
				painter->drawPoint(static_cast<int>(round(x)), y);
				x -= m;
			}
		}
	}

	update();
}

void ViewerWidget::drawLineBresenham(QVector<QPoint>& linePoints) {
	int p, k1, k2;
	int dx = linePoints.last().x() - linePoints.first().x();
	int dy = linePoints.last().y() - linePoints.first().y();

	int adx = abs(dx);
	int ady = abs(dy);

	int x = linePoints.first().x();
	int y = linePoints.first().y();

	int incrementX = (dx > 0) ? 1 : -1;
	int incrementY = (dy > 0) ? 1 : -1;

	if (adx > ady) {
		p = 2 * ady - adx;
		k1 = 2 * ady;
		k2 = 2 * (ady - adx);

		while (x != linePoints.last().x()) {
			painter->drawPoint(x, y);
			x += incrementX;
			if (p >= 0) {
				y += incrementY;
				p += k2;
			}
			else {
				p += k1;
			}
		}
	}
	else {
		p = 2 * adx - ady;
		k1 = 2 * adx;
		k2 = 2 * (adx - ady);

		while (y != linePoints.last().y()) {
			painter->drawPoint(x, y);
			y += incrementY;
			if (p >= 0) {
				x += incrementX;
				p += k2;
			}
			else {
				p += k1;
			}
		}
	}

	painter->drawPoint(linePoints.last().x(), linePoints.last().y());
	update();
}

void ViewerWidget::drawPolygon(const QColor& color, const QColor& color1, const QColor& color2, int algType, int interpolType) {
	if (pointsVector.size() < 2) {
		QMessageBox::warning(this, "Nizky pocet bodov", "Nebol dosiahnuty minimalny pocet bodov pre vykreslenie polygonu.");
		return;
	}

	painter->setPen(QPen(color));
	QVector<QPoint> polygon = pointsVector;
	fillPolygon(pointsVector, color, color1, color2, interpolType);

	bool allPointsOutside = std::all_of(pointsVector.begin(), pointsVector.end(), [this](const QPoint& point) {
		return !isInside(point);
		});

	if (allPointsOutside) {
		qDebug() << "Polygon je mimo hranicu.";
		return;
	}

	for (QPoint point : pointsVector) {
		if (!isInside(point)) {
			polygon = trimPolygon();
			break;
		}
	}

	if (!polygon.isEmpty()) {
		for (int i = 0; i < polygon.size() - 1; i++) {
			drawLine(polygon.at(i), polygon.at(i + 1), color, algType);
		}
		drawLine(polygon.last(), polygon.first(), color, algType);
	}

	update();
}

//-----------------------------------------
//		*** Moving functions ***
//-----------------------------------------

void ViewerWidget::moveLine(const QColor& color, const QPoint& offset, int algType) {
	if (!linePoints.isEmpty()) {
		for (QPoint& point : linePoints) {
			point += offset;
		}
		updateLine(color, algType);
	}
}

void ViewerWidget::updateLine(const QColor& color, int algType) {
	clear();
	drawLine(linePoints.first(), linePoints.last(), color, algType);
	update();
}

void ViewerWidget::movePolygon(const QColor& color, const QColor& color1, const QColor& color2, const QPoint& offset, int algType, int interpolType) {
	if (!pointsVector.isEmpty()) {
		for (QPoint& point : pointsVector) {
			point += offset;
		}
		updatePolygon(color, color1, color2, algType, interpolType);
	}
}

void ViewerWidget::updatePolygon(const QColor& color, const QColor& color1, const QColor& color2, int algType, int interpolType) {
	clear();
	drawPolygon(color, color1, color2, algType, interpolType);
	update();
}

//-----------------------------------------
//		*** Turning functions ***
//-----------------------------------------

void ViewerWidget::turnLine(const QColor& color, int algType, int angle) {
	QPoint center = getLineCenter();
	double radians = qDegreesToRadians(static_cast<double>(angle));

	double cosAngle = cos(radians);
	double sinAngle = sin(radians);

	QVector<QPoint> rotatedPoints;

	for (QPoint& point : linePoints) {
		int translatedX = point.x() - center.x();
		int translatedY = point.y() - center.y();

		int rotatedX = static_cast<int>(translatedX * cosAngle - translatedY * sinAngle);
		int rotatedY = static_cast<int>(translatedX * sinAngle + translatedY * cosAngle);

		rotatedX += center.x();
		rotatedY += center.y();

		rotatedPoints.push_back(QPoint(rotatedX, rotatedY));
	}

	linePoints = rotatedPoints;

	clear();
	drawLine(linePoints.first(), linePoints.last(), color, algType);
	update();
}

void ViewerWidget::turnPolygon(const QColor& color, const QColor& color1, const QColor& color2, int algType, int angle, int interpolType) {
	QPoint center = getPolygonCenter();
	double radians = qDegreesToRadians(static_cast<double>(angle));

	double cosAngle = cos(radians);
	double sinAngle = sin(radians);

	QVector<QPoint> rotatedPoints;

	for (QPoint& point : pointsVector) {
		int translatedX = point.x() - center.x();
		int translatedY = point.y() - center.y();

		int rotatedX = static_cast<int>(translatedX * cosAngle - translatedY * sinAngle);
		int rotatedY = static_cast<int>(translatedX * sinAngle + translatedY * cosAngle);

		rotatedX += center.x();
		rotatedY += center.y();

		rotatedPoints.push_back(QPoint(rotatedX, rotatedY));
	}

	pointsVector = rotatedPoints;

	clear();
	drawPolygon(color, color1, color2, algType, interpolType);
	update();
}

//-----------------------------------------
//		*** Scaling functions ***
//-----------------------------------------

QPoint ViewerWidget::getLineCenter() const {
	if (linePoints.isEmpty()) {
		return QPoint();
	}

	double centroidX = (linePoints.first().x() + linePoints.last().x()) / 2;
	double centroidY = (linePoints.first().y() + linePoints.last().y()) / 2;

	return QPoint(static_cast<int>(centroidX), static_cast<int>(centroidY));
}

void ViewerWidget::scaleLine(const QColor& color, int algType, double scaleX, double scaleY) {
	QPoint center = getLineCenter();

	for (QPoint& point : linePoints) {
		double newX = center.x() + (point.x() - center.x()) * scaleX;
		double newY = center.y() + (point.y() - center.y()) * scaleY;

		point.setX(round(newX));
		point.setY(round(newY));
	}

	clear();
	drawLine(linePoints.first(), linePoints.last(), color, algType);
	update();
}

QPoint ViewerWidget::getPolygonCenter() const {
	if (pointsVector.isEmpty()) {
		return QPoint();
	}
	
	double centroidX = 0.0;
	double centroidY = 0.0;
	int numPoints = pointsVector.size();

	for (const QPoint& point : pointsVector) {
		centroidX += point.x();
		centroidY += point.y();
	}

	centroidX /= numPoints;
	centroidY /= numPoints;
	return QPoint(static_cast<int>(centroidX), static_cast<int>(centroidY));
}

void ViewerWidget::scalePolygon(const QColor& color, const QColor& color1, const QColor& color2, int algType, double scaleX, double scaleY, int interpolType) {
	QPoint center = getPolygonCenter();

	for (QPoint& point : pointsVector) {
		double newX = center.x() + (point.x() - center.x()) * scaleX;
		double newY = center.y() + (point.y() - center.y()) * scaleY;

		point.setX(static_cast<int>(round(newX)));
		point.setY(static_cast<int>(round(newY)));
	}

	clear();
	drawPolygon(color, color1, color2, algType, interpolType);
	update();
}

//-----------------------------------------
//		*** Axial Symmetry functions ***
//-----------------------------------------

void ViewerWidget::axialSymmetry(const QColor& color, const QColor& color1, const QColor& color2, int algType, int object, int interpolType, int curveType) {
	saveOriginalState(object);

	if (object == 0) {
		int axisY = height() / 2;

		for (QPoint& point : linePoints) {
			int mirroredY = 2 * axisY - point.y();
			point.setY(mirroredY);
		}

		clear();
		drawLine(linePoints.first(), linePoints.last(), color, algType);
		update();
	}
	else if(object == 1) {
		QPoint center = getPolygonCenter();

		int axisX = center.x();
		int axisY = center.y();

		for (QPoint& point : pointsVector) {
			int mirroredX = 2 * axisX - point.x();
			int mirroredY = 2 * axisY - point.y();

			point.setX(mirroredX);
			point.setY(mirroredY);
		}

		clear();
		drawPolygon(color, color1, color2, algType, interpolType);
		update();
	}
	else if (object == 2) {
		int axisX = width() / 2;

		clear();
		for (QPoint& point : curvePoints) {
			int mirroredX = 2 * axisX - point.x();
			point.setX(mirroredX);
			setPixel(point.x(), point.y(), color2, true);
		}

		drawCurve(color, color2, curveType, algType);
		update();
	}
}

void ViewerWidget::saveOriginalState(int object) {
	if (object == 0)
		originalPointsVector = linePoints;
	else if (object == 1)
		originalPointsVector = pointsVector;
	else if (object == 2)
		originalPointsVector = curvePoints;
}

void ViewerWidget::restoreOriginalState(const QColor& color, const QColor& color1, const QColor& color2, int algType, int object, int interpolType, int curveType) {
	
	if (object == 0) {
		if (!originalPointsVector.isEmpty()) {
			linePoints = originalPointsVector;
			originalPointsVector.clear();

			clear();
			drawLine(linePoints.first(), linePoints.last(), color, algType);
			update();
		}
	}
	else if (object == 1) {
		if (!originalPointsVector.isEmpty()) {
			pointsVector = originalPointsVector;
			originalPointsVector.clear();

			clear();
			drawPolygon(color, color1, color2, algType, interpolType);
			update();
		}
	}
	else if (object == 2) {
		curvePoints = originalPointsVector;
		originalPointsVector.clear();

		clear();
		for (QPoint& point : curvePoints) {
			setPixel(point.x(), point.y(), color2, true);
		}
		drawCurve(color, color2, curveType, algType);
		update();
	}
}

//-----------------------------------------

void ViewerWidget::shearObject(const QColor& color, const QColor& color1, const QColor& color2, int algType, double coefficient, int interpolType) {
	QPoint center = getPolygonCenter();

	for (QPoint& point : pointsVector) {
		int newX = point.x() + static_cast<int>((point.y() - center.y()) * coefficient);
		point.setX(newX);
	}

	clear();
	drawPolygon(color, color1, color2, algType, interpolType);
	update();
}
//-----------------------------------------
//		*** Polygon trimming function ***
//-----------------------------------------

QVector<QPoint> ViewerWidget::trimPolygon() {
	if (pointsVector.isEmpty()) {
		qDebug() << "pointsVector je prazdny";
		return QVector<QPoint>();
	}

	QVector<QPoint> W, polygon = pointsVector;
	QPoint S;

	int xMin[] = { 0,0,-499,-499 };

	for (int i = 0; i < 4; i++) {
		if (pointsVector.size() == 0) {
			return polygon;
		}

		S = polygon[polygon.size() - 1];

		for (int j = 0; j < polygon.size(); j++) {
			if (polygon[j].x() >= xMin[i]) {
				if (S.x() >= xMin[i]) {
					W.push_back(polygon[j]);
				}
				else {
					QPoint P(xMin[i], S.y() + (xMin[i] - S.x()) * ((polygon[j].y() - S.y()) / static_cast<double>((polygon[j].x() - S.x()))));
					W.push_back(P);
					W.push_back(polygon[j]);
				}
			}
			else {
				if (S.x() >= xMin[i]) {
					QPoint P(xMin[i], S.y() + (xMin[i] - S.x()) * ((polygon[j].y() - S.y()) / static_cast<double>((polygon[j].x() - S.x()))));
					W.push_back(P);
				}
			}
			S = polygon[j];
		}
		polygon = W;
		W.clear();

		for (int j = 0; j < polygon.size(); j++) {
			QPoint swappingPoint = polygon[j];
			polygon[j].setX(swappingPoint.y());
			polygon[j].setY(-swappingPoint.x());
		}
	}

	return polygon;
}


//-----------------------------------------
//		*** Line trimming function ***
//-----------------------------------------

void ViewerWidget::clipLineWithPolygon(QVector<QPoint> linePoints) {
	if (linePoints.size() < 2) {
		return;
	}

	QVector<QPoint> clippedPoints;
	QPoint P1 = linePoints[0], P2 = linePoints[1];
	double t_min = 0, t_max = 1;
	QPoint d = P2 - P1;

	QVector<QPoint> E = { QPoint(0,0), QPoint(500,0), QPoint(500,500), QPoint(0,500) };

	for (int i = 0; i < E.size(); i++) {
		QPoint E1 = E[i];
		QPoint E2 = E[(i + 1) % E.size()];

		QPoint normal = QPoint(E2.y() - E1.y(), E1.x() - E2.x());

		QPoint w = P1 - E1;

		double dn = d.x() * normal.x() + d.y() * normal.y();
		double wn = w.x() * normal.x() + w.y() * normal.y();
		if (dn != 0) {
			double t = -wn / dn;
			if (dn > 0 && t <= 1) {
				t_min = std::max(t, t_min);
			}
			else if (dn < 0 && t >= 0) {
				t_max = std::min(t, t_max);
			}
		}
	}

	if (t_min < t_max) {
		QPoint clippedP1 = P1 + (P2 - P1) * t_min;
		QPoint clippedP2 = P1 + (P2 - P1) * t_max;

		clippedPoints.push_back(clippedP1);
		clippedPoints.push_back(clippedP2);
	}
	else {
		//qDebug() << "Useckovy segment je uplne mimo orezovacej oblasti alebo je neplatny.";
	}

	if (!clippedPoints.isEmpty()) {
		linePoints = clippedPoints;
	}
}

//-----------------------------------------
//		*** Filling of Polygon ***
//-----------------------------------------

QVector<ViewerWidget::Edge> ViewerWidget::loadEdges(QVector<QPoint>& points) {
	QVector<Edge> edges;

	for (int i = 0; i < points.size(); i++) {
		QPoint startPoint = points[i];
		QPoint endPoint = points[(i + 1) % points.size()];

		Edge edge(startPoint, endPoint);
		edge.adjustEndPoint();
		edges.push_back(edge);
	}

	std::sort(edges.begin(), edges.end(), compareByY);
	return edges;
}

void ViewerWidget::fillPolygon(QVector<QPoint>& points, const QColor& color, const QColor& color1, const QColor& color2, int interpolType) {
	if (points.isEmpty()) {
		return;
	}

	if (points.size() == 3) {
		fillTriangle(points, color, color1, color2, interpolType);
	}
	else {
		QVector<Edge> edges = loadEdges(points);
		if (edges.isEmpty()) {
			return;
		}

		int yMin = edges.front().startPoint().y();
		int yMax = edges.front().endPoint().y();

		for (const Edge& edge : edges) {
			int y1 = edge.startPoint().y();
			int y2 = edge.endPoint().y();
			yMin = qMin(yMin, qMin(y1, y2));
			yMax = qMax(yMax, qMax(y1, y2));
		}

		if (yMin >= yMax) {
			return;
		}

		QVector<QVector<Edge>> TH(yMax - yMin + 1);

		for (const auto& edge : edges) {
			int index = edge.startPoint().y() - yMin;
			if (index < 0 || index >= TH.size()) {
				//qDebug() << "Invalid index:" << index << "for edge start point y:" << edge.startPoint().y();
				continue;
			}
			TH[index].append(edge);
		}

		QVector<Edge> activeEdgeList;
		for (int y = yMin; y <= yMax; y++) {
			for (const auto& edge : TH[y - yMin]) {
				activeEdgeList.append(edge);
			}

			std::sort(activeEdgeList.begin(), activeEdgeList.end(), [](const Edge& a, const Edge& b) {
				return a.x() < b.x();
				});

			for (int i = 0; i < activeEdgeList.size(); i += 2) {
				if (i + 1 < activeEdgeList.size()) {
					int startX = qRound(activeEdgeList[i].x());
					int endX = qRound(activeEdgeList[i + 1].x());
					for (int x = startX; x <= endX; x++) {
						setPixel(x, y, color);
					}
				}
			}

			QMutableVectorIterator<Edge> it(activeEdgeList);
			while (it.hasNext()) {
				Edge& edge = it.next();
				if (edge.endPoint().y() == y) {
					it.remove();
				}
				else {
					edge.setX(edge.x() + edge.w());
				}
			}
		}
	}
}

//-----------------------------------------
//		*** Filling of Triangle ***
//-----------------------------------------

void ViewerWidget::fillTriangle(QVector<QPoint>& points, const QColor& color, const QColor& color1, const QColor& color2, int interpolType) {
	std::sort(points.begin(), points.end(), [](const QPoint& a, const QPoint& b) {
		return a.y() < b.y() || (a.y() == b.y() && a.x() < b.x());
		});

	const QPoint& T0 = points[0];
	const QPoint& T1 = points[1];
	const QPoint& T2 = points[2];

	if (T0.y() == T1.y()) {
		fillFlatBottomTriangle(T0, T1, T2, color, color1, color2, interpolType);
	}
	else if (T1.y() == T2.y()) {
		fillFlatTopTriangle(T0, T1, T2, color, color1, color2, interpolType);
	}
	else {
		double m = (T2.x() - T0.x()) != 0 ? static_cast<double>(T2.y() - T0.y()) / (T2.x() - T0.x()) : std::numeric_limits<double>::max();

		QPoint P(
			m != std::numeric_limits<double>::max() ? static_cast<int>(T0.x() + (T1.y() - T0.y()) / m) : T0.x(),
			T1.y()
		);

		if (T1.x() < P.x()) {
			for (int y = T0.y(); y <= T1.y(); y++) {
				int x1 = static_cast<int>((y - T0.y()) * (T1.x() - T0.x()) / (T1.y() - T0.y()) + T0.x());
				int x2 = static_cast<int>((y - T0.y()) * (P.x() - T0.x()) / (P.y() - T0.y()) + T0.x());
				
				for (int x = x1; x <= x2; x++) {
					QColor activeColor;
					if (interpolType == 0) {
						double length1 = sqrt(pow(T0.x() - x, 2) + pow(T0.y() - y, 2));
						double length2 = sqrt(pow(T1.x() - x, 2) + pow(T1.y() - y, 2));
						double length3 = sqrt(pow(T2.x() - x, 2) + pow(T2.y() - y, 2));

						if (length1 <= length2 && length1 <= length3) {
							activeColor = color;
						}
						else if (length2 < length1 && length2 <= length3) {
							activeColor = color1;
						}
						else {
							activeColor = color2;
						}
					}
					else {
						QPoint P(x, y);
						activeColor = interpolateColorBarycentric(P, T0, T1, T2, color, color1, color2);
					}

					if (isPointInsideTriangle(QPoint(x, y), T0, T1, T2)) {
						setPixel(x, y, activeColor);
					}
				}
			}
			for (int y = T1.y(); y <= T2.y(); y++) {
				int x1 = static_cast<int>((y - T1.y()) * (T2.x() - T1.x()) / (T2.y() - T1.y()) + T1.x());
				int x2 = static_cast<int>((y - P.y()) * (T2.x() - P.x()) / (T2.y() - P.y()) + P.x());
				
				for (int x = x1; x <= x2; x++) {
					QColor activeColor;
					if (interpolType == 0) {
						//	<< Nearest neighbor algorithm >>

						double length1 = sqrt(pow(T0.x() - x, 2) + pow(T0.y() - y, 2));
						double length2 = sqrt(pow(T1.x() - x, 2) + pow(T1.y() - y, 2));
						double length3 = sqrt(pow(T2.x() - x, 2) + pow(T2.y() - y, 2));

						if (length1 <= length2 && length1 <= length3) {
							activeColor = color;
						}
						else if (length2 < length1 && length2 <= length3) {
							activeColor = color1;
						}
						else {
							activeColor = color2;
						}
					}
					else {
						//	<< Barycentric interpolation >>

						QPoint P(x, y);
						activeColor = interpolateColorBarycentric(P, T0, T1, T2, color, color1, color2);
					}

					QVector<QVector<double>> zBuffer(height(), QVector<double>(width(), std::numeric_limits<double>::lowest()));

					if (isPointInsideTriangle(QPoint(x, y), T0, T1, T2)) {
						setPixel(x, y, activeColor);
					}
				}
			}
		}
		else {
			for (int y = T0.y(); y <= T1.y(); y++) {
				int x1 = static_cast<int>((y - T0.y()) * (P.x() - T0.x()) / (P.y() - T0.y()) + T0.x());
				int x2 = static_cast<int>((y - T0.y()) * (T1.x() - T0.x()) / (T1.y() - T0.y()) + T0.x());
				
				for (int x = x1; x <= x2; x++) {
					QColor activeColor;
					if (interpolType == 0) {
						double length1 = sqrt(pow(T0.x() - x, 2) + pow(T0.y() - y, 2));
						double length2 = sqrt(pow(T1.x() - x, 2) + pow(T1.y() - y, 2));
						double length3 = sqrt(pow(T2.x() - x, 2) + pow(T2.y() - y, 2));

						if (length1 <= length2 && length1 <= length3) {
							activeColor = color;
						}
						else if (length2 < length1 && length2 <= length3) {
							activeColor = color1;
						}
						else {
							activeColor = color2;
						}
					}
					else {
						QPoint P(x, y);
						activeColor = interpolateColorBarycentric(P, T0, T1, T2, color, color1, color2);
					}

					if (isPointInsideTriangle(QPoint(x, y), T0, T1, T2)) {
						setPixel(x, y, activeColor);
					}
				}
			}
			for (int y = T1.y(); y <= T2.y(); y++) {
				int x1 = static_cast<int>((y - P.y()) * (T2.x() - P.x()) / (T2.y() - P.y()) + P.x());
				int x2 = static_cast<int>((y - T1.y()) * (T2.x() - T1.x()) / (T2.y() - T1.y()) + T1.x());
				
				for (int x = x1; x <= x2; x++) {
					QColor activeColor;
					if (interpolType == 0) {
						double length1 = sqrt(pow(T0.x() - x, 2) + pow(T0.y() - y, 2));
						double length2 = sqrt(pow(T1.x() - x, 2) + pow(T1.y() - y, 2));
						double length3 = sqrt(pow(T2.x() - x, 2) + pow(T2.y() - y, 2));

						if (length1 <= length2 && length1 <= length3) {
							activeColor = color;
						}
						else if (length2 < length1 && length2 <= length3) {
							activeColor = color1;
						}
						else {
							activeColor = color2;
						}
					}
					else {
						QPoint P(x, y);
						activeColor = interpolateColorBarycentric(P, T0, T1, T2, color, color1, color2);
					}

					if (isPointInsideTriangle(QPoint(x, y), T0, T1, T2)) {
						setPixel(x, y, activeColor);
					}
				}
			}
		}
	}
}

bool ViewerWidget::isPointInsideTriangle(const QPoint& P, const QPoint& A, const QPoint& B, const QPoint& C) {
	int as_x = P.x() - A.x();
	int as_y = P.y() - A.y();

	bool s_ab = (B.x() - A.x()) * as_y - (B.y() - A.y()) * as_x > 0;

	if ((C.x() - A.x()) * as_y - (C.y() - A.y()) * as_x > 0 == s_ab) return false;

	if ((C.x() - B.x()) * (P.y() - B.y()) - (C.y() - B.y()) * (P.x() - B.x()) > 0 != s_ab) return false;

	return true;
}

void ViewerWidget::fillFlatBottomTriangle(const QPoint& T0, const QPoint& T1, const QPoint& T2, const QColor& color, const QColor& color1, const QColor& color2, int interpolType) {
	double W1 = (T1.x() - T0.x()) != 0 ? (T1.y() - T0.y()) / static_cast<double>(T1.x() - T0.x()) : std::numeric_limits<double>::max();
	double W2 = (T2.x() - T0.x()) != 0 ? (T2.y() - T0.y()) / static_cast<double>(T2.x() - T0.x()) : std::numeric_limits<double>::max();

	for (int y = T0.y(); y <= T1.y(); y++) {
		int startX = T0.x() + static_cast<int>((y - T0.y()) * W1);
		int endX = T0.x() + static_cast<int>((y - T0.y()) * W2);

		if (startX > endX) std::swap(startX, endX);

		for (int x = startX; x <= endX; x++) {
			//	<< Nearest neighbor algorithm >>
			if (interpolType == 0) {
				double length1 = sqrt(pow(T0.x() - x, 2) + pow(T0.y() - y, 2));
				double length2 = sqrt(pow(T1.x() - x, 2) + pow(T1.y() - y, 2));
				double length3 = sqrt(pow(T2.x() - x, 2) + pow(T2.y() - y, 2));

				QColor activeColor;
				if (length1 <= length2 && length1 <= length3) {
					activeColor = color;
				}
				else if (length2 < length1 && length2 <= length3) {
					activeColor = color1;
				}
				else {
					activeColor = color2;
				}

				if (isPointInsideTriangle(QPoint(x, y), T0, T1, T2)) {
					setPixel(x, y, activeColor);
				}
			}
			// << Barycentric interpolation >>
			else {
				QPoint P(x, y);
				if (isPointInsideTriangle(P, T0, T1, T2)) {
					QColor activeColor = interpolateColorBarycentric(P, T0, T1, T2, color, color1, color2);
					setPixel(x, y, activeColor);
				}
			}
		}
	}
}

void ViewerWidget::fillFlatTopTriangle(const QPoint& T0, const QPoint& T1, const QPoint& T2, const QColor& color, const QColor& color1, const QColor& color2, int interpolType) {
	double W1 = (T2.x() - T0.x()) != 0 ? (T2.y() - T0.y()) / static_cast<double>(T2.x() - T0.x()) : std::numeric_limits<double>::max();
	double W2 = (T2.x() - T1.x()) != 0 ? (T2.y() - T1.y()) / static_cast<double>(T2.x() - T1.x()) : std::numeric_limits<double>::max();

	for (int y = T2.y(); y >= T0.y(); y--) {
		int startX = T2.x() - static_cast<int>((T2.y() - y) * W1);
		int endX = T2.x() - static_cast<int>((T2.y() - y) * W2);

		if (startX > endX) std::swap(startX, endX);

		for (int x = startX; x <= endX; x++) {
			if (interpolType == 0) {
				// << Nearest neighbor algorithm >>
				double length1 = sqrt(pow(T0.x() - x, 2) + pow(T0.y() - y, 2));
				double length2 = sqrt(pow(T1.x() - x, 2) + pow(T1.y() - y, 2));
				double length3 = sqrt(pow(T2.x() - x, 2) + pow(T2.y() - y, 2));

				QColor activeColor;
				if (length1 <= length2 && length1 <= length3) {
					activeColor = color;
				}
				else if (length2 < length1 && length2 <= length3) {
					activeColor = color1;
				}
				else {
					activeColor = color2;
				}

				if (isPointInsideTriangle(QPoint(x, y), T0, T1, T2)) {
					setPixel(x, y, activeColor);
				}
			}
			else {
				// << Barycentric interpolation >>
				QPoint P(x, y);
				if (isPointInsideTriangle(P, T0, T1, T2)) {
					QColor activeColor = interpolateColorBarycentric(P, T0, T1, T2, color, color1, color2);
					setPixel(x, y, activeColor);
				}
			}
		}
	}
}

QColor ViewerWidget::interpolateColorBarycentric(const QPoint& P, const QPoint& T0, const QPoint& T1, const QPoint& T2, const QColor& color0, const QColor& color1, const QColor& color2) {
	double areaTotal = (T1.y() - T2.y()) * (T0.x() - T2.x()) + (T2.x() - T1.x()) * (T0.y() - T2.y());

	double lambda0 = ((T1.y() - T2.y()) * (P.x() - T2.x()) + (T2.x() - T1.x()) * (P.y() - T2.y())) / areaTotal;
	double lambda1 = ((T2.y() - T0.y()) * (P.x() - T2.x()) + (T0.x() - T2.x()) * (P.y() - T2.y())) / areaTotal;
	double lambda2 = 1 - lambda0 - lambda1;

	lambda0 = std::max(0.0, std::min(1.0, lambda0));
	lambda1 = std::max(0.0, std::min(1.0, lambda1));
	lambda2 = std::max(0.0, std::min(1.0, lambda2));

	int r = static_cast<int>(lambda0 * color0.red() + lambda1 * color1.red() + lambda2 * color2.red());
	int g = static_cast<int>(lambda0 * color0.green() + lambda1 * color1.green() + lambda2 * color2.green());
	int b = static_cast<int>(lambda0 * color0.blue() + lambda1 * color1.blue() + lambda2 * color2.blue());

	r = std::clamp(r, 0, 255);
	g = std::clamp(g, 0, 255);
	b = std::clamp(b, 0, 255);

	return QColor(r, g, b);
}

//-----------------------------------------
//		*** Curve functions ***
//-----------------------------------------

void ViewerWidget::drawCurve(const QColor& color, const QColor& color2, int curveType, int algType) {
	if (curveType == 0) {
		// << Hermit cubic >>

		if (curvePoints.size() < 2) {
			QMessageBox::warning(this, "Nedostatocny pocet bodov", "Nemozno nakreslit krivku s menej ako dvomi riadiacimi bodmi.", QMessageBox::Ok);
			return;
		}
		
		drawTangentVectors(color2, algType);
		
		auto hermiteBasisF0 = [](double t) { return 2 * t * t * t - 3 * t * t + 1; };
		auto hermiteBasisF1 = [](double t) { return t * t * t - 2 * t * t + t; };
		auto hermiteBasisF2 = [](double t) { return -2 * t * t * t + 3 * t * t; };
		auto hermiteBasisF3 = [](double t) { return t * t * t - t * t; };

		float deltaT = 0.01f;
		for (int i = 1; i < curvePoints.size(); i++) {
			QPoint Q0 = curvePoints[i - 1];

			for (float t = deltaT; t < 1.0f; t += deltaT) {
				QPoint Q1(
					curvePoints[i - 1].x() * hermiteBasisF0(t) + curvePoints[i].x() * hermiteBasisF2(t) + tangents[i - 1].x() * hermiteBasisF1(t) + tangents[i].x() * hermiteBasisF3(t),
					curvePoints[i - 1].y() * hermiteBasisF0(t) + curvePoints[i].y() * hermiteBasisF2(t) + tangents[i - 1].y() * hermiteBasisF1(t) + tangents[i].y() * hermiteBasisF3(t)
				);

				drawLine(Q0, Q1, color, algType);
				Q0 = Q1;
			}

			drawLine(Q0, curvePoints[i], color, algType);
		}
	}
	else if (curveType == 1) {
		// << Bezier curve >>

		if (curvePoints.size() < 2) {
			QMessageBox::warning(this, "Nedostatocny pocet bodov", "Nemozno nakreslit krivku s menej ako dvomi riadiacimi bodmi.", QMessageBox::Ok);
			return;
		}

		float deltaT = 0.01f;
		QPoint Q0 = curvePoints[0];

		for (float t = deltaT; t <= 1; t += deltaT) {
			QVector<QPoint> tempPoints = curvePoints;

			for (int i = 1; i < tempPoints.size(); ++i) {
				for (int j = 0; j < tempPoints.size() - i; ++j) {
					tempPoints[j] = tempPoints[j] * (1 - t) + tempPoints[j + 1] * t;
				}
			}

			drawLine(Q0, tempPoints[0], color, algType);
			Q0 = tempPoints[0];
		}
		if (deltaT * floor(1 / deltaT) < 1) {
			drawLine(Q0, curvePoints.last(), color, algType);
		}
	}
	else if (curveType == 2) {
		// << Coons cubic B-spline >>

		if (curvePoints.size() < 4) {
			QMessageBox::warning(this, "Nedostatocny pocet bodov", "Nemozno nakreslit Coonsovu krivku s menej ako styrmi riadiacimi bodmi.", QMessageBox::Ok);
			return;
		}

		float deltaT = 0.01f;

		auto B0 = [](float t) { return pow(1 - t, 3) / 6; };
		auto B1 = [](float t) { return (3 * pow(t, 3) - 6 * pow(t, 2) + 4) / 6; };
		auto B2 = [](float t) { return (-3 * pow(t, 3) + 3 * pow(t, 2) + 3 * t + 1) / 6; };
		auto B3 = [](float t) { return pow(t, 3) / 6; };

		for (int i = 3; i < curvePoints.size(); i++) {
			QPoint Q0 = curvePoints[i - 3] * B0(0) + curvePoints[i - 2] * B1(0) +
				curvePoints[i - 1] * B2(0) + curvePoints[i] * B3(0);

			for (float t = deltaT; t <= 1.0f; t += deltaT) {
				QPoint Q1 = curvePoints[i - 3] * B0(t) + curvePoints[i - 2] * B1(t) +
					curvePoints[i - 1] * B2(t) + curvePoints[i] * B3(t);

				drawLine(Q0, Q1, color, algType);
				Q0 = Q1;
			}
		}
	}
}

void ViewerWidget::drawTangentVectors(const QColor& color, int algType) {
	painter->setPen(QPen(color));

	const int tangentLength = 70;

	for (int i = 0; i < tangents.size(); i++) {
		QPoint currentPoint = curvePoints[i];
		QPoint tangentVector = tangents[i];

		double length = std::sqrt(tangentVector.x() * tangentVector.x() + tangentVector.y() * tangentVector.y());
		if (length == 0)
			continue;

		double unitX = tangentVector.x() / length;
		double unitY = tangentVector.y() / length;

		QPoint tangentEnd(currentPoint.x() + static_cast<int>(unitX * tangentLength),
			currentPoint.y() + static_cast<int>(unitY * tangentLength));

		setPixel(tangentEnd.x(), tangentEnd.y(), Qt::green, true);
		drawLine(currentPoint, tangentEnd, color, algType);
	}
}

void ViewerWidget::calculateTangents() {
	if (curvePoints.size() < 2) 
		return;

	tangents.clear();

	QPoint firstTangent = (curvePoints[1] - curvePoints[0]);
	tangents.push_back(firstTangent);

	for (int i = 1; i < curvePoints.size() - 1; i++) {
		QPoint prevVector = curvePoints[i] - curvePoints[i - 1];
		QPoint nextVector = curvePoints[i + 1] - curvePoints[i];
		QPoint tangent = (prevVector + nextVector) / 2;
		tangents.push_back(tangent);
	}

	QPoint lastTangent = (curvePoints.last() - curvePoints[curvePoints.size() - 2]);
	tangents.push_back(lastTangent);

}

void ViewerWidget::moveCurve(const QColor& color, const QColor& color2, const QPoint& offset, int algType, int curveType, bool isCurve) {
	if (!curvePoints.isEmpty()) {
		for (QPoint& point : curvePoints) {
			point += offset;
		}
		updateCurve(color, color2, offset, algType, curveType, isCurve);
	}
}

void ViewerWidget::updateCurve(const QColor& color, const QColor& color2, const QPoint& offset, int algType, int curveType, bool isCurve) {
	clear();
	for (QPoint& point : curvePoints) {
		setPixel(point.x(), point.y(), color2, isCurve);
	}
	drawCurve(color, color2, curveType, algType);
	update();
}

void ViewerWidget::scaleCurve(const QColor& color, const QColor& color2, int algType, double scaleX, double scaleY, int curveType, bool isCurve) {
	for (int i = 0; i < curvePoints.size(); i++) {
		int newX = static_cast<int>(curvePoints[i].x() * scaleX);
		int newY = static_cast<int>(curvePoints[i].y() * scaleY);
		curvePoints[i] = QPoint(newX, newY);
	}

	clear();
	for (QPoint& point : curvePoints) {
		setPixel(point.x(), point.y(), color2, isCurve);
	}
	drawCurve(color, color2, curveType, algType);
	update();
}

void ViewerWidget::turnCurve(const QColor& color, const QColor& color2, int algType, int angle, int curveType, bool isCurve) {
	double radians = qDegreesToRadians(static_cast<double>(angle));

	QPoint pivot = calculateCurveCenter();
	for (QPoint& point : curvePoints) {
		int x = point.x() - pivot.x();
		int y = point.y() - pivot.y();

		int rotatedX = static_cast<int>(x * cos(radians) - y * sin(radians)) + pivot.x();
		int rotatedY = static_cast<int>(x * sin(radians) + y * cos(radians)) + pivot.y();

		point = QPoint(rotatedX, rotatedY);
	}

	clear();
	for (QPoint& point : curvePoints) {
		setPixel(point.x(), point.y(), color2, isCurve);
	}
	drawCurve(color, color2, curveType, algType);
	update();
}

QPoint ViewerWidget::calculateCurveCenter() {
	int minX = std::numeric_limits<int>::max();
	int maxX = std::numeric_limits<int>::min();
	int minY = std::numeric_limits<int>::max();
	int maxY = std::numeric_limits<int>::min();

	for (const QPoint& point : curvePoints) {
		minX = std::min(point.x(), minX);
		maxX = std::max(point.x(), maxX);
		minY = std::min(point.y(), minY);
		maxY = std::max(point.y(), maxY);
	}

	return QPoint((minX + maxX) / 2, (minY + maxY) / 2);
}

QPoint ViewerWidget::calculateTheNearestPoint(QPoint position) {
	if (curvePoints.isEmpty() || tangents.isEmpty())
		return QPoint();

	double minDistance = std::numeric_limits<double>::max();
	QPoint nearestPoint;

	for (int i = 0; i < curvePoints.size(); i++) {
		QPoint startPoint = curvePoints[i];
		QPoint endPoint = startPoint + tangents[i];

		double distanceToStart = std::sqrt(std::pow(startPoint.x() - position.x(), 2) + std::pow(startPoint.y() - position.y(), 2));
		double distanceToEnd = std::sqrt(std::pow(endPoint.x() - position.x(), 2) + std::pow(endPoint.y() - position.y(), 2));

		if (distanceToStart < minDistance) {
			minDistance = distanceToStart;
			nearestPoint = startPoint;
		}
		if (distanceToEnd < minDistance) {
			minDistance = distanceToEnd;
			nearestPoint = endPoint;
		}
	}

	return nearestPoint;
}

void ViewerWidget::moveTheNearestPoint(const QColor& color, const QColor& color2, const QPoint& offset, int curveType, int algType) {
	if (curvePoints.isEmpty() || tangents.isEmpty())
		return;
	if (moveStart.isNull())
		return;

	bool isControlPoint = false;
	int index = -1;
	double minDistance = std::numeric_limits<double>::max();

	for (int i = 0; i < curvePoints.size(); i++) {
		double distanceToStart = std::hypot(curvePoints[i].x() - moveStart.x(), curvePoints[i].y() - moveStart.y());
		QPoint endPoint = curvePoints[i] + tangents[i];
		double distanceToEnd = std::hypot(endPoint.x() - moveStart.x(), endPoint.y() - moveStart.y());

		if (distanceToStart < minDistance) {
			minDistance = distanceToStart;
			index = i;
			isControlPoint = true;
		}
		if (distanceToEnd < minDistance) {
			minDistance = distanceToEnd;
			index = i;
			isControlPoint = false;
		}
	}

	if (index != -1) {
		if (isControlPoint) {
			curvePoints[index] += offset;
		}
		else {
			QPoint newEndPoint = (curvePoints[index] + tangents[index]) + offset;
			tangents[index] = newEndPoint - curvePoints[index];
		}
	}

	clear();
	for (QPoint& point : curvePoints) {
		setPixel(point.x(), point.y(), color2, true);
	}
	drawCurve(color, color2, curveType, algType);
	update();
}

//****************************************************************
//-----------------------------------------
//		*** <<< Projekt 2 >>> ***
//-----------------------------------------
//****************************************************************

void ViewerWidget::saveVTKCube(int sideLength) {
	double half = sideLength / 2.0;

	std::vector<Vertex> vertices = {
		{-half, -half, -half}, {half, -half, -half},
		{half, half, -half}, {-half, half, -half},
		{-half, -half, half}, {half, -half, half},
		{half, half, half}, {-half, half, half}
	};

	std::vector<std::vector<int>> faces = {
		{0, 1, 2}, {0, 2, 3},
		{7, 6, 5}, {7, 5, 4},
		{0, 3, 7}, {0, 7, 4},
		{1, 2, 6}, {1, 6, 5},
		{0, 4, 5}, {0, 5, 1},
		{3, 2, 6}, {3, 6, 7}
	};

	QString fileName = QFileDialog::getSaveFileName(nullptr, QObject::tr("Save cube"), "", QObject::tr("VTK Files (*.vtk)"));
	if (fileName.isEmpty())
		return;

	QFile file(fileName);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
		QMessageBox::warning(this, "Error opening file", "File could not be opened.");
		return;
	}

	QTextStream out(&file);

	out << "# vtk DataFile Version 3.0\n";
	out << "Cube visualization\n";
	out << "ASCII\n";
	out << "DATASET POLYDATA\n";

	out << "POINTS " << vertices.size() << " float\n";
	for (const auto& vertex : vertices) {
		out << vertex.x << " " << vertex.y << " " << vertex.z << "\n";
	}

	int numberOfTriangles = faces.size();
	int totalIndices = numberOfTriangles * 4;
	out << "POLYGONS " << numberOfTriangles << " " << totalIndices << "\n";
	for (const auto& face : faces) {
		out << "3";
		for (int index : face) {
			out << " " << index;
		}
		out << "\n";
	}

	file.close();
}

void ViewerWidget::saveSphereVTK(int parallels, int meridians, double radius) {
	struct Point {
		double x, y, z;
	};

	struct Triangle {
		int a, b, c;
	};

	std::vector<Point> points;
	std::vector<Triangle> triangles;

	for (int p = 0; p <= parallels; p++) {
		double phi = M_PI * double(p) / double(parallels);
		for (int m = 0; m <= meridians; m++) {
			double theta = 2.0 * M_PI * double(m) / double(meridians);
			points.push_back(Point{
				radius * sin(phi) * cos(theta),
				radius * sin(phi) * sin(theta),
				radius * cos(phi)
				});
		}
	}

	for (int p = 0; p < parallels; p++) {
		for (int m = 0; m < meridians; m++) {
			int current = p * (meridians + 1) + m;
			int next = current + meridians + 1;

			triangles.push_back(Triangle{ current, next, current + 1 });

			if (p != (parallels - 1)) {
				triangles.push_back(Triangle{ current + 1, next, next + 1 });
			}
		}
	}

	QString fileName = QFileDialog::getSaveFileName(nullptr, QObject::tr("Ulozit sferu"), "", QObject::tr("VTK Files (*.vtk)"));
	if (fileName.isEmpty())
		return;

	QFile vtkFile(fileName);
	if (!vtkFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
		QMessageBox::warning(this, "Chyba pri otvarani suboru", "Subor sa nepodarilo otvorit.");
		return;
	}

	QTextStream out(&vtkFile);

	out << "# vtk DataFile Version 3.0\n";
	out << "Sphere data\n";
	out << "ASCII\n";
	out << "DATASET POLYDATA\n";

	out << "POINTS " << points.size() << " float\n";
	for (const auto& point : points) {
		out << point.x << " " << point.y << " " << point.z << "\n";
	}

	out << "POLYGONS " << triangles.size() << " " << triangles.size() * 4 << "\n";
	for (const auto& triangle : triangles) {
		out << "3 " << triangle.a << " " << triangle.b << " " << triangle.c << "\n";
	}

	vtkFile.close();
}

void ViewerWidget::loadCubeVTK() {
	QString fileName = QFileDialog::getOpenFileName(this, tr("Open Cube File"), "", tr("VTK Files (*.vtk)"));
	if (fileName.isEmpty()) {
		return;
	}

	QFile file(fileName);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
		QMessageBox::warning(this, tr("Error"), tr("Cannot open file"));
		return;
	}

	for (auto* v : cubeVertices) delete v;
	for (auto* e : cubeEdges) delete e;
	for (auto* f : cubeFaces) delete f;
	cubeVertices.clear();
	cubeEdges.clear();
	cubeFaces.clear();

	QTextStream in(&file);
	QString line;

	bool readingPoints = false;
	bool readingPolygons = false;

	while (!in.atEnd()) {
		line = in.readLine().trimmed();

		if (line.isEmpty() || line.startsWith("#")) {
			continue;
		}

		if (line.startsWith("POINTS")) {
			readingPoints = true;
			readingPolygons = false;
			continue;
		}
		else if (line.startsWith("POLYGONS")) {
			readingPoints = false;
			readingPolygons = true;
			continue;
		}

		if (readingPoints) {
			QStringList coords = line.split(" ", Qt::SkipEmptyParts);
			double x = coords[0].toDouble();
			double y = coords[1].toDouble();
			double z = coords[2].toDouble();
			Vertex* vertex = new Vertex(x, y, z);
			cubeVertices.append(vertex);
		}

		if (readingPolygons) {
			QStringList indices = line.split(" ", Qt::SkipEmptyParts);
			indices.removeFirst();

			QVector<H_edge*> faceEdges;
			H_edge* firstEdge = nullptr;
			H_edge* prevEdge = nullptr;

			for (int i = 0; i < indices.size(); ++i) {
				Vertex* currentVertex = cubeVertices[indices[i].toInt()];
				H_edge* currentEdge = new H_edge(currentVertex, nullptr);

				if (prevEdge != nullptr) {
					prevEdge->edge_next = currentEdge;
					currentEdge->edge_prev = prevEdge;
				}
				prevEdge = currentEdge;
				faceEdges.append(currentEdge);
				cubeEdges.append(currentEdge);

				if (i == 0) {
					firstEdge = currentEdge;
				}
			}

			prevEdge->edge_next = firstEdge;
			firstEdge->edge_prev = prevEdge;

			Face* newFace = new Face(firstEdge);
			foreach(H_edge * edge, faceEdges) {
				edge->face = newFace;
			}
			cubeFaces.append(newFace);
		}
	}

	file.close();

	for (int i = 0; i < cubeEdges.size(); i++) {
		for (int j = i + 1; j < cubeEdges.size(); j++) {
			if (cubeEdges[i]->vert_origin == cubeEdges[j]->edge_next->vert_origin &&
				cubeEdges[i]->edge_next->vert_origin == cubeEdges[j]->vert_origin) {
				cubeEdges[i]->pair = cubeEdges[j];
				cubeEdges[j]->pair = cubeEdges[i];
			}
		}
	}
}

void ViewerWidget::loadSphereVTK() {
	QString fileName = QFileDialog::getOpenFileName(this, tr("Open Sphere File"), "", tr("VTK Files (*.vtk)"));
	if (fileName.isEmpty()) {
		return;
	}

	QFile file(fileName);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
		QMessageBox::warning(this, tr("Error"), tr("Cannot open file"));
		return;
	}

	for (auto* v : sphereVertices) delete v;
	for (auto* e : sphereEdges) delete e;
	for (auto* f : sphereFaces) delete f;
	sphereVertices.clear();
	sphereEdges.clear();
	sphereFaces.clear();

	QTextStream in(&file);
	QHash<QPair<int, int>, H_edge*> edgeMap;

	QString line;
	while (!in.atEnd()) {
		line = in.readLine();
		if (line.contains("POINTS")) {
			int numPoints = line.split(' ')[1].toInt();
			for (int i = 0; i < numPoints; i++) {
				line = in.readLine();
				QStringList coords = line.split(' ');
				double x = coords[0].toDouble();
				double y = coords[1].toDouble();
				double z = coords[2].toDouble();
				sphereVertices.append(new Vertex(x, y, z));
			}
		}
		else if (line.contains("POLYGONS")) {
			int numPolygons = line.split(' ')[1].toInt();
			for (int i = 0; i < numPolygons; i++) {
				line = in.readLine();
				QStringList indices = line.split(' ');
				int numVertices = indices[0].toInt();

				QVector<H_edge*> polyEdges;
				for (int j = 0; j < numVertices; j++) {
					int idx = indices[j + 1].toInt();
					H_edge* edge = new H_edge(sphereVertices[idx], nullptr);
					sphereEdges.append(edge);
					polyEdges.append(edge);
				}

				for (int j = 0; j < numVertices; j++) {
					polyEdges[j]->edge_next = polyEdges[(j + 1) % numVertices];
					polyEdges[(j + 1) % numVertices]->edge_prev = polyEdges[j];
					int startIndex = sphereVertices.indexOf(polyEdges[j]->vert_origin);
					int endIndex = sphereVertices.indexOf(polyEdges[(j + 1) % numVertices]->vert_origin);
					edgeMap.insert(QPair<int, int>(startIndex, endIndex), polyEdges[j]);
				}

				Face* face = new Face(polyEdges[0]);
				sphereFaces.append(face);
				for (H_edge* edge : polyEdges) {
					edge->face = face;
				}
			}
		}
	}

	for (H_edge* edge : sphereEdges) {
		int startIdx = sphereVertices.indexOf(edge->vert_origin);
		int endIdx = sphereVertices.indexOf(edge->edge_next->vert_origin);
		H_edge* pairEdge = edgeMap.value(QPair<int, int>(endIdx, startIdx));
		if (pairEdge) {
			edge->pair = pairEdge;
			pairEdge->pair = edge;
		}
	}

	file.close();
}


void ViewerWidget::adjustProjection(double theta, double phi, bool objectLoaded, int objectType, int algType, int projectionType, double distance, int cameraDistance, Lighting prime) {
	if (!objectLoaded) {
		return;
	}

	int thetaDirection = (theta < prevTheta) ? -1 : 1;
	int phiDirection = (phi < prevPhi) ? -1 : 1;

	QMatrix4x4 rotationMatrix;

	if (theta != prevTheta) {
		rotationMatrix.rotate(qRadiansToDegrees(theta) * thetaDirection, 1, 0, 0);
	}
	else if (phi != prevPhi) {
		rotationMatrix.rotate(qRadiansToDegrees(phi) * phiDirection, 0, 1, 0);
	}

	prevTheta = theta;
	prevPhi = phi;

	n = QVector3D(sin(theta) * sin(phi), sin(theta) * cos(phi), cos(theta));
	u = QVector3D(sin(theta + M_PI_2) * sin(phi), sin(theta + M_PI_2) * cos(phi), cos(theta + M_PI_2));
	v = QVector3D::crossProduct(n, u);

	QVector<QVector3D> originalVertices;
	QVector<Vertex*>* vertices = nullptr;

	if (objectType == isCube || objectType == isCubeWireFrame) {
		vertices = &cubeVertices;
	}
	else if (objectType == isSphere || objectType == isSphereWireFrame) {
		vertices = &sphereVertices;
	}

	QVector3D drawingAreaCenter(width() / 2.0f, height() / 2.0f, 0.0f);

	if (vertices) {
		originalVertices.reserve(vertices->size());
		QVector3D center = computeCenter(*vertices);

		for (Vertex* vertex : *vertices) {
			QVector3D pos(vertex->x, vertex->y, vertex->z);
			pos -= center;
			pos = rotationMatrix * pos;
			pos += center;
			originalVertices.append(pos);
		}

		for (int i = 0; i < vertices->size(); i++) {
			Vertex* vertex = (*vertices)[i];
			vertex->x = originalVertices[i].x();
			vertex->y = originalVertices[i].y();
			vertex->z = originalVertices[i].z();
		}
	}
}

double ViewerWidget::mapSliderValueToDegrees(int value, int min, int max) {
	double range = max - min;
	double angleRadians = (value - min) / range * 2 * M_PI;
	double angleDegrees = angleRadians * M_PI / 50;
	return angleDegrees;
}

QVector3D ViewerWidget::computeCenter(const QVector<Vertex*>& vertices) {
	QVector3D minPoint(FLT_MAX, FLT_MAX, FLT_MAX);
	QVector3D maxPoint(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	for (Vertex* vertex : vertices) {
		minPoint.setX(qMin(minPoint.x(), vertex->x));
		minPoint.setY(qMin(minPoint.y(), vertex->y));
		minPoint.setZ(qMin(minPoint.z(), vertex->z));
		maxPoint.setX(qMax(maxPoint.x(), vertex->x));
		maxPoint.setY(qMax(maxPoint.y(), vertex->y));
		maxPoint.setZ(qMax(maxPoint.z(), vertex->z));
	}
	return (minPoint + maxPoint) * 0.5;
}


void ViewerWidget::renderParallelProjection(double theta, double phi, bool objectIsLoaded, int algType, int objectType, int projectionType, double distance, int cameraDistance, Lighting prime, int shadingType, int h) {
	clear();
	QVector<QPoint> projectedVertices;
	QVector<QVector<QColor>> F;
	Z = QVector<QVector<double>>(width(), QVector<double>(height(), std::numeric_limits<double>::lowest()));

	adjustProjection(theta, phi, objectIsLoaded, objectType, algType, projectionType, distance, cameraDistance, prime);

	QSet<QPair<int, int>> drawnEdges;

	switch (objectType) {
	case isCube:
		for (Face* f : cubeFaces) {
			if (!f || !f->edge || !f->edge->vert_origin || !f->edge->edge_next || !f->edge->edge_prev) {
				continue;
			}

			QVector3D p1o(f->edge->vert_origin->x, f->edge->vert_origin->y, f->edge->vert_origin->z);
			QVector3D p2o(f->edge->edge_next->vert_origin->x, f->edge->edge_next->vert_origin->y, f->edge->edge_next->vert_origin->z);
			QVector3D p3o(f->edge->edge_prev->vert_origin->x, f->edge->edge_prev->vert_origin->y, f->edge->edge_prev->vert_origin->z);

			QVector3D p1(QVector3D::dotProduct(p1o, v), QVector3D::dotProduct(p1o, u), QVector3D::dotProduct(p1o, n));
			QVector3D p2(QVector3D::dotProduct(p2o, v), QVector3D::dotProduct(p2o, u), QVector3D::dotProduct(p2o, n));
			QVector3D p3(QVector3D::dotProduct(p3o, v), QVector3D::dotProduct(p3o, u), QVector3D::dotProduct(p3o, n));

			QVector<QPoint> T = {
				QPoint(static_cast<int>(p1.x()) + width() / 2, static_cast<int>(p1.y()) + height() / 2),
				QPoint(static_cast<int>(p2.x()) + width() / 2, static_cast<int>(p2.y()) + height() / 2),
				QPoint(static_cast<int>(p3.x()) + width() / 2, static_cast<int>(p3.y()) + height() / 2)
			};

			rasterizeForFilling(prime, cameraDistance, F, T, { p1, p2, p3 }, shadingType, h);
		}

		break;

	case isCubeWireFrame:
		for (auto* edge : cubeEdges) {
			if (!edge || !edge->vert_origin || !edge->edge_next || !edge->edge_next->vert_origin) continue;

			int startIdx = cubeVertices.indexOf(edge->vert_origin);
			int endIdx = cubeVertices.indexOf(edge->edge_next->vert_origin);

			if (!drawnEdges.contains(qMakePair(endIdx, startIdx)) && !drawnEdges.contains(qMakePair(startIdx, endIdx))) {
				drawnEdges.insert(qMakePair(startIdx, endIdx));

				Vertex* startVertex = cubeVertices[startIdx];
				Vertex* endVertex = cubeVertices[endIdx];

				QPoint startPoint(startVertex->x + width() / 2, startVertex->y + height() / 2);
				QPoint endPoint(endVertex->x + width() / 2, endVertex->y + height() / 2);

				drawLine(startPoint, endPoint, Qt::black, algType);
			}
		}

		break;

	case isSphere:
		for (Face* f : sphereFaces) {
			if (!f || !f->edge || !f->edge->vert_origin || !f->edge->edge_next || !f->edge->edge_prev) {
				continue;
			}

			QVector3D p1o(f->edge->vert_origin->x, f->edge->vert_origin->y, f->edge->vert_origin->z);
			QVector3D p2o(f->edge->edge_next->vert_origin->x, f->edge->edge_next->vert_origin->y, f->edge->edge_next->vert_origin->z);
			QVector3D p3o(f->edge->edge_prev->vert_origin->x, f->edge->edge_prev->vert_origin->y, f->edge->edge_prev->vert_origin->z);

			QVector3D p1(QVector3D::dotProduct(p1o, v), QVector3D::dotProduct(p1o, u), QVector3D::dotProduct(p1o, n));
			QVector3D p2(QVector3D::dotProduct(p2o, v), QVector3D::dotProduct(p2o, u), QVector3D::dotProduct(p2o, n));
			QVector3D p3(QVector3D::dotProduct(p3o, v), QVector3D::dotProduct(p3o, u), QVector3D::dotProduct(p3o, n));

			QVector<QPoint> T = {
				QPoint(static_cast<int>(p1.x()) + width() / 2, static_cast<int>(p1.y()) + height() / 2),
				QPoint(static_cast<int>(p2.x()) + width() / 2, static_cast<int>(p2.y()) + height() / 2),
				QPoint(static_cast<int>(p3.x()) + width() / 2, static_cast<int>(p3.y()) + height() / 2)
			};

			rasterizeForFilling(prime, cameraDistance, F, T, { p1, p2, p3 }, shadingType, h);
		}
		break;

	case isSphereWireFrame:
		QVector3D sphereCenter = computeCenter(sphereVertices);

		QVector3D translation(
			width() / 2.0 - sphereCenter.x(),
			height() / 2.0 - sphereCenter.y(),
			0
		);

		for (H_edge* edge : sphereEdges) {
			if (edge) {
				Vertex* v1 = edge->vert_origin;
				Vertex* v2 = edge->edge_next->vert_origin;

				QPoint start(v1->x + translation.x(), v1->y + translation.y());
				QPoint end(v2->x + translation.x(), v2->y + translation.y());

				drawLine(start, end, Qt::black, algType);
			}
		}
		break;
	}

	update();
}

void ViewerWidget::renderPerspectiveProjection(double theta, double phi, bool objectIsLoaded, int algType, int objectType, int projectionType, double distance, int cameraDistance, Lighting prime, int shadingType, int h) {
	clear();
	QVector<QVector<QColor>> F;
	Z = QVector<QVector<double>>(width(), QVector<double>(height(), std::numeric_limits<double>::lowest()));

	adjustProjection(theta, phi, objectIsLoaded, objectType, algType, projectionType, distance, cameraDistance, prime);

	switch (objectType) {
	case isCubeWireFrame:
		for (H_edge* edge : cubeEdges) {
			if (edge->vert_origin && edge->edge_next && edge->edge_next->vert_origin) {
				QPoint point1 = projectPoint(edge->vert_origin, distance);
				QPoint point2 = projectPoint(edge->edge_next->vert_origin, distance);

				if (!(point1.isNull() || point2.isNull())) {
					drawLine(point1, point2, Qt::black, algType);
				}
			}
		}
		break;

	case isCube:
		for (Face* f : cubeFaces) {
			if (!f || !f->edge || !f->edge->vert_origin || !f->edge->edge_next || !f->edge->edge_prev) {
				continue;
			}

			QVector3D p1o(f->edge->vert_origin->x, f->edge->vert_origin->y, f->edge->vert_origin->z);
			QVector3D p2o(f->edge->edge_next->vert_origin->x, f->edge->edge_next->vert_origin->y, f->edge->edge_next->vert_origin->z);
			QVector3D p3o(f->edge->edge_prev->vert_origin->x, f->edge->edge_prev->vert_origin->y, f->edge->edge_prev->vert_origin->z);

			QVector3D p1(QVector3D::dotProduct(p1o, v), QVector3D::dotProduct(p1o, u), QVector3D::dotProduct(p1o, n));
			QVector3D p2(QVector3D::dotProduct(p2o, v), QVector3D::dotProduct(p2o, u), QVector3D::dotProduct(p2o, n));
			QVector3D p3(QVector3D::dotProduct(p3o, v), QVector3D::dotProduct(p3o, u), QVector3D::dotProduct(p3o, n));

			QVector<QPoint> T = { QPoint(distance * p1.x() / (distance - p1.z()) + width() / 2, distance * p1.y() / (distance - p1.z()) + height() / 2),
				QPoint(distance * p2.x() / (distance - p2.z()) + width() / 2, distance * p2.y() / (distance - p2.z()) + height() / 2),
				QPoint(distance * p3.x() / (distance - p3.z()) + width() / 2, distance * p3.y() / (distance - p3.z()) + height() / 2) };

			rasterizeForFilling(prime, cameraDistance, F, T, { p1, p2, p3 }, shadingType, h);
		}
		break;

	case isSphereWireFrame:
		for (H_edge* edge : sphereEdges) {
			if (edge->vert_origin && edge->edge_next && edge->edge_next->vert_origin) {
				QPoint point1 = projectPoint(edge->vert_origin, distance);
				QPoint point2 = projectPoint(edge->edge_next->vert_origin, distance);

				if (!(point1.isNull() || point2.isNull())) {
					drawLine(point1, point2, Qt::black, algType);
				}
			}
		}
		break;

	case isSphere:
		for (Face* f : sphereFaces) {
			if (!f || !f->edge || !f->edge->vert_origin || !f->edge->edge_next || !f->edge->edge_prev) {
				continue;
			}

			QVector3D p1o(f->edge->vert_origin->x, f->edge->vert_origin->y, f->edge->vert_origin->z);
			QVector3D p2o(f->edge->edge_next->vert_origin->x, f->edge->edge_next->vert_origin->y, f->edge->edge_next->vert_origin->z);
			QVector3D p3o(f->edge->edge_prev->vert_origin->x, f->edge->edge_prev->vert_origin->y, f->edge->edge_prev->vert_origin->z);

			QVector3D p1(QVector3D::dotProduct(p1o, v), QVector3D::dotProduct(p1o, u), QVector3D::dotProduct(p1o, n));
			QVector3D p2(QVector3D::dotProduct(p2o, v), QVector3D::dotProduct(p2o, u), QVector3D::dotProduct(p2o, n));
			QVector3D p3(QVector3D::dotProduct(p3o, v), QVector3D::dotProduct(p3o, u), QVector3D::dotProduct(p3o, n));

			QVector<QPoint> T = { QPoint(distance * p1.x() / (distance - p1.z()) + width() / 2, distance * p1.y() / (distance - p1.z()) + height() / 2),
				QPoint(distance * p2.x() / (distance - p2.z()) + width() / 2, distance * p2.y() / (distance - p2.z()) + height() / 2),
				QPoint(distance * p3.x() / (distance - p3.z()) + width() / 2, distance * p3.y() / (distance - p3.z()) + height() / 2) };

			rasterizeForFilling(prime, cameraDistance, F, T, { p1, p2, p3 }, shadingType, h);
		}
		break;
	}

	update();
}

QPoint ViewerWidget::projectPoint(const Vertex* vertex, double distance) {
	if (vertex->z <= -distance) {
		return QPoint();
	}

	int xPrime = static_cast<int>((distance * vertex->x) / (distance - vertex->z));
	int yPrime = static_cast<int>((distance * vertex->y) / (distance - vertex->z));

	int translatedX = xPrime + width() / 2;
	int translatedY = yPrime + height() / 2;

	return QPoint(translatedX, translatedY);
}


void ViewerWidget::rasterizeForFilling(Lighting prime, int cameraDistance, QVector<QVector<QColor>>& F, QVector<QPoint> T, QVector<QVector3D> p, int shadingType, int h) {
	std::sort(T.begin(), T.end(), [](const QPoint& a, const QPoint& b) {
		return a.y() == b.y() ? a.x() < b.x() : a.y() < b.y();
		});
	std::sort(p.begin(), p.end(), [](const QVector3D& a, const QVector3D& b) {
		return a.y() == b.y() ? a.x() < b.x() : a.y() < b.y();
		});

	QPoint p0 = T.at(0);
	QPoint p1 = T.at(1);
	QPoint p2 = T.at(2);

	auto slope = [](const QPoint& a, const QPoint& b) -> double {
		if (b.x() == a.x()) return std::numeric_limits<double>::max();
		return static_cast<double>(b.y() - a.y()) / static_cast<double>(b.x() - a.x());
		};

	if (p0.y() == p1.y()) {
		double m1 = slope(p2, p0);
		double m2 = slope(p2, p1);
		int ymin = p0.y();
		int ymax = p2.y();
		double x1 = static_cast<double>(p0.x());
		double x2 = static_cast<double>(p1.x());

		fillObject3D(prime, cameraDistance, x1, x2, m1, m2, ymin, ymax, T, p, shadingType, h);
	}
	else if (p1.y() == p2.y()) {
		double m1 = slope(p1, p0);
		double m2 = slope(p2, p0);
		int ymin = p0.y();
		int ymax = p1.y();
		double x1 = static_cast<double>(p0.x());
		double x2 = x1;

		fillObject3D(prime, cameraDistance, x1, x2, m1, m2, ymin, ymax, T, p, shadingType, h);
	}
	else {
		double m = slope(p2, p0);
		int y_intercept = static_cast<int>((p1.y() - p0.y()) / m + p0.x());

		QPoint P(y_intercept, p1.y());

		if (p1.x() < P.x()) {
			p0 = p1;
			p1 = P;
		}
		else {
			p0 = P;
		}

		double m1 = slope(p2, p0);
		double m2 = slope(p2, p1);
		int ymin = p0.y();
		int ymax = p2.y();
		double x1 = static_cast<double>(p0.x());
		double x2 = static_cast<double>(p1.x());

		fillObject3D(prime, cameraDistance, x1, x2, m1, m2, ymin, ymax, T, p, shadingType, h);

		p0 = T[0];
		p1 = T[1];
		p2 = T[2];
		if (p1.x() < P.x()) {
			p2 = P;
		}
		else {
			p2 = p1;
			p1 = P;
		}

		m1 = slope(p1, p0);
		m2 = slope(p2, p0);
		ymin = p0.y();
		ymax = p1.y();
		x1 = static_cast<double>(p0.x());
		x2 = x1;

		fillObject3D(prime, cameraDistance, x1, x2, m1, m2, ymin, ymax, T, p, shadingType, h);
	}
}

void ViewerWidget::fillObject3D(const Lighting& prime, int cameraDistance,
	double x1, double x2, double m1, double m2, int ymin, int ymax,
	const QVector<QPoint>& T, const QVector<QVector3D>& p, int shadingType, int h) {
	QVector3D camera(200, 200, cameraDistance);
	QVector3D source(prime.source.x, prime.source.y, prime.source.z);
	QColor color;
	QVector<QColor> col;
	QVector3D center;

	QVector3D p0(T[0].x(), T[0].y(), p.at(0).z());
	QVector3D p1(T[1].x(), T[1].y(), p.at(1).z());
	QVector3D p2(T[2].x(), T[2].y(), p.at(2).z());

	if (shadingType == 0) {
		col.append(phongModel(p0, prime, camera, source, h));
		col.append(phongModel(p1, prime, camera, source, h));
		col.append(phongModel(p2, prime, camera, source, h));
	}
	else {
		QVector3D center((p0 + p1 + p2) / 3.0);
		color = phongModel(center, prime, camera, source, h);
	}

	for (int y = ymin; y < ymax; y++) {
		if (x1 != x2) {
			int start = floor(x1);
			int end = floor(x2 + 1);
			for (int x = start; x < end; x++) {
				double z = interpolateZ(x, y, T, p);

				if (shadingType == 0) {
					color = interpolateColor(x, y, T, p, col);
				}

				if (isInside(x, y)) {
					if (Z[x][y] < z) {
						Z[x][y] = z;
						setPixel(x, y, color);
					}
				}
			}
		}
		x1 += 1. / m1;
		x2 += 1. / m2;
	}
}

QColor ViewerWidget::phongModel(const QVector3D& p, Lighting prime, QVector3D camera, QVector3D source, int h) {
	QVector3D view = (camera - p).normalized();
	QVector3D normal = p.normalized();
	QVector3D light = (p - source).normalized();
	QVector3D reflection = (2 * QVector3D::dotProduct(light, normal) * normal - light).normalized();

	double dot = pow(QVector3D::dotProduct(view, reflection), h);

	int rs = prime.source.r * prime.reflection.coeffR * dot;
	int gs = prime.source.g * prime.reflection.coeffG * dot;
	int bs = prime.source.b * prime.reflection.coeffB * dot;

	dot = QVector3D::dotProduct(light, normal);
	int rd = prime.source.r * prime.diffusion.coeffR * dot;
	int gd = prime.source.g * prime.diffusion.coeffG * dot;
	int bd = prime.source.b * prime.diffusion.coeffB * dot;

	int ra = prime.ambient.r * prime.ambient.coeffR;
	int ga = prime.ambient.g * prime.ambient.coeffG;
	int ba = prime.ambient.b * prime.ambient.coeffB;

	int r = rs + rd + ra;
	int g = gs + gd + ga;
	int b = bs + bd + ba;

	r = std::min(r, 255);
	g = std::min(g, 255);
	b = std::min(b, 255);

	r = std::max(0, r);
	g = std::max(0, g);
	b = std::max(0, b);

	return QColor(r, g, b);
}

double ViewerWidget::interpolateZ(const double& x, const double& y, QVector<QPoint> T, QVector<QVector3D> p) {
	double A = static_cast<double>(abs((T[1].x() - T[0].x()) * (T[2].y() - T[0].y()) - (T[1].y() - T[0].y()) * (T[2].x() - T[0].x())));

	double lambda0 = static_cast<double>(abs((T[1].x() - x) * (T[2].y() - y) - (T[1].y() - y) * (T[2].x() - x))) / A;
	double lambda1 = static_cast<double>(abs((T[0].x() - x) * (T[2].y() - y) - (T[0].y() - y) * (T[2].x() - x))) / A;
	double lambda2 = 1 - lambda0 - lambda1;

	return (lambda0 * p[0].z() + lambda1 * p[1].z() + lambda2 * p[2].z());
}

QColor ViewerWidget::interpolateColor(const double& x, const double& y, QVector<QPoint> T, QVector<QVector3D> p, QVector<QColor> col) {
	double A = static_cast<double>(abs((T[1].x() - T[0].x()) * (T[2].y() - T[0].y()) - (T[1].y() - T[0].y()) * (T[2].x() - T[0].x())));
	
	double lambda0 = static_cast<double>(abs((T[1].x() - x) * (T[2].y() - y) - (T[1].y() - y) * (T[2].x() - x))) / A;
	double lambda1 = static_cast<double>(abs((T[0].x() - x) * (T[2].y() - y) - (T[0].y() - y) * (T[2].x() - x))) / A;
	double lambda2 = 1 - lambda0 - lambda1;

	int r = lambda0 * col[0].red() + lambda1 * col[1].red() + lambda2 * col[2].red();
	int g = lambda0 * col[0].green() + lambda1 * col[1].green() + lambda2 * col[2].green();
	int b = lambda0 * col[0].blue() + lambda1 * col[1].blue() + lambda2 * col[2].blue();

	return QColor(r, g, b);
}
