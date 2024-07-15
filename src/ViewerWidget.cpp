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

	// Overenie, èi bola úseèka orezaná a aktualizácia linePoints pod¾a potreby
	if (lineToClip.size() == 2) { // Kontrola, èi orezanie zmenilo body
		linePoints.append(lineToClip[0]);
		linePoints.append(lineToClip[1]);
	}
	else {
		// Ak orezanie úplne odstránilo úseèku alebo nezmenilo body, vykreslenie pôvodnej úseèky
		linePoints.append(start);
		linePoints.append(end);
	}

	// Výber algoritmu kreslenia a aktualizácia
	if (algType == 0) {
		drawLineDDA(linePoints);
	}
	else {
		drawLineBresenham(linePoints);
	}

	update();
}

void ViewerWidget::drawLineDDA(QVector<QPoint>& linePoints) {
	int dx = linePoints.last().x() - linePoints.first().x(); // Výpoèet rozdielu x-ových súradníc
	int dy = linePoints.last().y() - linePoints.first().y(); // Výpoèet rozdielu y-ových súradníc

	if (abs(dx) > abs(dy)) {
		// Ak je absolutná hodnota dx väèšia, iterácia cez x
		double m = static_cast<double>(dy) / static_cast<double>(dx); // Výpoèet smernice èiary
		double y = linePoints.first().y();

		if (dx > 0) {
			// Prechod z ¾ava do prava
			for (int x = linePoints.first().x(); x <= linePoints.last().x(); x++) {
				painter->drawPoint(x, static_cast<int>(round(y)));
				y += m; // Aktualizácia y pod¾a smernice
			}
		}
		else {
			// Prechod z prava do ¾ava
			for (int x = linePoints.first().x(); x >= linePoints.last().x(); x--) {
				painter->drawPoint(x, static_cast<int>(round(y)));
				y -= m; // Aktualizácia y pod¾a smernice
			}
		}
	}
	else {
		// Ak je absolutná hodnota dy väèšia, iterácia cez y
		double m = static_cast<double>(dx) / static_cast<double>(dy);
		double x = linePoints.first().x();
		if (dy > 0) {
			// Prechod zdola nahor
			for (int y = linePoints.first().y(); y <= linePoints.last().y(); y++) {
				painter->drawPoint(static_cast<int>(round(x)),y);
				x += m; // Aktualizácia x pod¾a smernice
			}
		}
		else {
			// Prechod zhora nadol
			for (int y = linePoints.first().y(); y >= linePoints.last().y(); y--) {
				painter->drawPoint(static_cast<int>(round(x)), y);
				x -= m; // Aktualizácia x pod¾a smernice
			}
		}
	}

	update();
}

void ViewerWidget::drawLineBresenham(QVector<QPoint>& linePoints) {
	int p, k1, k2;
	int dx = linePoints.last().x() - linePoints.first().x();  // Rozdiel x súradníc
	int dy = linePoints.last().y() - linePoints.first().y();  // Rozdiel y súradníc

	int adx = abs(dx); // Absolútna hodnota dx
	int ady = abs(dy); // Absolútna hodnota dy

	int x = linePoints.first().x(); // Zaèiatoèná x pozícia
	int y = linePoints.first().y(); // Zaèiatoèná y pozícia

	int incrementX = (dx > 0) ? 1 : -1; // Urèenie smeru posunu po x-ovej osi
	int incrementY = (dy > 0) ? 1 : -1; // Urèenie smeru posunu po y-ovej osi

	if (adx > ady) {
		// Èiara je strmšia v x-ovej osi
		p = 2 * ady - adx;  // Inicializácia rozhodovacieho parametra
		k1 = 2 * ady;       // Konštanta pre horizontálny krok
		k2 = 2 * (ady - adx);  // Konštanta pre diagonálny krok

		while (x != linePoints.last().x()) {
			painter->drawPoint(x, y); // Kreslenie bodu na aktuálnych súradniciach
			x += incrementX; // Posun v x-ovej osi
			if (p >= 0) {
				y += incrementY; // Posun v y-ovej osi, ak je to potrebné
				p += k2; // Aktualizácia rozhodovacieho parametra
			}
			else {
				p += k1; // Aktualizácia rozhodovacieho parametra
			}
		}
	}
	else {
		// Èiara je strmšia v y-ovej osi
		p = 2 * adx - ady;  // Inicializácia rozhodovacieho parametra
		k1 = 2 * adx;       // Konštanta pre vertikálny krok
		k2 = 2 * (adx - ady);  // Konštanta pre diagonálny krok

		while (y != linePoints.last().y()) {
			painter->drawPoint(x, y); // Kreslenie bodu na aktuálnych súradniciach
			y += incrementY; // Posun v y-ovej osi
			if (p >= 0) {
				x += incrementX; // Posun v x-ovej osi, ak je to potrebné
				p += k2; // Aktualizácia rozhodovacieho parametra
			}
			else {
				p += k1; // Aktualizácia rozhodovacieho parametra
			}
		}
	}

	painter->drawPoint(linePoints.last().x(), linePoints.last().y()); // Vykreslenie posledného bodu
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

	// Kontrola, èi sú všetky body mimo definovaného plátna/kresliacej oblasti
	bool allPointsOutside = std::all_of(pointsVector.begin(), pointsVector.end(), [this](const QPoint& point) {
		return !isInside(point);
		});

	if (allPointsOutside) {
		qDebug() << "Polygon je mimo hranicu.";
		return;
	}

	// Kontrola každého bodu, èi sa nachádza v kresliacej oblasti, a prípadné orezanie polygonu
	for (QPoint point : pointsVector) {
		if (!isInside(point)) {
			polygon = trimPolygon(); // Orezanie polygónu, ak nejaké body prekraèujú hranice
			break;
		}
	}

	// Kreslenie obrysu polygónu
	if (!polygon.isEmpty()) {
		// Kreslenie liniek medzi bodmi polygónu
		for (int i = 0; i < polygon.size() - 1; i++) {
			drawLine(polygon.at(i), polygon.at(i + 1), color, algType);
		}
		// Zatvorenie polygónu kreslením èiary medzi posledným a prvým bodom
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

	// Výpoèet kosínusu a sínusu uhla pre použitie v transformaèných rovniciach
	double cosAngle = cos(radians);
	double sinAngle = sin(radians);

	QVector<QPoint> rotatedPoints;

	for (QPoint& point : linePoints) {
		// Prepoèet súradníc bodu do sústavy so stredom v bode otáèania
		int translatedX = point.x() - center.x();
		int translatedY = point.y() - center.y();

		// Výpoèet nových súradníc bodu po otoèení
		int rotatedX = static_cast<int>(translatedX * cosAngle - translatedY * sinAngle);
		int rotatedY = static_cast<int>(translatedX * sinAngle + translatedY * cosAngle);

		// Transformácia súradníc spä do pôvodnej súradnicovej sústavy
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

	// Výpoèet kosínusu a sínusu uhla pre použitie v transformaèných rovniciach
	double cosAngle = cos(radians);
	double sinAngle = sin(radians);

	QVector<QPoint> rotatedPoints;

	for (QPoint& point : pointsVector) {
		// Prepoèet súradníc bodu do sústavy so stredom v bode otáèania
		int translatedX = point.x() - center.x();
		int translatedY = point.y() - center.y();

		// Výpoèet nových súradníc bodu po otoèení
		int rotatedX = static_cast<int>(translatedX * cosAngle - translatedY * sinAngle);
		int rotatedY = static_cast<int>(translatedX * sinAngle + translatedY * cosAngle);

		// Transformácia súradníc spä do pôvodnej súradnicovej sústavy
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
		// Objektom osovej súmernosti je priamka
		int axisY = height() / 2; // Výpoèet polohy horizontálnej osi v rámci widgetu

		for (QPoint& point : linePoints) {
			// Osová súmernos pod¾a osi x (horizontálna)
			int mirroredY = 2 * axisY - point.y();
			point.setY(mirroredY);
		}

		clear();
		drawLine(linePoints.first(), linePoints.last(), color, algType);
		update();
	}
	else if(object == 1) {
		// Objektom osovej súmernosti je polygón
		QPoint center = getPolygonCenter();

		int axisX = center.x();
		int axisY = center.y();

		for (QPoint& point : pointsVector) {
			// Pre každý bod polygónu vypoèítame jeho zrkadlový obraz okolo stredového bodu polygónu
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
		int axisX = width() / 2; // Výpoèet polohy vertikálnej osi v rámci widgetu

		clear();
		for (QPoint& point : curvePoints) {
			// Osová súmernos pod¾a osi y (vertikálna)
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

	QVector<QPoint> W, polygon = pointsVector; // Inicializácia pomocného vektora a kópie pôvodného vektora bodov
	QPoint S; // Pomocný bod pre prácu s bodmi polygonu

	//qDebug() << "Pociatocny pointsVector:" << pointsVector;

	int xMin[] = { 0,0,-499,-499 }; // Hranice orezania pre x súradnice

	// Prechádzame štyri hranice orezania
	for (int i = 0; i < 4; i++) {
		if (pointsVector.size() == 0) {
			//qDebug() << "pointsVector ostal prazdny, vraciam polygon:" << polygon;
			return polygon;
		}

		S = polygon[polygon.size() - 1]; // Nastavenie S na posledný bod v polygone

		// Iterácia cez všetky body polygonu
		for (int j = 0; j < polygon.size(); j++) {
			// Logika orezania založená na pozícii bodu vzh¾adom na orezavaciu hranicu
			if (polygon[j].x() >= xMin[i]) {
				if (S.x() >= xMin[i]) {
					W.push_back(polygon[j]);
				}
				else {
					// Vytvorenie nového bodu na hranici orezania a jeho pridanie do výstupného vektora
					QPoint P(xMin[i], S.y() + (xMin[i] - S.x()) * ((polygon[j].y() - S.y()) / static_cast<double>((polygon[j].x() - S.x()))));
					W.push_back(P);
					W.push_back(polygon[j]);
				}
			}
			else {
				if (S.x() >= xMin[i]) {
					// Vytvorenie bodu na hranici a pridanie do W, ak predchádzajúci bod bol vnútri orezanej oblasti
					QPoint P(xMin[i], S.y() + (xMin[i] - S.x()) * ((polygon[j].y() - S.y()) / static_cast<double>((polygon[j].x() - S.x()))));
					W.push_back(P);
				}
			}
			S = polygon[j]; // Aktualizácia S na aktuálny bod pre ïalšiu iteráciu
		}
		//qDebug() << "Po orezavani s xMin[" << i << "] =" << xMin[i] << "W:" << W;
		polygon = W; // Nastavenie orezaného polygonu ako aktuálneho polygonu pre ïalšiu iteráciu
		W.clear(); // Vymazanie pomocného vektora pre ïalšie použitie

		// Rotácia bodov polygonu pre ïalšiu hranicu orezania
		for (int j = 0; j < polygon.size(); j++) {
			QPoint swappingPoint = polygon[j];
			polygon[j].setX(swappingPoint.y());
			polygon[j].setY(-swappingPoint.x());
		}
		//qDebug() << "Po vymene, polygon:" << polygon;
	}

	//qDebug() << "Vysledny orezany polygon:" << polygon;
	return polygon;
}


//-----------------------------------------
//		*** Line trimming function ***
//-----------------------------------------

void ViewerWidget::clipLineWithPolygon(QVector<QPoint> linePoints) {
	if (linePoints.size() < 2) {
		return; // Nedostatok bodov na vytvorenie èiary
	}

	QVector<QPoint> clippedPoints;
	QPoint P1 = linePoints[0], P2 = linePoints[1];
	double t_min = 0, t_max = 1; // Inicializácia t-hodnôt
	QPoint d = P2 - P1; // Smerový vektor úseèky
	//qDebug() << "Povodny useckovy segment od" << P1 << "do" << P2;

	// Definícia hrán orezovacieho obdåžnika
	QVector<QPoint> E = { QPoint(0,0), QPoint(500,0), QPoint(500,500), QPoint(0,500) };

	for (int i = 0; i < E.size(); i++) {
		QPoint E1 = E[i];
		QPoint E2 = E[(i + 1) % E.size()]; // Zopnutie pre poslednú hranu

		QPoint normal = QPoint(E2.y() - E1.y(), E1.x() - E2.x()); // Opravené znamienko

		QPoint w = P1 - E1; // Vektor z koncového bodu hrany k P1

		double dn = d.x() * normal.x() + d.y() * normal.y();
		double wn = w.x() * normal.x() + w.y() * normal.y();
		if (dn != 0) {
			double t = -wn / dn;
			//qDebug() << "Hodnota t priesecnika s hranou" << i << ":" << t;
			if (dn > 0 && t <= 1) {
				t_min = std::max(t, t_min); // Aktualizácia t_min, ak dn > 0 a t <= 1
			}
			else if (dn < 0 && t >= 0) {
				t_max = std::min(t, t_max); // Aktualizácia t_max, ak dn < 0 a t >= 0
			}
		}
	}

	//qDebug() << "t_min:" << t_min << "t_max:" << t_max;

	if (t_min < t_max) {
		QPoint clippedP1 = P1 + (P2 - P1) * t_min; // Výpoèet orezaného zaèiatoèného bodu
		QPoint clippedP2 = P1 + (P2 - P1) * t_max; // Výpoèet orezaného koncového bodu
		//qDebug() << "Orezany useckovy segment od" << clippedP1 << "do" << clippedP2;

		clippedPoints.push_back(clippedP1);
		clippedPoints.push_back(clippedP2);
	}
	else {
		//qDebug() << "Useckovy segment je uplne mimo orezovacej oblasti alebo je neplatny.";
	}

	// Aktualizácia pôvodných linePoints s orezanými bodmi
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
		// Urèenie zaèiatoèného a koncového bodu hrany
		QPoint startPoint = points[i];
		QPoint endPoint = points[(i + 1) % points.size()]; // Po poslednom bode, vrátenie sa na prvý

		// Priame vytvorenie hrany bez manuálneho výpoètu sklonu
		Edge edge(startPoint, endPoint);

		// Upravenie koncového bodu hrany pod¾a pôvodnej logiky, ak je to potrebné
		edge.adjustEndPoint();

		edges.push_back(edge);
	}

	// Prepoèet sklonu a zmena bodov prebieha v konštruktore triedy

	std::sort(edges.begin(), edges.end(), compareByY); // Usporiadanie hrán pod¾a ich y-ovej súradnice
	return edges;
}

void ViewerWidget::fillPolygon(QVector<QPoint>& points, const QColor& color, const QColor& color1, const QColor& color2, int interpolType) {
	if (points.isEmpty()) {
		//qDebug() << "Neobsahuje body pre vyplnanie.";
		return;
	}

	if (points.size() == 3) {
		fillTriangle(points, color, color1, color2, interpolType);
	}
	else {
		// Naèítanie hrán z bodov
		QVector<Edge> edges = loadEdges(points);
		if (edges.isEmpty()) {
			//qDebug() << "Vektor hran je prazdny.";
			return; // Predèasný výstup, ak neboli generované žiadne hrany
		}

		// Inicializácia yMin a yMax na základe prvej hrany
		int yMin = edges.front().startPoint().y();
		int yMax = edges.front().endPoint().y();

		// Nájdenie celkových yMin a yMax hodnôt
		for (const Edge& edge : edges) {
			int y1 = edge.startPoint().y();
			int y2 = edge.endPoint().y();
			yMin = qMin(yMin, qMin(y1, y2));
			yMax = qMax(yMax, qMax(y1, y2));
		}

		//qDebug() << "Prepocitane yMin:" << yMin << "yMax:" << yMax;

		// Kontrola platnosti hodnôt yMin a yMax
		if (yMin >= yMax) {
			//qDebug() << "Neplatne yMin a yMax hodnoty. Mozne nespravne nastavenie hrany.";
			return;
		}

		// Tabu¾ka hrán, inicializovaná tak, aby pokrývala od yMin po yMax
		QVector<QVector<Edge>> TH(yMax - yMin + 1);

		//qDebug() << "yMin:" << yMin << "yMax:" << yMax;

		// Populácia tabu¾ky hrán
		for (const auto& edge : edges) {
			int index = edge.startPoint().y() - yMin; // Index založený na offsete yMin
			if (index < 0 || index >= TH.size()) {
				//qDebug() << "Invalid index:" << index << "for edge start point y:" << edge.startPoint().y();
				continue;
			}
			TH[index].append(edge);
		}

		QVector<Edge> activeEdgeList; // Zoznam aktívnych hrán (AEL)

		// Zaèiatok prechodu scan line od yMin po yMax
		for (int y = yMin; y <= yMax; y++) {
			// Pridanie hrán do AEL
			for (const auto& edge : TH[y - yMin]) {
				activeEdgeList.append(edge);
			}

			// Zoradenie AEL pod¾a aktuálnej hodnoty X
			std::sort(activeEdgeList.begin(), activeEdgeList.end(), [](const Edge& a, const Edge& b) {
				return a.x() < b.x();
				});

			// Kreslenie èiar medzi pármi hodnôt X
			for (int i = 0; i < activeEdgeList.size(); i += 2) {
				if (i + 1 < activeEdgeList.size()) {
					int startX = qRound(activeEdgeList[i].x());
					int endX = qRound(activeEdgeList[i + 1].x());
					for (int x = startX; x <= endX; x++) {
						setPixel(x, y, color); // Vyplnenie medzi hranami
					}
				}
			}

			// Aktualizácia a odstránenie hrán z AEL
			QMutableVectorIterator<Edge> it(activeEdgeList);
			while (it.hasNext()) {
				Edge& edge = it.next();
				if (edge.endPoint().y() == y) {
					it.remove(); // Odstránenie hrany, ak konèí na aktuálnej scan line
				}
				else {
					edge.setX(edge.x() + edge.w()); // Aktualizácia X pre ïalšiu scan line
				}
			}
		}
	}
}

//-----------------------------------------
//		*** Filling of Triangle ***
//-----------------------------------------

void ViewerWidget::fillTriangle(QVector<QPoint>& points, const QColor& color, const QColor& color1, const QColor& color2, int interpolType) {
	// Usporiadanie vrcholov trojuholníka pod¾a y-súradnice vzostupne, pri rovnakom y pod¾a x-súradnice
	std::sort(points.begin(), points.end(), [](const QPoint& a, const QPoint& b) {
		return a.y() < b.y() || (a.y() == b.y() && a.x() < b.x());
		});

	// Priradenie usporiadaných vrcholov trojuholníka
	const QPoint& T0 = points[0]; // Vrchol s najmenšou y-súradnicou
	const QPoint& T1 = points[1]; // Vrchol s prostrednou y-súradnicou
	const QPoint& T2 = points[2]; // Vrchol s najväèšou y-súradnicou

	// Kontrola, èi je trojuholník s plochým dnom alebo vrchom
	if (T0.y() == T1.y()) {
		fillFlatBottomTriangle(T0, T1, T2, color, color1, color2, interpolType);
	}
	else if (T1.y() == T2.y()) {
		fillFlatTopTriangle(T0, T1, T2, color, color1, color2, interpolType);
	}
	else {
		// Výpoèet smernice strany trojuholníka od T0 po T2
		double m = (T2.x() - T0.x()) != 0 ? static_cast<double>(T2.y() - T0.y()) / (T2.x() - T0.x()) : std::numeric_limits<double>::max();

		// **Výpoèet deliaceho bodu P pomocou smernice**
		// Výpoèet x-súradnice bodu P, ktorý leží na spojnici bodov T0 a T2 v horizontálnej úrovni bodu T1.
		// Ak nie je smernica m nekoneèno (èo znamená, že spojnica nie je zvislá), vypoèíta sa x-súradnica bodu P
		// ako bod na spojnici T0 a T2, ktorý je horizontálne v rovnakej výške ako T1.
		// Ak je spojnica zvislá (smernica m je nekoneèno), x-súradnica bodu P bude rovnaká ako x-súradnica T0,
		// pretože zvislý posun nezmení x-súradnicu.
		QPoint P(
			m != std::numeric_limits<double>::max() ? static_cast<int>(T0.x() + (T1.y() - T0.y()) / m) : T0.x(),
			T1.y()
		);

		// Ak je x-súradnica bodu T1 menšia ako x-súradnica bodu P, vyplníme trojuholník ako s plochým vrchom
		if (T1.x() < P.x()) {
			// Vyplnenie horného trojuholníka s plochým dnom (T0, T1, P)
			for (int y = T0.y(); y <= T1.y(); y++) {
				// Výpoèet x-súradníc pre zaèiatok a koniec horizontálnej línie
				int x1 = static_cast<int>((y - T0.y()) * (T1.x() - T0.x()) / (T1.y() - T0.y()) + T0.x());
				int x2 = static_cast<int>((y - T0.y()) * (P.x() - T0.x()) / (P.y() - T0.y()) + T0.x());
				
				// Kreslenie horizontálnej línie s použitím interpolácie farieb
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
			// Vyplnenie dolného trojuholníka s plochým vrchom (T1, P, T2)
			for (int y = T1.y(); y <= T2.y(); y++) {
				// Výpoèet x-súradníc pre zaèiatok a koniec horizontálnej línie
				int x1 = static_cast<int>((y - T1.y()) * (T2.x() - T1.x()) / (T2.y() - T1.y()) + T1.x());
				int x2 = static_cast<int>((y - P.y()) * (T2.x() - P.x()) / (T2.y() - P.y()) + P.x());
				
				// Kreslenie horizontálnej línie s použitím interpolácie farieb
				for (int x = x1; x <= x2; x++) {
					QColor activeColor;
					if (interpolType == 0) {
						//	<< Nearest neighbor algoritmus >>

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
						//	<< Barycentrická interpolácia >>

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
		// Ak je x-súradnica bodu T1 väèšia alebo rovná ako x-súradnica bodu P
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

// Funkcia na zistenie, èi je bod P vnútri trojuholníka ABC
bool ViewerWidget::isPointInsideTriangle(const QPoint& P, const QPoint& A, const QPoint& B, const QPoint& C) {
	// Vypoèítame súradnice vektora AP
	int as_x = P.x() - A.x();
	int as_y = P.y() - A.y();

	// Urèíme, na ktorej strane priamky AB sa bod P nachádza
	bool s_ab = (B.x() - A.x()) * as_y - (B.y() - A.y()) * as_x > 0;

	// Ak sa bod P nachádza na opaènej strane priamky AC, nie je vnútri trojuholníka
	if ((C.x() - A.x()) * as_y - (C.y() - A.y()) * as_x > 0 == s_ab) return false;

	// Ak sa bod P nachádza na opaènej strane priamky BC, nie je vnútri trojuholníka
	if ((C.x() - B.x()) * (P.y() - B.y()) - (C.y() - B.y()) * (P.x() - B.x()) > 0 != s_ab) return false;

	return true;
}

void ViewerWidget::fillFlatBottomTriangle(const QPoint& T0, const QPoint& T1, const QPoint& T2, const QColor& color, const QColor& color1, const QColor& color2, int interpolType) {
	// Výpoèet inverzných smerníc (W) pre oba okraje trojuholníka
	double W1 = (T1.x() - T0.x()) != 0 ? (T1.y() - T0.y()) / static_cast<double>(T1.x() - T0.x()) : std::numeric_limits<double>::max();
	double W2 = (T2.x() - T0.x()) != 0 ? (T2.y() - T0.y()) / static_cast<double>(T2.x() - T0.x()) : std::numeric_limits<double>::max();

	// Iterácia od y-súradnice bodu T0 po y-súradnicu bodu T1 (predpokladá sa, že T1.y == T2.y, pretože je to trojuholník s plochým dnom)
	for (int y = T0.y(); y <= T1.y(); y++) {
		// Výpoèet zaèiatoènej a koncovej x-súradnice použitím inverzných smerníc
		int startX = T0.x() + static_cast<int>((y - T0.y()) * W1);
		int endX = T0.x() + static_cast<int>((y - T0.y()) * W2);

		// Zabezpeèenie, že startX je menšie ako endX
		if (startX > endX) std::swap(startX, endX);

		// Vyplnenie horizontálnej èiary
		for (int x = startX; x <= endX; x++) {
			//	<< Nearest neighbor algoritmus >>
			if (interpolType == 0) {
				// Výpoèet vzdialeností od bodov trojuholníka
				double length1 = sqrt(pow(T0.x() - x, 2) + pow(T0.y() - y, 2));
				double length2 = sqrt(pow(T1.x() - x, 2) + pow(T1.y() - y, 2));
				double length3 = sqrt(pow(T2.x() - x, 2) + pow(T2.y() - y, 2));

				// Urèenie aktívnej farby pod¾a najbližšieho vrcholu
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

				// Overenie, èi je bod vnútri trojuholníka a v prípade potreby nastavenie pixelu
				if (isPointInsideTriangle(QPoint(x, y), T0, T1, T2)) {
					setPixel(x, y, activeColor);
				}
			}
			// << Barycentrická interpolácia >>
			else {
				// Bod pre kontrolu a interpoláciu
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
	// Výpoèet inverzných smerníc (W) pre oba okraje trojuholníka
	double W1 = (T2.x() - T0.x()) != 0 ? (T2.y() - T0.y()) / static_cast<double>(T2.x() - T0.x()) : std::numeric_limits<double>::max();
	double W2 = (T2.x() - T1.x()) != 0 ? (T2.y() - T1.y()) / static_cast<double>(T2.x() - T1.x()) : std::numeric_limits<double>::max();

	// Iterácia od y-súradnice bodu T2 po y-súradnicu bodu T0 (predpokladá sa, že T0.y == T1.y, pretože je to trojuholník s plochým vrchom)
	for (int y = T2.y(); y >= T0.y(); y--) {
		// Výpoèet zaèiatoènej a koncovej x-súradnice použitím inverzných smerníc
		int startX = T2.x() - static_cast<int>((T2.y() - y) * W1);
		int endX = T2.x() - static_cast<int>((T2.y() - y) * W2);

		// Zabezpeèenie, že startX je menšie ako endX
		if (startX > endX) std::swap(startX, endX);

		// Vyplnenie horizontálnej èiary
		for (int x = startX; x <= endX; x++) {
			if (interpolType == 0) {
				// << Nearest neighbor algoritmus >>
				double length1 = sqrt(pow(T0.x() - x, 2) + pow(T0.y() - y, 2));
				double length2 = sqrt(pow(T1.x() - x, 2) + pow(T1.y() - y, 2));
				double length3 = sqrt(pow(T2.x() - x, 2) + pow(T2.y() - y, 2));

				// Urèenie aktívnej farby pod¾a najbližšieho vrcholu
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
				// << Barycentricka interpolacia >>
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
	// Vypoèítame celkovú plochu trojuholníka pre barycentrické súradnice
	double areaTotal = (T1.y() - T2.y()) * (T0.x() - T2.x()) + (T2.x() - T1.x()) * (T0.y() - T2.y());

	// Vypoèítame barycentrické koeficienty lambda pre bod P
	double lambda0 = ((T1.y() - T2.y()) * (P.x() - T2.x()) + (T2.x() - T1.x()) * (P.y() - T2.y())) / areaTotal;
	double lambda1 = ((T2.y() - T0.y()) * (P.x() - T2.x()) + (T0.x() - T2.x()) * (P.y() - T2.y())) / areaTotal;
	double lambda2 = 1 - lambda0 - lambda1;

	// Uistíme sa, že lambda koeficienty sú v intervale [0, 1]
	lambda0 = std::max(0.0, std::min(1.0, lambda0));
	lambda1 = std::max(0.0, std::min(1.0, lambda1));
	lambda2 = std::max(0.0, std::min(1.0, lambda2));

	// Interpolujeme farby založené na barycentrických koeficientoch
	int r = static_cast<int>(lambda0 * color0.red() + lambda1 * color1.red() + lambda2 * color2.red());
	int g = static_cast<int>(lambda0 * color0.green() + lambda1 * color1.green() + lambda2 * color2.green());
	int b = static_cast<int>(lambda0 * color0.blue() + lambda1 * color1.blue() + lambda2 * color2.blue());

	// Orezávame hodnoty RGB, aby boli v platnom rozsahu [0, 255]
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
		// << Hermitovská kubika >>

		if (curvePoints.size() < 2) {
			QMessageBox::warning(this, "Nedostatocny pocet bodov", "Nemozno nakreslit krivku s menej ako dvomi riadiacimi bodmi.", QMessageBox::Ok);
			return;
		}
		
		drawTangentVectors(color2, algType);
		
		// Definícia lambda bázických funkcií Hermitovej krivky
		auto hermiteBasisF0 = [](double t) { return 2 * t * t * t - 3 * t * t + 1; };
		auto hermiteBasisF1 = [](double t) { return t * t * t - 2 * t * t + t; };
		auto hermiteBasisF2 = [](double t) { return -2 * t * t * t + 3 * t * t; };
		auto hermiteBasisF3 = [](double t) { return t * t * t - t * t; };

		// Kreslenie krivky pod¾a riadiacich bodov
		float deltaT = 0.01f;
		for (int i = 1; i < curvePoints.size(); i++) {
			QPoint Q0 = curvePoints[i - 1];

			for (float t = deltaT; t < 1.0f; t += deltaT) {
				QPoint Q1(
					curvePoints[i - 1].x() * hermiteBasisF0(t) + curvePoints[i].x() * hermiteBasisF2(t) + tangents[i - 1].x() * hermiteBasisF1(t) + tangents[i].x() * hermiteBasisF3(t),
					curvePoints[i - 1].y() * hermiteBasisF0(t) + curvePoints[i].y() * hermiteBasisF2(t) + tangents[i - 1].y() * hermiteBasisF1(t) + tangents[i].y() * hermiteBasisF3(t)
				);

				// Vykreslenie segmentu krivky
				drawLine(Q0, Q1, color, algType);
				Q0 = Q1;
			}

			// Vykreslenie posledného segmentu krivky
			drawLine(Q0, curvePoints[i], color, algType);
		}
	}
	else if (curveType == 1) {
		// << Beziérova krivka >>

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
		// << Coonsov kubický B-spline >>

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

	const int tangentLength = 70; // Preddefinovaná dåžka

	for (int i = 0; i < tangents.size(); i++) {
		QPoint currentPoint = curvePoints[i];  // Aktuálny riadiaci bod
		QPoint tangentVector = tangents[i]; // Dotyènicový vektor pre aktuálny bod

		// Normalizácia dotyènicového vektora na jednotkovú dåžku
		double length = std::sqrt(tangentVector.x() * tangentVector.x() + tangentVector.y() * tangentVector.y());
		if (length == 0)
			continue; // Preskoèenie výpoètu, ak je dåžka vektora 0

		double unitX = tangentVector.x() / length; // Výpoèet jednotkovej x-súradnice vektora
		double unitY = tangentVector.y() / length; // Výpoèet jednotkovej y-súradnice vektora

		// Vypoèítanie koncového bodu dotyènicového vektora z jeho zaèiatoèného bodu a normalizovaného smeru
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

	// Výpoèet dotyènicového vektora pre prvý kontrolný bod
	QPoint firstTangent = (curvePoints[1] - curvePoints[0]);
	tangents.push_back(firstTangent);

	for (int i = 1; i < curvePoints.size() - 1; i++) {
		// Pre každý stredný bod vypoèíta dotyènicový vektor ako priemer vektorov smerujúcich k predchádzajúcemu a nasledujúcemu bodu
		QPoint prevVector = curvePoints[i] - curvePoints[i - 1];
		QPoint nextVector = curvePoints[i + 1] - curvePoints[i];
		QPoint tangent = (prevVector + nextVector) / 2;
		tangents.push_back(tangent);
	}

	// Výpoèet dotyènicového vektora pre posledný kontrolný bod
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
		// Scale each control point
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

	double minDistance = std::numeric_limits<double>::max(); // Inicializácia minimálnej vzdialenosti na maximálnu možnú hodnotu
	QPoint nearestPoint; // Inicializácia najbližšieho bodu

	// Iterácia cez všetky riadiace body a ich dotyènicové vektory
	for (int i = 0; i < curvePoints.size(); i++) {
		QPoint startPoint = curvePoints[i]; // Riadiaci bod krivky
		QPoint endPoint = startPoint + tangents[i]; // Koncový bod dotyènicového vektora

		// Výpoèet vzdialeností od zadaného bodu (pozícii myšky) k riadiacemu bodu a koncovému bodu tangentového vektora
		double distanceToStart = std::sqrt(std::pow(startPoint.x() - position.x(), 2) + std::pow(startPoint.y() - position.y(), 2));
		double distanceToEnd = std::sqrt(std::pow(endPoint.x() - position.x(), 2) + std::pow(endPoint.y() - position.y(), 2));

		// Aktualizácia najbližšieho bodu, ak je nájdená menšia vzdialenos
		if (distanceToStart < minDistance) {
			minDistance = distanceToStart;
			nearestPoint = startPoint;
		}
		if (distanceToEnd < minDistance) {
			minDistance = distanceToEnd;
			nearestPoint = endPoint;
		}
	}

	return nearestPoint; // Uloží do moveStart
}

void ViewerWidget::moveTheNearestPoint(const QColor& color, const QColor& color2, const QPoint& offset, int curveType, int algType) {
	if (curvePoints.isEmpty() || tangents.isEmpty())
		return;
	if (moveStart.isNull())
		return;

	bool isControlPoint = false; // Premenná urèujúca, èi je najbližší bod riadiaci bod
	int index = -1; // Index najbližšieho bodu
	double minDistance = std::numeric_limits<double>::max(); // Inicializácia minimálnej vzdialenosti na maximálnu možnú hodnotu

	// Iterácia cez všetky riadiace body a ich dotyènicové vektory na nájdenie najbližšieho bodu k moveStart
	for (int i = 0; i < curvePoints.size(); i++) {
		double distanceToStart = std::hypot(curvePoints[i].x() - moveStart.x(), curvePoints[i].y() - moveStart.y());
		QPoint endPoint = curvePoints[i] + tangents[i];
		double distanceToEnd = std::hypot(endPoint.x() - moveStart.x(), endPoint.y() - moveStart.y());

		// Aktualizácia najbližšieho bodu a jeho indexu, ak je nájdená menšia vzdialenos
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

	// Ak bol najbližší bod identifikovaný, vykoná sa jeho posunutie
	if (index != -1) {
		if (isControlPoint) {
			curvePoints[index] += offset;
		}
		else {
			// Nový koncový bod dotyènicového vektora je vypoèítaný ako súèet pôvodného koncového bodu a offsetu
			QPoint newEndPoint = (curvePoints[index] + tangents[index]) + offset;
			// Dotyènicový vektor je aktualizovaný tak, aby odrážal novú pozíciu jeho koncového bodu
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

	// Definicia vrcholov kocky
	std::vector<Vertex> vertices = {
		{-half, -half, -half}, {half, -half, -half},
		{half, half, -half}, {-half, half, -half},
		{-half, -half, half}, {half, -half, half},
		{half, half, half}, {-half, half, half}
	};

	// Definicia stien kocky pomocou indexov vrcholov
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

	// Zapis vrcholov do suboru
	out << "POINTS " << vertices.size() << " float\n";
	for (const auto& vertex : vertices) {
		out << vertex.x << " " << vertex.y << " " << vertex.z << "\n";
	}

	// Zapis stien kocky do suboru
	int numberOfTriangles = faces.size();
	int totalIndices = numberOfTriangles * 4; // Vypocet celkoveho poctu indexov (tri vrcholy + pocet trojuholnikov)
	out << "POLYGONS " << numberOfTriangles << " " << totalIndices << "\n";
	for (const auto& face : faces) {
		out << "3"; // Zapis poctu vrcholov trojuholnika
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

	// Generovanie bodov sfery podla zvolenych rovnobeziek a poludnikov
	for (int p = 0; p <= parallels; p++) {
		double phi = M_PI * double(p) / double(parallels);
		for (int m = 0; m <= meridians; m++) {
			double theta = 2.0 * M_PI * double(m) / double(meridians); // Azimutalny uhol
			points.push_back(Point{
				radius * sin(phi) * cos(theta),
				radius * sin(phi) * sin(theta),
				radius * cos(phi)
				});
		}
	}

	// Vytvorenie trojuholnikov pre VTK subor
	for (int p = 0; p < parallels; p++) {
		for (int m = 0; m < meridians; m++) {
			int current = p * (meridians + 1) + m;
			int next = current + meridians + 1;

			triangles.push_back(Triangle{ current, next, current + 1 }); // Dolny trojuholnik

			if (p != (parallels - 1)) {
				triangles.push_back(Triangle{ current + 1, next, next + 1 }); // Horny trojuholnik
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

	// Zapis bodov do suboru
	out << "POINTS " << points.size() << " float\n";
	for (const auto& point : points) {
		out << point.x << " " << point.y << " " << point.z << "\n";
	}

	// Zapis trojuholnikov do suboru
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

	bool readingPoints = false; // Indikator citania bodov
	bool readingPolygons = false; // Indikator citania polygonov

	while (!in.atEnd()) {
		line = in.readLine().trimmed(); // Citanie a orezanie riadku

		if (line.isEmpty() || line.startsWith("#")) { // Preskocenie prazdnych riadkov a komentarov
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

		// Spracovanie bodov
		if (readingPoints) {
			QStringList coords = line.split(" ", Qt::SkipEmptyParts);
			double x = coords[0].toDouble();
			double y = coords[1].toDouble();
			double z = coords[2].toDouble();
			Vertex* vertex = new Vertex(x, y, z);
			cubeVertices.append(vertex);
		}

		// Spracovanie polygonov
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

			// Zatvorenie kruhu hran
			prevEdge->edge_next = firstEdge;
			firstEdge->edge_prev = prevEdge;

			// Vytvorenie novej steny
			Face* newFace = new Face(firstEdge);
			foreach(H_edge * edge, faceEdges) {
				edge->face = newFace;
			}
			cubeFaces.append(newFace);
		}
	}

	file.close();

	// Spojenie parovych hran
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
	QHash<QPair<int, int>, H_edge*> edgeMap; // Hash mapa pre mapovanie hrán

	QString line;
	while (!in.atEnd()) {
		line = in.readLine();
		if (line.contains("POINTS")) {
			int numPoints = line.split(' ')[1].toInt(); // Citanie poctu bodov
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
			int numPolygons = line.split(' ')[1].toInt(); // Citanie poctu polygonov
			for (int i = 0; i < numPolygons; i++) {
				line = in.readLine();
				QStringList indices = line.split(' ');
				int numVertices = indices[0].toInt(); // Citanie poctu vrcholov v polygone

				QVector<H_edge*> polyEdges; // Vektor hran polygona
				for (int j = 0; j < numVertices; j++) {
					int idx = indices[j + 1].toInt();
					H_edge* edge = new H_edge(sphereVertices[idx], nullptr);
					sphereEdges.append(edge);
					polyEdges.append(edge);
				}

				for (int j = 0; j < numVertices; j++) {
					polyEdges[j]->edge_next = polyEdges[(j + 1) % numVertices]; // Nastavenie nasledujucich hran
					polyEdges[(j + 1) % numVertices]->edge_prev = polyEdges[j]; // Nastavenie predchadzajucich hran
					int startIndex = sphereVertices.indexOf(polyEdges[j]->vert_origin);
					int endIndex = sphereVertices.indexOf(polyEdges[(j + 1) % numVertices]->vert_origin);
					edgeMap.insert(QPair<int, int>(startIndex, endIndex), polyEdges[j]); // Ulozenie hrany do mapy
				}

				Face* face = new Face(polyEdges[0]);
				sphereFaces.append(face);
				for (H_edge* edge : polyEdges) {
					edge->face = face; // Priradenie steny ku hranam
				}
			}
		}
	}

	// Nastavenie parovych hran pre vsetky hranové spoje
	for (H_edge* edge : sphereEdges) {
		int startIdx = sphereVertices.indexOf(edge->vert_origin);
		int endIdx = sphereVertices.indexOf(edge->edge_next->vert_origin);
		H_edge* pairEdge = edgeMap.value(QPair<int, int>(endIdx, startIdx));
		if (pairEdge) {
			edge->pair = pairEdge; // Nastavenie parovej hrany
			pairEdge->pair = edge;
		}
	}

	file.close();
}


void ViewerWidget::adjustProjection(double theta, double phi, bool objectLoaded, int objectType, int algType, int projectionType, double distance, int cameraDistance, Lighting prime) {
	if (!objectLoaded) {
		return;
	}

	// Urcenie smeru otacania pre theta a phi
	int thetaDirection = (theta < prevTheta) ? -1 : 1;
	int phiDirection = (phi < prevPhi) ? -1 : 1;

	QMatrix4x4 rotationMatrix;

	// Aplikacia rotacie v zavislosti od zmeny uhla theta a phi
	if (theta != prevTheta) {
		rotationMatrix.rotate(qRadiansToDegrees(theta) * thetaDirection, 1, 0, 0);
	}
	else if (phi != prevPhi) {
		rotationMatrix.rotate(qRadiansToDegrees(phi) * phiDirection, 0, 1, 0);
	}

	prevTheta = theta;
	prevPhi = phi;

	// Vypocet normaloveho vektora 'n' a pomocnych vektorov 'u' a 'v' pre rotaciu
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

	// Stred kresliacej plochy
	QVector3D drawingAreaCenter(width() / 2.0f, height() / 2.0f, 0.0f);

	if (vertices) {
		originalVertices.reserve(vertices->size()); // Rezervacia miesta pre pociatocne vrcholy
		QVector3D center = computeCenter(*vertices);

		// Rotacia kazdeho vrchola okolo stredu objektu
		for (Vertex* vertex : *vertices) {
			QVector3D pos(vertex->x, vertex->y, vertex->z);
			pos -= center; // Posunutie vrchola do stredu suradnicoveho systemu
			pos = rotationMatrix * pos; // Rotacia vrchola
			pos += center; // Posunutie vrchola spat k jeho povodnej pozicii
			originalVertices.append(pos); // Pridanie novej pozicie do zoznamu
		}

		// Aktualizacia pozicii vrcholov v hlavnom zozname
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

	// Mnozina pre uchovavanie uz vykreslenych hran
	QSet<QPair<int, int>> drawnEdges;

	switch (objectType) {
	case isCube:
		// Iteracia cez steny kocky
		for (Face* f : cubeFaces) {
			if (!f || !f->edge || !f->edge->vert_origin || !f->edge->edge_next || !f->edge->edge_prev) {
				continue; // Preskocenie neplatnych stien
			}

			// Vypocet projekcii troch bodov steny
			QVector3D p1o(f->edge->vert_origin->x, f->edge->vert_origin->y, f->edge->vert_origin->z);
			QVector3D p2o(f->edge->edge_next->vert_origin->x, f->edge->edge_next->vert_origin->y, f->edge->edge_next->vert_origin->z);
			QVector3D p3o(f->edge->edge_prev->vert_origin->x, f->edge->edge_prev->vert_origin->y, f->edge->edge_prev->vert_origin->z);

			QVector3D p1(QVector3D::dotProduct(p1o, v), QVector3D::dotProduct(p1o, u), QVector3D::dotProduct(p1o, n));
			QVector3D p2(QVector3D::dotProduct(p2o, v), QVector3D::dotProduct(p2o, u), QVector3D::dotProduct(p2o, n));
			QVector3D p3(QVector3D::dotProduct(p3o, v), QVector3D::dotProduct(p3o, u), QVector3D::dotProduct(p3o, n));

			// Vytvorenie trojuholnika pre Z-buffer a rasterizaciu
			QVector<QPoint> T = {
				QPoint(static_cast<int>(p1.x()) + width() / 2, static_cast<int>(p1.y()) + height() / 2),
				QPoint(static_cast<int>(p2.x()) + width() / 2, static_cast<int>(p2.y()) + height() / 2),
				QPoint(static_cast<int>(p3.x()) + width() / 2, static_cast<int>(p3.y()) + height() / 2)
			};

			rasterizeForFilling(prime, cameraDistance, F, T, { p1, p2, p3 }, shadingType, h);
		}

		break;

	case isCubeWireFrame:
		// Iteracia cez hrany kocky
		for (auto* edge : cubeEdges) {
			if (!edge || !edge->vert_origin || !edge->edge_next || !edge->edge_next->vert_origin) continue;

			// Kontrola, ci uz bola hrana vykreslena
			int startIdx = cubeVertices.indexOf(edge->vert_origin);
			int endIdx = cubeVertices.indexOf(edge->edge_next->vert_origin);

			if (!drawnEdges.contains(qMakePair(endIdx, startIdx)) && !drawnEdges.contains(qMakePair(startIdx, endIdx))) {
				drawnEdges.insert(qMakePair(startIdx, endIdx));

				// Vypocet a vykreslenie hrany
				Vertex* startVertex = cubeVertices[startIdx];
				Vertex* endVertex = cubeVertices[endIdx];

				QPoint startPoint(startVertex->x + width() / 2, startVertex->y + height() / 2);
				QPoint endPoint(endVertex->x + width() / 2, endVertex->y + height() / 2);

				drawLine(startPoint, endPoint, Qt::black, algType);
			}
		}

		break;

	case isSphere:
		// Iteracia cez steny sfery
		for (Face* f : sphereFaces) {
			if (!f || !f->edge || !f->edge->vert_origin || !f->edge->edge_next || !f->edge->edge_prev) {
				continue;
			}

			// Vypocet projekcii troch bodov steny
			QVector3D p1o(f->edge->vert_origin->x, f->edge->vert_origin->y, f->edge->vert_origin->z);
			QVector3D p2o(f->edge->edge_next->vert_origin->x, f->edge->edge_next->vert_origin->y, f->edge->edge_next->vert_origin->z);
			QVector3D p3o(f->edge->edge_prev->vert_origin->x, f->edge->edge_prev->vert_origin->y, f->edge->edge_prev->vert_origin->z);

			QVector3D p1(QVector3D::dotProduct(p1o, v), QVector3D::dotProduct(p1o, u), QVector3D::dotProduct(p1o, n));
			QVector3D p2(QVector3D::dotProduct(p2o, v), QVector3D::dotProduct(p2o, u), QVector3D::dotProduct(p2o, n));
			QVector3D p3(QVector3D::dotProduct(p3o, v), QVector3D::dotProduct(p3o, u), QVector3D::dotProduct(p3o, n));

			// Vytvorenie trojuholnika pre Z-buffer a rasterizaciu
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

		// Vykreslenie droteneho modelu sfery
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
				QPoint point1 = projectPoint(edge->vert_origin, distance); // Projekcia prveho vrcholu
				QPoint point2 = projectPoint(edge->edge_next->vert_origin, distance); // Projekcia druheho vrcholu

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

			// Vypocet projekcii troch bodov steny
			QVector3D p1o(f->edge->vert_origin->x, f->edge->vert_origin->y, f->edge->vert_origin->z);
			QVector3D p2o(f->edge->edge_next->vert_origin->x, f->edge->edge_next->vert_origin->y, f->edge->edge_next->vert_origin->z);
			QVector3D p3o(f->edge->edge_prev->vert_origin->x, f->edge->edge_prev->vert_origin->y, f->edge->edge_prev->vert_origin->z);

			QVector3D p1(QVector3D::dotProduct(p1o, v), QVector3D::dotProduct(p1o, u), QVector3D::dotProduct(p1o, n));
			QVector3D p2(QVector3D::dotProduct(p2o, v), QVector3D::dotProduct(p2o, u), QVector3D::dotProduct(p2o, n));
			QVector3D p3(QVector3D::dotProduct(p3o, v), QVector3D::dotProduct(p3o, u), QVector3D::dotProduct(p3o, n));

			// Vytvorenie trojuholnika pre Z-buffer a rasterizaciu
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

			// Vypocet projekcii troch bodov steny
			QVector3D p1o(f->edge->vert_origin->x, f->edge->vert_origin->y, f->edge->vert_origin->z);
			QVector3D p2o(f->edge->edge_next->vert_origin->x, f->edge->edge_next->vert_origin->y, f->edge->edge_next->vert_origin->z);
			QVector3D p3o(f->edge->edge_prev->vert_origin->x, f->edge->edge_prev->vert_origin->y, f->edge->edge_prev->vert_origin->z);

			QVector3D p1(QVector3D::dotProduct(p1o, v), QVector3D::dotProduct(p1o, u), QVector3D::dotProduct(p1o, n));
			QVector3D p2(QVector3D::dotProduct(p2o, v), QVector3D::dotProduct(p2o, u), QVector3D::dotProduct(p2o, n));
			QVector3D p3(QVector3D::dotProduct(p3o, v), QVector3D::dotProduct(p3o, u), QVector3D::dotProduct(p3o, n));

			// Vytvorenie trojuholnika pre Z-buffer a rasterizaciu
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
	// Skontroluje, ci bod nie je prilis blizko alebo za kamerou voci zadanej vzdialenosti
	if (vertex->z <= -distance) {
		return QPoint(); // Vrati prazdny bod, ak je bod za kamerou
	}

	// Vypocet projekcie bodu v 3D na x-ovu suradnicu v 2D
	int xPrime = static_cast<int>((distance * vertex->x) / (distance - vertex->z));
	// Vypocet projekcie bodu v 3D na y-ovu suradnicu v 2D
	int yPrime = static_cast<int>((distance * vertex->y) / (distance - vertex->z));

	// Pridanie posunu, aby bol stred obrazovky v bode (0, 0)
	int translatedX = xPrime + width() / 2;
	int translatedY = yPrime + height() / 2;

	return QPoint(translatedX, translatedY);
}


void ViewerWidget::rasterizeForFilling(Lighting prime, int cameraDistance, QVector<QVector<QColor>>& F, QVector<QPoint> T, QVector<QVector3D> p, int shadingType, int h) {
	// Triedenie bodov podla y, a ak su y rovnake, potom podla x
	std::sort(T.begin(), T.end(), [](const QPoint& a, const QPoint& b) {
		return a.y() == b.y() ? a.x() < b.x() : a.y() < b.y();
		});
	// Triedenie 3D bodov rovnakym sposobom
	std::sort(p.begin(), p.end(), [](const QVector3D& a, const QVector3D& b) {
		return a.y() == b.y() ? a.x() < b.x() : a.y() < b.y();
		});

	// Priradenie zoradenych bodov do premennych
	QPoint p0 = T.at(0);
	QPoint p1 = T.at(1);
	QPoint p2 = T.at(2);

	// Funkcia na vypocet sklonu priamky
	auto slope = [](const QPoint& a, const QPoint& b) -> double {
		if (b.x() == a.x()) return std::numeric_limits<double>::max();
		return static_cast<double>(b.y() - a.y()) / static_cast<double>(b.x() - a.x());
		};

	// Rasterizacia podla polohy bodov
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

		// Reset bodov na povodne pozicie pre druhu cast rasterizacie
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
	// Inicializacia kamery a zdroja svetla
	QVector3D camera(200, 200, cameraDistance);
	QVector3D source(prime.source.x, prime.source.y, prime.source.z);
	QColor color;
	QVector<QColor> col; // Pole pre farby pri Phongovom modeli osvetlenia
	QVector3D center; // Centrum objektu pre vypocet osvetlenia (konstantne tienovanie)

	// Nacitanie 3D pozicii vrcholov
	QVector3D p0(T[0].x(), T[0].y(), p.at(0).z());
	QVector3D p1(T[1].x(), T[1].y(), p.at(1).z());
	QVector3D p2(T[2].x(), T[2].y(), p.at(2).z());

	if (shadingType == 0) {
		// Vypocet osvetlenia pre kazdy vrchol
		col.append(phongModel(p0, prime, camera, source, h));
		col.append(phongModel(p1, prime, camera, source, h));
		col.append(phongModel(p2, prime, camera, source, h));
	}
	else {
		QVector3D center((p0 + p1 + p2) / 3.0); // Vypocet stredu objektu pre jednoduchsie osvetlenie
		color = phongModel(center, prime, camera, source, h);
	}

	// Iteracia cez scan-line od ymin do ymax
	for (int y = ymin; y < ymax; y++) {
		if (x1 != x2) {
			int start = floor(x1);
			int end = floor(x2 + 1);
			for (int x = start; x < end; x++) {
				double z = interpolateZ(x, y, T, p); // Interpolacia hlbky z

				if (shadingType == 0) {
					color = interpolateColor(x, y, T, p, col);
				}

				// Overenie, ci je bod vnutorne k objektu
				if (isInside(x, y)) {
					if (Z[x][y] < z) {
						Z[x][y] = z;
						setPixel(x, y, color);
					}
				}
			}
		}
		x1 += 1. / m1; // Posun x1 podla sklonu m1
		x2 += 1. / m2; // Posun x2 podla sklonu m2
	}
}

QColor ViewerWidget::phongModel(const QVector3D& p, Lighting prime, QVector3D camera, QVector3D source, int h) {
	// Vektor pohladu, normalizovany smer od bodu p k kamere
	QVector3D view = (camera - p).normalized();
	// Normalovy vektor bodu, normalizovany
	QVector3D normal = p.normalized();
	// Svetelny vektor, normalizovany smer od zdroja svetla k bodu p
	QVector3D light = (p - source).normalized();
	// Vektor odrazenia svetla
	QVector3D reflection = (2 * QVector3D::dotProduct(light, normal) * normal - light).normalized();

	//qDebug() << "Normal vector: " << normal;
	//qDebug() << "Light vector: " << light;
	//qDebug() << "Source position: " << source;
	//qDebug() << "Point position: " << p;

	// Vypocet zrkadloveho komponentu svetla
	double dot = pow(QVector3D::dotProduct(view, reflection), h);
	//qDebug() << "Phong dot value: " << dot << "\n";

	//qDebug() << "Phong prime.source: R:" << prime.source.r << ", G: " << prime.source.g << ", B: " << prime.source.b << "\n";
	//qDebug() << "Phong prime.reflection: coeffR:" << prime.reflection.coeffR << ", coeffG: " << prime.reflection.coeffG << ", coeffB: " << prime.reflection.coeffB << "\n";
	//qDebug() << "Phong prime.diffusion: R:" << prime.diffusion.coeffR << ", G: " << prime.diffusion.coeffG << ", B: " << prime.diffusion.coeffB << "\n";
	//qDebug() << "Phong prime.ambient: R:" << prime.ambient.coeffR << ", G: " << prime.ambient.coeffG << ", B: " << prime.ambient.coeffB << "\n";

	// Vypocet farebneho zlozenia odrazenia zdroja svetla
	int rs = prime.source.r * prime.reflection.coeffR * dot;
	int gs = prime.source.g * prime.reflection.coeffG * dot;
	int bs = prime.source.b * prime.reflection.coeffB * dot;

	// Vypocet difuzneho komponentu svetla
	dot = QVector3D::dotProduct(light, normal);
	//qDebug() << "Phong dot value: " << dot << "\n";
	int rd = prime.source.r * prime.diffusion.coeffR * dot;
	int gd = prime.source.g * prime.diffusion.coeffG * dot;
	int bd = prime.source.b * prime.diffusion.coeffB * dot;

	// Vypocet ambientneho svetla
	int ra = prime.ambient.r * prime.ambient.coeffR;
	int ga = prime.ambient.g * prime.ambient.coeffG;
	int ba = prime.ambient.b * prime.ambient.coeffB;

	//qDebug() << "Phong farby source: R:" << rs << ", G: " << gs << ", B: " << bs << "\n";
	//qDebug() << "Phong farby diff: R:" << rd << ", G: " << gd << ", B: " << bd << "\n";
	//qDebug() << "Phong farby ambient: R:" << ra << ", G: " << ga << ", B: " << ba << "\n";

	// Sumarizacia komponentov svetla pre kazdu farebnu zlozku
	int r = rs + rd + ra;
	int g = gs + gd + ga;
	int b = bs + bd + ba;

	//qDebug() << "Phong Farby: R:" << r << ", G: " << g << ", B: " << b << "\n";

	// Orezanie hodnot na maximum 255, co je maximalna hodnota pre farebne zlozky v RGB
	r = std::min(r, 255);
	g = std::min(g, 255);
	b = std::min(b, 255);

	// Orezanie hodnot na minimum 0, co je minimalna hodnota pre farebne zlozky v RGB
	r = std::max(0, r);
	g = std::max(0, g);
	b = std::max(0, b);

	return QColor(r, g, b);
}

double ViewerWidget::interpolateZ(const double& x, const double& y, QVector<QPoint> T, QVector<QVector3D> p) {
	// Vypocet plochy trojuholnika pomocou determinantu
	double A = static_cast<double>(abs((T[1].x() - T[0].x()) * (T[2].y() - T[0].y()) - (T[1].y() - T[0].y()) * (T[2].x() - T[0].x())));

	// Vypocet barycentrickych suradnic lambda pre bod
	double lambda0 = static_cast<double>(abs((T[1].x() - x) * (T[2].y() - y) - (T[1].y() - y) * (T[2].x() - x))) / A;
	double lambda1 = static_cast<double>(abs((T[0].x() - x) * (T[2].y() - y) - (T[0].y() - y) * (T[2].x() - x))) / A;
	double lambda2 = 1 - lambda0 - lambda1;

	return (lambda0 * p[0].z() + lambda1 * p[1].z() + lambda2 * p[2].z());
}

QColor ViewerWidget::interpolateColor(const double& x, const double& y, QVector<QPoint> T, QVector<QVector3D> p, QVector<QColor> col) {
	// Vypocet plochy trojuholnika pomocou determinantu
	double A = static_cast<double>(abs((T[1].x() - T[0].x()) * (T[2].y() - T[0].y()) - (T[1].y() - T[0].y()) * (T[2].x() - T[0].x())));
	
	// Vypocet barycentrickych suradnic pre bod
	double lambda0 = static_cast<double>(abs((T[1].x() - x) * (T[2].y() - y) - (T[1].y() - y) * (T[2].x() - x))) / A;
	double lambda1 = static_cast<double>(abs((T[0].x() - x) * (T[2].y() - y) - (T[0].y() - y) * (T[2].x() - x))) / A;
	double lambda2 = 1 - lambda0 - lambda1;

	// Vypocet interpolovanej farby pre bod x, y na zaklade barycentrickych suradnic
	int r = lambda0 * col[0].red() + lambda1 * col[1].red() + lambda2 * col[2].red();
	int g = lambda0 * col[0].green() + lambda1 * col[1].green() + lambda2 * col[2].green();
	int b = lambda0 * col[0].blue() + lambda1 * col[1].blue() + lambda2 * col[2].blue();

	return QColor(r, g, b);
}
