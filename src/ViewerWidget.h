#pragma once
#include <QtWidgets>
#include <QtMath>
#include <QSet>
#include <QVector3D>
#include "lighting.h"
#include "representation.h"

struct ClippedLine {
	QVector<QPoint> points;
	bool isClipped = false;
};

class ViewerWidget :public QWidget {
	Q_OBJECT
private:
	QSize areaSize = QSize(0, 0);
	QImage* img = nullptr;
	QPainter* painter = nullptr;
	uchar* data = nullptr;

	bool drawLineActivated = false;
	bool drawCircleActivated = false;
	bool drawPolygonActivated = false;
	QPoint drawLineBegin = QPoint(0, 0);
	QPoint drawCircleCenter = QPoint(0, 0);
	QPoint moveStart = QPoint(0, 0);
	QVector<QPoint> pointsVector;
	QVector<QPoint> originalPointsVector;
	QVector<QPoint> linePoints;
	QVector<QPoint> curvePoints;
	QVector<QPoint> tangents;

	double prevTheta = 0.0;
	double prevPhi = 0.0;
	QVector<QVector<double>> Z;
	QVector<Vertex*> cubeVertices;
	QVector<Face*> cubeFaces;
	QVector<H_edge*> cubeEdges;
	QVector<Vertex*> sphereVertices;
	QVector<H_edge*> sphereEdges;
	QVector<Face*> sphereFaces;
	QVector3D n, u, v;

public:
	ViewerWidget(QSize imgSize, QWidget* parent = Q_NULLPTR);
	~ViewerWidget();
	void resizeWidget(QSize size);

	//Image functions
	bool setImage(const QImage& inputImg);
	QImage* getImage() { return img; };
	QPainter* getPainter() { return painter; }
	bool isEmpty();
	bool changeSize(int width, int height);

	void setPixel(int x, int y, uchar r, uchar g, uchar b, uchar a = 255);
	void setPixel(int x, int y, double valR, double valG, double valB, double valA = 1.);
	void setPixel(int x, int y, const QColor& color);
	void setPixel(int x, int y, const QColor& color, bool isCurve);
	bool isInside(QPoint point) { return (point.x() > 0 && point.y() > 0 && point.x() < img->width() - 1 && point.y() < img->height() - 1) ? true : false; }
	bool isInside(int x, int y) { return (x > 0 && y > 0 && x < img->width() && y < img->height()) ? true : false; }

	//Draw functions

	//	Lines
	void drawLine(QPoint start, QPoint end, QColor color, int algType = 0);
	void setDrawLineBegin(QPoint begin) { drawLineBegin = begin; }
	QPoint getDrawLineBegin() { return drawLineBegin; }
	void setDrawLineActivated(bool state) { drawLineActivated = state; }
	bool getDrawLineActivated() { return drawLineActivated; }
	void drawLineDDA(QVector<QPoint>& linePoints);
	void drawLineBresenham(QVector<QPoint>& linePoints);
	void updateLine(const QColor& color, int algType);
	
	//	Circles
	void drawCircle(QPoint center, QPoint radiusPoint, QColor color);
	void drawSymmetricPoints(const QPoint& center, int x, int y);
	void setDrawCircleActivated(bool state) { drawCircleActivated = state; }
	bool getDrawCircleActivated() { return drawCircleActivated; }
	void setDrawCircleCenter(QPoint center) { drawCircleCenter = center; }
	QPoint getDrawCircleCenter() { return drawCircleCenter; }

	// Polygons
	void drawPolygon(const QColor& color, const QColor& color1, const QColor& color2, int algType, int interpolType);
	void addToVectorOfPoints(QPoint currentPoint) { pointsVector.append(currentPoint); }
	QVector<QPoint> getVectorOfPoints() { return pointsVector; }
	void clearVectorOfPoints() { pointsVector.clear(); }
	void updatePolygon(const QColor& color, const QColor& color1, const QColor& color2, int algType, int interpolType);

	// Actions
	void setMoveStart(QPoint start) { moveStart = start; };
	QPoint getMoveStart() { return moveStart; }
	void moveLine(const QColor& color, const QPoint& offset, int algType);
	void turnLine(const QColor& color, int algType, int angle);
	void movePolygon(const QColor& color, const QColor& color1, const QColor& color2, const QPoint& offset, int algType, int interpolType);
	void turnPolygon(const QColor& color, const QColor& color1, const QColor& color2, int algType, int angle, int interpolType);
	QPoint getLineCenter() const;
	void scaleLine(const QColor& color, int algType, double scaleX, double scaleY);
	QPoint getPolygonCenter() const;
	void scalePolygon(const QColor& color, const QColor& color1, const QColor& color2, int algType, double scaleX, double scaleY, int interpolType);
	void axialSymmetry(const QColor& color, const QColor& color1, const QColor& color2, int algType, int object, int interpolType, int curveType);
	void saveOriginalState(int object);
	void restoreOriginalState(const QColor& color, const QColor& color1, const QColor& color2, int algType, int object, int interpolType, int curveType);
	void shearObject(const QColor& color, const QColor& color1, const QColor& color2, int algType, double coefficient, int interpolType);
	
	//  **Trimming functions**
	QVector<QPoint> trimPolygon();
	void clipLineWithPolygon(QVector<QPoint> linePoints);

	//	**Polygon filling handling**

	//	<Subclass for edges>
	class Edge {
	private:
		QPoint startPoint_;
		QPoint endPoint_;
		double slope_;
		double x_;
		double w_;

	public:
		Edge() : startPoint_(QPoint(0, 0)), endPoint_(QPoint(0, 0)), slope_(0.0), x_(0.0), w_(0.0) {}
		Edge(QPoint start, QPoint end) : startPoint_(start), endPoint_(end), x_(0.0), w_(0.0) {
			calculateAttributes();
		}

		void calculateAttributes() {
			double dx = static_cast<double>(endPoint_.x() - startPoint_.x());
			double dy = static_cast<double>(endPoint_.y() - startPoint_.y());

			if (dx == 0) {
				slope_ = std::numeric_limits<double>::max();
				w_ = 0;
			}
			else {
				slope_ = dy / dx;
				w_ = 1.0 / slope_;
			}

			x_ = static_cast<double>(startPoint_.x());

			if (startPoint_.y() > endPoint_.y()) {
				swapStartEndPoints();
				calculateAttributes();
			}
		}

		void swapStartEndPoints() {
			std::swap(startPoint_, endPoint_);
		}

		void adjustEndPoint() {
			endPoint_.setY(endPoint_.y() - 1);
		}

		QPoint startPoint() const { return startPoint_; }
		QPoint endPoint() const { return endPoint_; }
		double slope() const { return slope_; }
		double x() const { return x_; }
		double w() const { return w_; }

		void setX(double x) { x_ = x; }
	};

	static bool compareByY(const Edge& edge1, const Edge& edge2){ return edge1.startPoint().y() < edge2.startPoint().y(); }
	static bool compareByX(const Edge& edge1, const Edge& edge2){ return edge1.x() < edge2.x(); }
	
	void fillPolygon(QVector<QPoint>& points, const QColor& color, const QColor& color1, const QColor& color2, int interpolType);
	QVector<Edge> loadEdges(QVector<QPoint>& points);
	void fillTriangle(QVector<QPoint>& points, const QColor& color, const QColor& color1, const QColor& color2, int interpolType);
	void fillFlatBottomTriangle(const QPoint& T0, const QPoint& T1, const QPoint& T2, const QColor& color, const QColor& color1, const QColor& color2, int interpolType);
	void fillFlatTopTriangle(const QPoint& T0, const QPoint& T1, const QPoint& T2, const QColor& color, const QColor& color1, const QColor& color2, int interpolType);

	QColor interpolateColorBarycentric(const QPoint& P, const QPoint& T0, const QPoint& T1, const QPoint& T2, const QColor& color0, const QColor& color1, const QColor& color2);
	bool isPointInsideTriangle(const QPoint& P, const QPoint& A, const QPoint& B, const QPoint& C);

	//	** Curve function declarations **
	void addToVectorOfCurvePoints(QPoint currentPoint) { curvePoints.append(currentPoint); }
	void clearVectorOfCurvePoints() { curvePoints.clear(); }
	void drawCurve(const QColor& color, const QColor& color2, int curveType, int algType);
	void drawTangentVectors(const QColor& color, int algType);
	void calculateTangents();
	void clearVectorOfTangents() { tangents.clear(); }

	void moveCurve(const QColor& color, const QColor& color2, const QPoint& offset, int algType, int curveType, bool isCurve);
	void updateCurve(const QColor& color, const QColor& color2, const QPoint& offset, int algType, int curveType, bool isCurve);
	void scaleCurve(const QColor& color, const QColor& color2, int algType, double scaleX, double scaleY, int curveType, bool isCurve);
	void turnCurve(const QColor& color, const QColor& color2, int algType, int angle, int curveType, bool isCurve);
	QPoint calculateCurveCenter();
	QPoint calculateTheNearestPoint(QPoint position);
	void moveTheNearestPoint(const QColor& color, const QColor& color2, const QPoint& offset, int curveType, int algType);


	void saveVTKCube(int sideLength);
	void saveSphereVTK(int parallels, int meridians, double radius);

	void loadCubeVTK();
	void loadSphereVTK();

	void adjustProjection(double theta, double phi, bool objectIsLoaded, int objectType, int algType, int projectionType, double distance, int cameraDistance, Lighting prime);
	double mapSliderValueToDegrees(int value, int min, int max);
	QVector3D computeCenter(const QVector<Vertex*>& vertices);

	void renderParallelProjection(double theta, double phi, bool objectIsLoaded,int algType, int objectType, int projectionType, double distance, int cameraDistance, Lighting prime, int shadingType, int h);
	void renderPerspectiveProjection(double theta, double phi, bool objectIsLoaded, int algType, int objectType, int projectionType, double distance, int cameraDistance, Lighting prime, int shadingType, int h);
	void rasterizeForFilling(Lighting prime, int cameraDistance, QVector<QVector<QColor>>& F, QVector<QPoint> T, QVector<QVector3D> p, int shadingType, int h);
	void fillObject3D(const Lighting& prime, int cameraDistance,
		double x1, double x2, double m1, double m2, int ymin, int ymax,
		const QVector<QPoint>& T, const QVector<QVector3D>& p, int shadingType, int h);
	QColor phongModel(const QVector3D& p, Lighting prime, QVector3D camera, QVector3D source, int h);
	double interpolateZ(const double& x, const double& y, QVector<QPoint> T, QVector<QVector3D> p);
	QColor interpolateColor(const double& x, const double& y, QVector<QPoint> T, QVector<QVector3D> p, QVector<QColor> col);

	QPoint projectPoint(const Vertex* vertex, double distance);

	//Get/Set functions
	uchar* getData() { return data; }
	void setDataPtr() { data = img->bits(); }
	void setPainter() { painter = new QPainter(img); }

	int getImgWidth() { return img->width(); };
	int getImgHeight() { return img->height(); };

	void clear();

public slots:
	void paintEvent(QPaintEvent* event) Q_DECL_OVERRIDE;
};
