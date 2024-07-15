#pragma once

#define isCube 0
#define isSphere 1
#define isCubeWireFrame 2
#define isSphereWireFrame 3

//	<< Half-Edges >>
class H_edge;

class Vertex {
public:
	double x, y, z;
	H_edge* edge;

	Vertex() : x(0), y(0), z(0), edge(nullptr) {};
	Vertex(QVector3D v, H_edge* e = nullptr) : x(v.x()), y(v.y()), z(v.z()), edge(e) {};
	Vertex(double x, double y, double z) : x(x), y(y), z(z), edge(nullptr) {};

};

class Face {
public:
	H_edge* edge;

	Face() : edge(nullptr) {}
	Face(H_edge* e) : edge(e) {};
};

class H_edge {
public:
	Vertex* vert_origin;
	Face* face;
	H_edge* edge_prev, * edge_next;
	H_edge* pair;

	H_edge() : vert_origin(nullptr), face(nullptr), edge_prev(nullptr), edge_next(nullptr), pair(nullptr) {};
	H_edge(Vertex* v, Face* f, H_edge* ep = nullptr, H_edge* en = nullptr, H_edge* p = nullptr) :vert_origin(v), face(f), edge_prev(ep), edge_next(en), pair(p) {};
};