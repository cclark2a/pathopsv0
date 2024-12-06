// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef Path2D_DEFINED
#define Path2D_DEFINED

#include "curves/BinaryWinding.h"
#include "curves/UnaryWinding.h"
#include "PathOps.h"

#define CONIC_SUPPORT 0

namespace TwoD {

enum class Types {
    move,
    line,
    quad,
#if CONIC_SUPPORT    
	conic,
#endif
    cubic,
	close,
	lastPt,  // denotes that line is added from current point to next point
};

enum class Ops {
	sect,
	_union,
	diff,
	revDiff,
	_xor
};

struct Curve {
	std::string command;
	std::vector<float> data;
};

struct Path {
	void addPath(Path& path);  // !!! need to add transform
	Path clone() { Path result; result = *this; return result; }
	int curveCount() { return (int) types.size(); }
	Curve getCurve(int index, bool includeFirstPt);
	void moveTo(float x, float y);
	void rMoveTo(float dx, float dy);
	void lineTo(float x, float y);
	void rLineTo(float dx, float dy);
	void quadraticCurveTo(float cx, float cy, float x, float y);
	void rQuadraticCurveTo(float dcx, float dcy, float dx, float dy);
	void bezierCurveTo(float c1x, float c1y, float c2x, float c2y, float x, float y);
	void rBezierCurveTo(float dc1x, float dc1y, float dc2x, float dc2y, float dx, float dy);
	void closePath();
	void rect(float x, float y, float width, float height);
#if CONIC_SUPPORT // these require a v0 arc library in curves; or arc to conic to arc conversion code
	void arcTo(float x1, float y1, float x2, float y2, float radius);
	void arc(float x, float y, float radius, float startAngle, float endAngle, bool ccw);
	void ellipse(float x, float y, float radiusX, float radiusY, float rotation, 
			float startAngle, float endAngle, bool ccw);
	void roundRect(float x, float y, float width, float height, float radii);
#endif
	void difference(Path& path) { opCommon(path, Ops::diff); }
	void intersect(Path& path) { opCommon(path, Ops::sect); }
	void reverseDifference(Path& path) { opCommon(path, Ops::revDiff); }
	void _union(Path& path) { opCommon(path, Ops::_union); }
	void _xor(Path& path) { opCommon(path, Ops::_xor); }
	void simplify();
	void fromCommands(std::vector<Curve>& curves);
	void fromSVG(std::string s);
	std::vector<Curve> toCommands();
	std::string toSVG();
	// internal
	void opCommon(Path& path, Ops oper);
	void addPath(PathOpsV0Lib::Context* context, PathOpsV0Lib::AddWinding winding);
	void commonOutput(PathOpsV0Lib::Curve c, Types type, bool firstPt, bool lastPt, 
			PathOpsV0Lib::PathOutput output);
	OpPoint lastPt();
	void reset();
#if CONIC_SUPPORT  // a utility like this may be needed for ellipse
	void rotate(float angle);
#endif

	std::vector<float> points;  // and conic weights? 
	std::vector<Types> types;
	std::vector<int> offsets;  // cache of offset into points for get data
};

}

#endif
