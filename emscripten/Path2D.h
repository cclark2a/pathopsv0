// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef Path2D_DEFINED
#define Path2D_DEFINED

#include "curves/BinaryWinding.h"
#include "curves/UnaryWinding.h"
#include "PathOps.h"

#define ARC_SUPPORT 0

namespace TwoD {

enum class Types {
    move,
    line,
    quad,
    cubic,
	close
};

enum class Ops {
	sect,
	_union,
	diff,
	revDiff,
	_xor
};

struct Curve {
	Types type;
	std::vector<float> data;
};

struct Path {
	void addPath(Path& path);  // !!! need to add transform
	void eraseRange(int start, int end);
	void insertPath(int index, Path& path);
	void clear() { curves.clear(); }
	int curveCount() { return (int) curves.size(); }
	Curve getCurve(int index, bool includeFirstPt);
	void setCurve(int index, Curve& );
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
	void transform(float a, float b, float c, float d, float e, float f);
#if ARC_SUPPORT // these require a v0 arc library in curves; or arc to conic to arc conversion code
	void arcTo(float x1, float y1, float x2, float y2, float radius);
	void arc(float x, float y, float radius, float startAngle, float endAngle, bool ccw);
	void ellipse(float x, float y, float radiusX, float radiusY, float rotation, 
			float startAngle, float endAngle, bool ccw);
	void roundRect(float x, float y, float width, float height, float radii);
#endif
	void fromCommands(std::vector<Curve>& curves);
	void fromSVG(std::string s);
	std::vector<Curve> toCommands();
	std::string toSVG();
	// internal
	void opAddPath(PathOpsV0Lib::Context* context, PathOpsV0Lib::AddWinding winding, bool closeLoops);
	void commonOutput(PathOpsV0Lib::Curve c, Types type, bool firstPt, bool lastPt, 
			PathOpsV0Lib::PathOutput output);
	OpPoint lastPt(int index = OpMax);
#if ARC_SUPPORT  // a utility like this may be needed for ellipse
	void rotate(float angle);
#endif

	static constexpr std::array<int, 5> sizes { 2, 2, 4, 6, 0 };
	std::vector<Curve> curves;
};

struct FillPath : Path {
	FillPath clone() { FillPath result; result = *this; return result; }
	void difference(FillPath& path) { opCommon(path, Ops::diff); }
	void intersect(FillPath& path) { opCommon(path, Ops::sect); }
	void reverseDifference(FillPath& path) { opCommon(path, Ops::revDiff); }
	void _union(FillPath& path) { opCommon(path, Ops::_union); }
	void _xor(FillPath& path) { opCommon(path, Ops::_xor); }
	void simplify();
	// internal
	void opCommon(FillPath& path, Ops oper);
};

struct FramePath : Path {
	Path clone() { FramePath result; result = *this; return result; }
	void difference(FillPath& path) { opCommon(path, Ops::diff); }
	void intersect(FillPath& path) { opCommon(path, Ops::sect); }
	// internal
	void opCommon(FillPath& path, Ops oper);
};

}

#endif
