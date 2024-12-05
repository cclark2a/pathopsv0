// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef Path2D_DEFINED
#define Path2D_DEFINED

#include "curves/BinaryWinding.h"
#include "curves/UnaryWinding.h"
#include "PathOps.h"

namespace TwoD {

enum class Types {
    move,
    line,
    quad,
//    conic,
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

enum class Format {
	command,
	svg
};

struct Path {
	void addPath(Path& path);  // !!! need to add transform
	void moveTo(float x, float y);
	void lineTo(float x, float y);
	void quadraticCurveTo(float cx, float cy, float x, float y);
	void bezierCurveTo(float c1x, float c1y, float c2x, float c2y, float x, float y);
	void closePath();
	void rect(float x, float y, float width, float height);
#define CONIC_SUPPORT 0
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
	Path* fromCommands(std::string s) { return fromCommon(s, Format::command); }
	Path* fromSVG(std::string s) { return fromCommon(s, Format::svg); }
	std::string toCommands() { return toCommon(Format::command); }
	std::string toSVG() { return toCommon(Format::svg); }
	// internal
	Path* fromCommon(std::string s, Format );
	std::string toCommon(Format );
	void opCommon(Path& path, Ops oper);
	void addPath(PathOpsV0Lib::Context* context, PathOpsV0Lib::AddWinding winding);
	void commonOutput(PathOpsV0Lib::Curve c, Types type, bool firstPt, bool lastPt, 
			PathOpsV0Lib::PathOutput output);
	void reset();
#if CONIC_SUPPORT  // a utility like this may be neede for ellipse
	void rotate(float angle);
#endif

	std::vector<float> points;  // and conic weights
	std::vector<Types> types;
};

}

#endif
