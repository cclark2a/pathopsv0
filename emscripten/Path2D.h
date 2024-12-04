// (c) 2024, Cary Clark cclark2@gmail.com
#ifndef Path2D_DEFINED
#define Path2D_DEFINED

#include "curves/BinaryWinding.h"
#include "PathOps.h"

namespace TwoD {

enum class Types {
    move,
    line,
    quad,
    conic,
    cubic,
	close
};

enum class Ops {
	intersect,
	_union,
	diff,
	exor
};

struct Path {
	void moveTo(float x, float y);
	void lineTo(float x, float y);
	void quadraticCurveTo(float cx, float cy, float x, float y);
	void closePath();
	Path* fromCommands(std::string s);
	Path* fromSVG(std::string s);
	std::string toCommands();
	std::string toSVG();
	// internal
	static char* skipSpace(char* ch, const char* chEnd);
	static float nextFloat(char** chPtr, const char* chEnd);
	void addPath(PathOpsV0Lib::Context* context, PathOpsV0Lib::AddWinding winding);
	static PathOpsV0Lib::Contour* GetContour(PathOpsV0Lib::Context* context, 
			PathOpsV0Lib::BinaryOperand operand, 
			PathOpsV0Lib::BinaryWindType windType, Ops ops);
	void commonOutput(PathOpsV0Lib::Curve c, Types type, bool firstPt, bool lastPt, 
			PathOpsV0Lib::PathOutput output);
	void reset();

	std::vector<float> points;
	std::vector<Types> types;
};

struct PathOps {
	static Path Intersect(Path& path1, Path& path2);
	// internal
	static Path Op(Path& path1, Path& path2, Ops oper);
};

}

#endif
