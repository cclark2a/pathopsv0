// (c) 2024, Cary Clark cclark2@gmail.com

#include "Path2D.h"
#include "curves/Line.h"
#include "curves/NoCurve.h"
#include "curves/CubicBezier.h"
#include "curves/QuadBezier.h"
#if 0 // emscripten as of 12/7/2024 does not support std::from_chars
#include <charconv>
#else
#include <cstdlib>
#endif

namespace TwoD {

void Path::addPath(Path& path) {
	insertPath(curveCount(), path);
}

void Path::insertPath(int index, Path& path) {
	index = std::min(std::max(0, index), curveCount());
	curves.insert(curves.begin() + index, path.curves.begin(), path.curves.end());
}

void Path::eraseRange(int start, int end) {
	start = std::min(std::max(0, start), curveCount());
	end = std::min(std::max(0, end), curveCount());
	if (start > end)
		return;
	curves.erase(curves.begin() + start, curves.begin() + end);
}

Curve Path::getCurve(int index, bool includeFirstPt) {
	if ((size_t) index >= curves.size())
		return Curve();
	Curve result = curves[index];
	if (includeFirstPt) {
		OpPoint last = lastPt(index - 1);
		result.data.insert(result.data.begin(), { last.x, last.y } );
	}		
	return result;
}

void Path::setCurve(int index, Curve& curve) {
	if ((size_t) index >= curves.size()) {
		curves.push_back(curve);
		return;
	}
	curves[index] = curve;
}

std::vector<Curve> Path::toCommands() {
	return curves;
}

void Path::fromCommands(std::vector<Curve>& curveData) {
	curves = curveData;
}

void Path::moveTo(float x, float y) {
	curves.push_back({ Types::move, { x, y }} );
}

OpPoint Path::lastPt(int index) {
	index = std::min(std::max(0, index), curveCount() - 1);
	OpPoint last { 0, 0 };
	if (index < 0)
		return last;
	Curve* lastCurve = &curves.front() + index;
	do {
		if (Types::close != lastCurve->type) {
			last = { *(&lastCurve->data.back() - 1), lastCurve->data.back() } ;
			break;
		}
	} while (lastCurve-- != &curves.front());
	return last;
}

void Path::rMoveTo(float dx, float dy) {
	OpPoint last = lastPt();
	moveTo(last.x + dx, last.y + dy);
}

void Path::lineTo(float x, float y) {
	curves.push_back({ Types::line, { x, y }} );
}

void Path::rLineTo(float dx, float dy) {
	OpPoint last = lastPt();
	lineTo(last.x + dx, last.y + dy);
}

void Path::quadraticCurveTo(float cx, float cy, float x, float y) {
	curves.push_back( { Types::quad, { cx, cy, x, y }} );
}

void Path::rQuadraticCurveTo(float dcx, float dcy, float dx, float dy) {
	OpPoint last = lastPt();
	quadraticCurveTo(last.x + dcx, last.y + dcy, last.x + dx, last.y + dy);
}

void Path::bezierCurveTo(float c1x, float c1y, float c2x, float c2y, float x, float y) {
	curves.push_back( { Types::cubic, { c1x, c1y, c2x, c2y, x, y }} );
}

void Path::rBezierCurveTo(float rc1x, float rc1y, float rc2x, float rc2y, float rx, float ry) {
	OpPoint last = lastPt();
	bezierCurveTo(last.x + rc1x, last.y + rc1y, last.x + rc2x, last.y + rc2y, last.x + rx, last.y + ry);
}

void Path::closePath() {
	curves.push_back( { Types::close, {}} );
}

void Path::rect(float x, float y, float width, float height) {
	moveTo(x, y);
	lineTo(x + width, y);
	lineTo(x + width, y + height);
	lineTo(x, y + height);
	lineTo(x, y);
	closePath();
}

void Path::transform(float scaleX, float skewY, float skewX, float scaleY, float dx, float dy) {
	OpVector scale { scaleX, scaleY };
	OpVector skew { skewX, skewY };
	OpVector move { dx, dy };
	for (Curve& curve : curves) {
		for (size_t index = 0; index < curve.data.size(); index += 2) {
			OpPoint& xy = *(OpPoint*) &curve.data[index];
			OpPoint yx = xy;
			std::swap(yx.x, yx.y);
			xy = xy * scale + yx * skew + move;
		}
	}
}

#if ARC_SUPPORT
!!! needs complete rewrite
void Path::rotate(float rotation) {
	if (0 == rotation)
		return;
	// incomplete
	OP_ASSERT(0);
}

void Path::ellipse(float x, float y, float rx, float ry, float rotation, 
			float startAngle, float endAngle, bool ccw) {
	Path circle;
	circle.types = { Types::move, Types::conic, Types::conic, Types::conic, Types::conic };
	float sqrt2over2 = sqrtf(2) / 2;
	// !!! consider putting weights in their own array in path so that last point is always
	//     last pair of values
	circle.points = { x + rx, y, x + rx, y + ry, x,      y + ry, sqrt2over2,
							   	 x - rx, y + ry, x - rx, y,      sqrt2over2,
								 x - rx, y - ry, x,      y - ry, sqrt2over2,
								 x + rx, y - ry, x + rx, y,      sqrt2over2 };
	float sweep = fabsf(startAngle - endAngle);
	if (sweep < OpPI * 2) {
		Path clip;
		clip.types = 
	}
	circle.rotate(rotation);
	types.push_back(Types::lastPt);  // add line from last point to circle start
	addPath(circle);
}
#endif

static char* skipSpace(char* ch, const char* chEnd) {
	while (ch < chEnd && ' ' >= *ch)
		ch++;
	return ch;
}

static float nextFloat(char** chPtr, const char* chEnd) {
	char* ch = *chPtr;
	if (ch >= chEnd)
		return OpNaN;
	ch = skipSpace(ch, chEnd);
	float result = OpNaN;
#if 0 
	std::from_chars_result chResult = std::from_chars(ch, chEnd, result);
	ch = const_cast<char*>(chResult.ptr);
#else
	char* endPtr;
	result = (float) strtod(ch, &endPtr);
	if (ch == endPtr)
		result = OpNaN;
	else
		ch = endPtr;
#endif
	*chPtr = skipSpace(ch, chEnd);
	if (OpMath::IsNaN(result))
		*chPtr = const_cast<char*>(chEnd);
	return result;
}

static float nextFloatComma(char** chPtr, const char* chEnd) {
	float result = nextFloat(chPtr, chEnd);
	if (*chPtr < chEnd && ',' == **chPtr)
		(*chPtr)++;
	*chPtr = skipSpace(*chPtr, chEnd);
	return result;
}

void Path::fromSVG(std::string s) {
	clear();
	size_t strLen = s.size();
	if (!strLen)
		return;
	char* ch = &s.front();
	const char* chEnd = ch + strLen;
	ch = skipSpace(ch, chEnd);
	auto skipCommaSpace = [&ch, chEnd]() {
		if (ch >= chEnd)
			return;
		ch = skipSpace(ch, chEnd);
		if (ch >= chEnd)
			return;
		if (',' == *ch)
			++ch;
		ch = skipSpace(ch, chEnd);
	};
	while (ch < chEnd) {
		char command = *ch++;
		bool relative = std::islower(command);
		command = std::toupper(command);
		do {
			switch (command) {
				case 'M': {
					skipCommaSpace();
					float x = nextFloatComma(&ch, chEnd);
					float y = nextFloatComma(&ch, chEnd);
					skipCommaSpace();
					relative ? rMoveTo(x, y) : moveTo(x, y);
					} break;
				case 'L': {
					skipCommaSpace();
					float x = nextFloatComma(&ch, chEnd);
					float y = nextFloatComma(&ch, chEnd);
					skipCommaSpace();
					relative ? rLineTo(x, y) : lineTo(x, y);
					} break;
				case 'H': {
					skipCommaSpace();
					float x = nextFloatComma(&ch, chEnd);
					skipCommaSpace();
					relative ? rLineTo(x, 0) : lineTo(x, lastPt().y);
					} break;
				case 'V': {
					skipCommaSpace();
					float y = nextFloatComma(&ch, chEnd);
					skipCommaSpace();
					relative ? rLineTo(0, y) : lineTo(lastPt().x, y);
					} break;
				case 'Q': {
					skipCommaSpace();
					float cx = nextFloatComma(&ch, chEnd);
					float cy = nextFloatComma(&ch, chEnd);
					float x = nextFloatComma(&ch, chEnd);
					float y = nextFloatComma(&ch, chEnd);
					skipCommaSpace();
					relative ? rQuadraticCurveTo(cx, cy, x, y) : quadraticCurveTo(cx, cy, x, y);
					} break;
				case 'T': {
					OpPoint lp = lastPt();
					OpPoint lc = lp;
					if (Types::quad == curves.back().type) {
						float* data = &curves.back().data.back();
						lc = { *(data - 3), *(data - 2) };
					}
					float x = nextFloatComma(&ch, chEnd);
					float y = nextFloatComma(&ch, chEnd);
					if (relative) {
						x += lp.x;
						y += lp.y;
					}
					lc.x = lp.x * 2 - lc.x;
					lc.y = lp.x * 2 - lc.y;
					skipCommaSpace();
					quadraticCurveTo(lc.x, lc.y, x, y);
					} break;
				case 'C': {
					skipCommaSpace();
					float c1x = nextFloatComma(&ch, chEnd);
					float c1y = nextFloatComma(&ch, chEnd);
					float c2x = nextFloatComma(&ch, chEnd);
					float c2y = nextFloatComma(&ch, chEnd);
					float x = nextFloatComma(&ch, chEnd);
					float y = nextFloatComma(&ch, chEnd);
					skipCommaSpace();
					relative ? rBezierCurveTo(c1x, c1y, c2x, c2y, x, y) 
							: bezierCurveTo(c1x, c1y, c2x, c2y, x, y);
					} break;
				case 'S': {
					OpPoint lp = lastPt();
					OpPoint lc = lp;
					if (Types::cubic == curves.back().type) {
						float* data = &curves.back().data.back();
						lc = { *(data - 3), *(data - 2) };
					}
					float c2x = nextFloatComma(&ch, chEnd);
					float c2y = nextFloatComma(&ch, chEnd);
					float x = nextFloatComma(&ch, chEnd);
					float y = nextFloatComma(&ch, chEnd);
					if (relative) {
						c2x += lp.x;
						c2x += lp.y;
						x += lp.x;
						y += lp.y;
					}
					lc.x = lp.x * 2 - lc.x;
					lc.y = lp.y * 2 - lc.y;
					skipCommaSpace();
					bezierCurveTo(lc.x, lc.y, c2x, c2y, x, y);
					} break;
				case 'Z':
					skipCommaSpace();
					closePath();
					break;
				default:
					OP_ASSERT(0);
					return;
			}
		} while (ch < chEnd && !std::isalpha(*ch));  // repeat last command
	}
}

std::string Path::toSVG() {
	std::string result;
	std::vector<float>* dataPtr = nullptr;
	auto ord = [&dataPtr](int index) {
		std::string s;
		s = std::to_string((*dataPtr)[index]);
		s.erase(s.find_last_not_of('0') + 1, std::string::npos); 
		s.erase(s.find_last_not_of('.') + 1, std::string::npos);
		s += " ";
		return s;
	};
	auto command = [](char cmd) {
		std::string s;
		s += cmd;
		s += " "; 
		return s;
	};
	for (Curve& curve : curves) {
		dataPtr = &curve.data;
		switch (curve.type) {
			case Types::move:
				result += command('M') + ord(0) + ord(1);
				break;
			case Types::line:
				result += command('L') + ord(0) + ord(1);
				break;
			case Types::quad:
				result += command('Q') + ord(0) + ord(1) + ord(2) + ord(3);
				break;
			case Types::cubic:
				result += command('C') + ord(0) + ord(1) + ord(2) + ord(3) + ord(4) + ord(5);
				break;
			case Types::close:
				result += command('Z');
				break;
			default:
				OP_ASSERT(0);
		}
	}
	if (result.size() && ' ' == result.back())
		result.pop_back();
	return result;
}

using namespace PathOpsV0Lib;

// insert moveTo and close as needed here
void Path::opAddPath(Context* context, Contour* contour, bool closeLoops) {
	if (!curves.size())
		return;
	OpPoint closeLine[2] {{0, 0}, {0, 0}};  // last point, first point
	for (Curve& curve : curves) {
		OpPoint* pts = curve.data.size() ? (OpPoint*) &curve.data.front() : nullptr;
		switch (curve.type) {
			case Types::move:
				if (closeLoops && closeLine[0] != closeLine[1])
					Add(contour, { closeLine, sizeof(closeLine), (CurveType) Types::line } );
				closeLine[0] = closeLine[1] = *pts++;
				break;
			case Types::line:
				if (closeLine[0] != pts[0]) {
					OpPoint closer[2] { closeLine[0], pts[0] };
					Add(contour, { closer, sizeof closer, (CurveType) Types::line } );
				}
				closeLine[0] = *pts++;
				break;
			case Types::quad: {
				OpPoint q[3] { closeLine[0], pts[1], pts[0] };
				AddQuads(contour, { q, sizeof q, (CurveType) Types::quad } );
				closeLine[0] = q[1];
				pts += 2;
				} break;
			case Types::cubic: {
				OpPoint c[4] { closeLine[0], pts[2], pts[0], pts[1] };
				AddCubics(contour, { c, sizeof c, (CurveType) Types::cubic } );
				closeLine[0] = c[1];
				pts += 3;
				} break;
			case Types::close:
				if (closeLoops && closeLine[0] != closeLine[1])
					Add(contour, { closeLine, sizeof closeLine, (CurveType) Types::line } );
				closeLine[0] = closeLine[1];
				continue;
			default:
				OP_ASSERT(0);
		}
	}
	if (closeLoops && closeLine[0] != closeLine[1])
		Add(contour, { closeLine, sizeof closeLine, (CurveType) Types::line } );
}

static Contour* GetBinary(Context* context, Winding winding, 
		BinaryOperand operand, BinaryWindType windType, Ops ops) {
	// set winding callbacks
	Contour* contour = CreateContour(context, winding);
	WindingKeep operatorFunc = nullptr;
	switch (ops) {
		case Ops::diff: operatorFunc = binaryWindingDifferenceFunc; break;
		case Ops::sect: operatorFunc = binaryWindingIntersectFunc; break;
		case Ops::_union: operatorFunc = binaryWindingUnionFunc; break;
		case Ops::revDiff: operatorFunc = binaryWindingReverseDifferenceFunc; break;
		case Ops::_xor: operatorFunc = binaryWindingExclusiveOrFunc; break;
		default: OP_ASSERT(0);
	}
	SetWindingCallBacks(contour, { binaryWindingAddFunc, operatorFunc, binaryWindingVisibleFunc, 
			binaryWindingZeroFunc, binaryWindingSubtractFunc });
#if OP_DEBUG
	SetDebugWindingCallBacks(contour, { nullptr, 0 }, nullptr
			OP_DEBUG_DUMP_PARAMS(binaryWindingDumpInFunc, binaryWindingDumpOutFunc, nullptr)
			OP_DEBUG_IMAGE_PARAMS(binaryWindingImageOutFunc, nullptr,
					nullptr, nullptr, nullptr)
	);
#endif
	return contour;
}

static Contour* GetUnary(Context* context, Winding winding) {
    Contour* contour = CreateContour(context, winding);
    SetWindingCallBacks(contour, { unaryWindingAddFunc, unaryWindingKeepFunc, unaryWindingVisibleFunc,
			unaryWindingZeroFunc, unaryWindingSubtractFunc });
#if OP_DEBUG
	SetDebugWindingCallBacks(contour, { nullptr, 0 }, nullptr
            OP_DEBUG_DUMP_PARAMS(unaryWindingDumpInFunc, unaryWindingDumpOutFunc, nullptr)
            OP_DEBUG_IMAGE_PARAMS(unaryWindingImageOutFunc, nullptr,
	        nullptr, nullptr, nullptr)
    );
#endif
    return contour;
}

struct OutPath {
	void commonOutput(PathOpsV0Lib::Curve c, Types type, bool firstPt, bool lastPt);
	bool extendCommon(OpPoint prior, OpPoint* last, OpPoint pt);
	bool extendLast(OpPoint pt);
	bool wrapAround(OpPoint pt);

	Path result;
	size_t startIndex;  // to handle wrap around, save moveTo position
};

void OutPath::commonOutput(PathOpsV0Lib::Curve c, Types type, bool firstPt, bool lastPt) {
	if (firstPt) {
		startIndex = result.curves.size();
		result.moveTo(c.data->start.x, c.data->start.y);
	}
	switch (type) {
		case Types::line:
			if ((!lastPt || firstPt || !wrapAround(c.data->end)) && !extendLast(c.data->end))
				result.lineTo(c.data->end.x, c.data->end.y);
			break;
		case Types::quad: {
			float* ctrls = (float*) ((char*)(c.data) + sizeof(CurveData));
			result.quadraticCurveTo(ctrls[0], ctrls[1], c.data->end.x, c.data->end.y);
			} break;
		case Types::cubic: {
			float* ctrls = (float*) ((char*)(c.data) + sizeof(CurveData));
			result.bezierCurveTo(ctrls[0], ctrls[1], ctrls[2], ctrls[3], c.data->end.x, c.data->end.y);
			} break;
		case Types::move:
		case Types::close:
		default:
			OP_ASSERT(0);
	}
	if (lastPt)
		result.closePath();
}

bool OutPath::extendCommon(OpPoint prior, OpPoint* last, OpPoint pt) {
	if (prior.x == last->x && last->x == pt.x
			&& OpMath::Between(prior.y, last->y, pt.y)) {
		last->y = pt.y;
		return true;
	}
	if (prior.y == last->y && last->y == pt.y
			&& OpMath::Between(prior.x, last->x, pt.x)) {
		last->x = pt.x;
		return true;
	}
	float leftCross = (prior.x - last->x) * (last->y - pt.y);
	float rightCross = (prior.y - last->y) * (last->x - pt.x);
	if (leftCross && rightCross && leftCross == rightCross
			&& OpMath::Between(prior.y, last->y, pt.y)
			&& OpMath::Between(prior.x, last->x, pt.x)) {
		*last = pt;
		return true;
	}
	return false;
}

bool OutPath::extendLast(OpPoint pt) {
	if (result.curves.size() < 2)
		return false;
	Curve& lastC = result.curves.back();
	if (Types::line != lastC.type)
		return false;
	OpPoint* last = ((OpPoint*) &lastC.data.front());
	Curve& priorC = *(&result.curves.back() - 1);
	if (Types::close == priorC.type)
		return false;
	OpPoint prior = *((OpPoint*) (&priorC.data.back() - 1));
	return extendCommon(prior, last, pt);
}

bool OutPath::wrapAround(OpPoint pt) {
	if (result.curves.size() < 3)
		return false;
	Curve* firstMove = &result.curves.front() + startIndex;
	OP_ASSERT(Types::move == firstMove->type);
	OpPoint* firstPt = (OpPoint*)&firstMove->data.front();
	if (pt != *firstPt)
		return false;
	Curve* line = firstMove + 1;
	if (Types::line != line->type)
		return false;
	Curve* back = &result.curves.back();
	OpPoint last = *(OpPoint*)(&back->data.back() - 1);
	OpPoint linePt = *(OpPoint*)&line->data.front();
	if (!extendCommon(linePt, firstPt, last))
		return false;
	// it's possible that one more extension can be done
	Curve* prior = back - 1;
	if (Types::line == back->type && prior > line + 1) {  // at least one out-of-line curve
		OpPoint priorPt = *(OpPoint*)(&prior->data.back() - 1);
		if (extendCommon(linePt, firstPt, priorPt))
			result.curves.pop_back();
	}
	return true;
}

static void EmptyFunc(PathOutput pathOutput) {
	((Path*) pathOutput)->clear();
}

static void LineOutput(PathOpsV0Lib::Curve c, bool firstPt, bool lastPt, PathOutput pathOutput) {
	OutPath* output = (OutPath*) pathOutput;
	output->commonOutput(c, Types::line, firstPt, lastPt);
}

static void QuadOutput(PathOpsV0Lib::Curve c, bool firstPt, bool lastPt, PathOutput pathOutput) {
	OutPath* output = (OutPath*) pathOutput;
	output->commonOutput(c, Types::quad, firstPt, lastPt);
}

#if ARC_SUPPORT
static void ConicOutput(PathOpsV0Lib::Curve c, bool firstPt, bool lastPt, PathOutput pathOutput) {
	Path* output = (Path*) pathOutput;
	output->commonOutput(c, Types::conic, firstPt, lastPt);
}
#endif

static void CubicOutput(PathOpsV0Lib::Curve c, bool firstPt, bool lastPt, PathOutput pathOutput) {
	OutPath* output = (OutPath*) pathOutput;
	output->commonOutput(c, Types::cubic, firstPt, lastPt);
}

static PathOpsV0Lib::CurveType LineType(PathOpsV0Lib::Curve ) {
	return (CurveType) Types::line;
}

static void SetupCurves(Context* context) {
	OP_DEBUG_CODE(CurveType lineType =) SetCurveCallBacks(context, { LineOutput });
	OP_DEBUG_CODE(SetDebugCurveCallBacks(context, lineType, debugLineScale
            OP_DEBUG_DUMP_PARAMS(lineDebugDumpName, noDumpCurveExtra)
            OP_DEBUG_IMAGE_PARAMS(debugLineAddToSkPath)));
	OP_ASSERT((int) lineType == (int) Types::line);
	OP_DEBUG_CODE(CurveType quadType =) SetCurveCallBacks(context, { QuadOutput, quadAxisT, 
			quadHull, quadIsFinite, quadIsLine, quadSetBounds, quadPinCtrl, 
			quadTangent, quadsEqual, quadPtAtT, quadHullPtCount, quadRotate, 
			quadSubDivide, quadXYAtT });
	OP_DEBUG_CODE(SetDebugCurveCallBacks(context, quadType, debugQuadScale
            OP_DEBUG_DUMP_PARAMS(quadDebugDumpName, noDumpCurveExtra)
            OP_DEBUG_IMAGE_PARAMS(debugQuadAddToSkPath)));
	OP_ASSERT((int) quadType == (int) Types::quad);
    OP_DEBUG_CODE(CurveType cubicType =) SetCurveCallBacks(context, { CubicOutput, cubicAxisT, 
			cubicHull, cubicIsFinite, cubicIsLine, cubicSetBounds, cubicPinCtrl, 
			cubicTangent, cubicsEqual, cubicPtAtT, cubicHullPtCount, cubicRotate, 
			cubicSubDivide, cubicXYAtT, cubicReverse });
	OP_DEBUG_CODE(SetDebugCurveCallBacks(context, cubicType, debugCubicScale
            OP_DEBUG_DUMP_PARAMS(cubicDebugDumpName, noDumpCurveExtra)
            OP_DEBUG_IMAGE_PARAMS(debugCubicAddToSkPath)));
	OP_ASSERT((int) cubicType == (int) Types::cubic);
}

ContextError FillPath::opCommon(FillPath& path, Ops oper) {
	Context* context = CreateContext();
	SetContextCallBacks(context, { LineType, EmptyFunc });	
	SetupCurves(context);
	int leftData[] = { 1, 0 };
	Winding leftWinding { leftData, sizeof(leftData) };
	Contour* left = GetBinary(context, leftWinding, 
			BinaryOperand::left, BinaryWindType::windLeft, oper);
	opAddPath(context, left, true);
	int rightData[] = { 0, 1 };
	Winding rightWinding { rightData, sizeof(rightData) };
	Contour* right = GetBinary(context, rightWinding,
			BinaryOperand::left, BinaryWindType::windRight, oper);
	path.opAddPath(context, right, true);
	return handleError(context);
}

ContextError FillPath::simplify() {
	Context* context = CreateContext();
	SetContextCallBacks(context, { LineType, EmptyFunc });	
	SetupCurves(context);
    int simpleData[] = { 1 };
    Winding simpleWinding { simpleData, sizeof(simpleData) };
	Contour* simple = GetUnary(context, simpleWinding);
    opAddPath(context, simple, true);
	return handleError(context);
}

enum class FrameFill {
	frame,
	fill
};

struct FrameWinding {
    FrameWinding(FrameFill frameFill) 
		: isFrame(frameFill) {
	}

    FrameWinding(FrameFill frameFill, int windValue) 
		: left(windValue)
		, isFrame(frameFill) {
	}

    FrameWinding(Winding w) {
        OP_ASSERT(w.size == sizeof(FrameWinding));
        std::memcpy(this, w.data, sizeof(FrameWinding));
	}

	void copyTo(Winding& w) const {
		OP_ASSERT(w.size == sizeof(FrameWinding));
		std::memcpy(w.data, this, sizeof(FrameWinding));
	}

#if OP_DEBUG_DUMP
	static std::string DumpOutFunc(Winding winding) {
		FrameWinding fw(winding);
		std::string s = "{" + STR(fw.left) + "}";
		return s;
	}
#endif

    int left = 1;
	FrameFill isFrame;
};

// winding is always frame; toAdd comes from another edge, and may be frame or fill
static void frameAddFunc(Winding winding, Winding toAdd) {
	FrameWinding sum(winding);
	if (FrameFill::frame == sum.isFrame)
		return;
	FrameWinding addend(toAdd);
	if (FrameFill::frame == addend.isFrame)
		return;
	sum.left += addend.left;
	sum.copyTo(winding);
}

// both winding and sumWinding come from the same edge
static WindKeep frameKeepFunc(Winding winding, Winding sumWinding) {
	FrameWinding wind(winding);
	if (FrameFill::fill == wind.isFrame)
		return WindKeep::Discard;
	FrameWinding sum(sumWinding);
	return sum.left ? WindKeep::Start : WindKeep::Discard;
}

static WindKeep frameDiscardFunc(Winding winding, Winding sumWinding) {
	FrameWinding wind(winding);
	if (FrameFill::fill == wind.isFrame)
		return WindKeep::Discard;
	FrameWinding sum(sumWinding);
	return !sum.left ? WindKeep::Start : WindKeep::Discard;
}

// winding is always frame; toAdd comes from another edge, and may be frame or fill
static void frameSubtractFunc(Winding winding, Winding toSubtract) {
	FrameWinding difference(winding);
	if (FrameFill::frame == difference.isFrame)
		return;
	FrameWinding subtrahend(toSubtract);
	if (FrameFill::frame == subtrahend.isFrame)
		return;
	difference.left -= subtrahend.left;
	difference.copyTo(winding);
}

bool frameVisibleFunc(Winding winding) {
    FrameWinding test(winding);
    return !!test.left;
}

void frameZeroFunc(Winding toZero) {
    FrameWinding zero(FrameFill::fill, 0);
    zero.copyTo(toZero);
}

static bool allowDisjointLines(ContextError err, Context* , PathOpsV0Lib::Curve* ) {
	return ContextError::end != err && ContextError::missing != err;
}

ContextError Path::handleError(Context* context) {
	OutPath outPath;
	Normalize(context);
	Resolve(context, &outPath);
	ContextError error = Error(context);
	if (ContextError::none == error)
		curves = outPath.result.curves;
	DeleteContext(context);
	return error;
}

#if OP_DEBUG_IMAGE
std::string frameWindingImageOutFunc(Winding winding, int index) {
    if (index > 0)
        return "-";
    FrameWinding frameWinding(winding);
    std::string s = STR(frameWinding.left);
    return s;
}
#endif


ContextError FramePath::opCommon(FillPath& path, Ops oper) {
	Context* context = CreateContext();
    SetContextCallBacks(context, { LineType });
	SetupCurves(context);
    FrameWinding frameData(FrameFill::frame, 1);
    Winding frameWinding { &frameData, sizeof(frameData) };
    Contour* frameContour = CreateContour(context, frameWinding);
	WindingKeep operatorFunc = Ops::sect == oper ? frameKeepFunc : frameDiscardFunc;
    SetWindingCallBacks(frameContour, { frameAddFunc, operatorFunc, frameVisibleFunc, frameZeroFunc, 
            frameSubtractFunc });
#if OP_DEBUG
	SetDebugWindingCallBacks(frameContour, { nullptr, 0 }, noDebugBitOper
            OP_DEBUG_DUMP_PARAMS(nullptr, FrameWinding::DumpOutFunc, nullptr)
            OP_DEBUG_IMAGE_PARAMS(frameWindingImageOutFunc, noNativePathFunc,
            noDebugGetDrawFunc, noDebugSetDrawFunc, noIsOppFunc)
	);
#endif

    FrameWinding fillData(FrameFill::fill, 1);
    Winding fillWinding { &fillData, sizeof(fillData) };
    Contour* fillContour = CreateContour(context, fillWinding);
    SetWindingCallBacks(fillContour, { frameAddFunc, operatorFunc, frameVisibleFunc, frameZeroFunc, 
            frameSubtractFunc });
#if OP_DEBUG
	SetDebugWindingCallBacks(fillContour, { nullptr , 0 }, noDebugBitOper
	        OP_DEBUG_DUMP_PARAMS(nullptr, FrameWinding::DumpOutFunc, nullptr)
            OP_DEBUG_IMAGE_PARAMS(frameWindingImageOutFunc, noNativePathFunc,
            noDebugGetDrawFunc, noDebugSetDrawFunc, noIsOppFunc)
	);
#endif
	opAddPath(context, frameContour, false);
	path.opAddPath(context, fillContour, true);
	SetErrorHandler(context, allowDisjointLines);
	return handleError(context);
}

}
