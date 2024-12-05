// (c) 2024, Cary Clark cclark2@gmail.com

#include "Path2D.h"
#include "curves/Line.h"
#include "curves/NoCurve.h"
#if CONIC_SUPPORT
#include "curves/ConicBezier.h"
#endif
#include "curves/CubicBezier.h"
#include "curves/QuadBezier.h"

using namespace PathOpsV0Lib;

namespace TwoD {

void Path::addPath(Path& path) {
	points.insert(points.end(), path.points.begin(), path.points.end());
	types.insert(types.end(), path.types.begin(), path.types.end());
}

void Path::moveTo(float x, float y) {
	points.push_back(x);
	points.push_back(y);
	types.push_back(Types::move);
}

void Path::lineTo(float x, float y) {
	if (!points.size())
		moveTo(0, 0);
	points.push_back(x);
	points.push_back(y);
	types.push_back(Types::line);
}

void Path::quadraticCurveTo(float cx, float cy, float x, float y) {
	points.insert(points.end(), { cx, cy, x, y });
	types.push_back(Types::quad);
}

void Path::bezierCurveTo(float c1x, float c1y, float c2x, float c2y, float x, float y) {
	points.insert(points.end(), { c1x, c1y, c2x, c2y, x, y });
	types.push_back(Types::cubic);
}

void Path::closePath() {
	types.push_back(Types::close);
}

#if CONIC_SUPPORT
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

void Path::rect(float x, float y, float width, float height) {
	moveTo(x, y);
	lineTo(x + width, y);
	lineTo(x + width, y + height);
	lineTo(x, y + height);
	closePath();
}

static char* skipSpace(char* ch, const char* chEnd) {
	while (ch < chEnd && ' ' >= *ch)
		ch++;
	return ch;
}

static float nextFloat(char** chPtr, const char* chEnd) {
	char* ch = *chPtr;
	if (ch >= chEnd)
		return 0.f;
	std::string temp;
	ch = skipSpace(ch, chEnd);
	while (('0' <= *ch && *ch <= '9') || '+' == *ch || '-' == *ch || '.' == *ch)
		temp += *ch++;
	*chPtr = skipSpace(ch, chEnd);
	return (float) std::atof(temp.c_str());
}

static float nextFloatComma(char** chPtr, const char* chEnd) {
	float result = nextFloat(chPtr, chEnd);
	if (*chPtr < chEnd && ',' == **chPtr)
		(*chPtr)++;
	*chPtr = skipSpace(*chPtr, chEnd);
	return result;
}

Path* Path::fromCommon(std::string s, Format format) {
	reset();
	size_t strLen = s.size();
	if (!strLen)
		return this;
	char* ch = &s.front();
	const char* chEnd = ch + strLen;
	ch = skipSpace(ch, chEnd);
	if (Format::command == format && '[' != *ch++)
		return this;
	auto skipCommaSpace = [&ch, chEnd, format](char delimiter) {
		if (ch >= chEnd)
			return;
		if (Format::command == format) {
			if (delimiter != *ch)
				return;
			++ch;
		}
		ch = skipSpace(ch, chEnd);
		if (ch >= chEnd)
			return;
		if (',' == *ch)
			++ch;
		ch = skipSpace(ch, chEnd);
	};
	while (ch < chEnd) {
		if (Format::command == format && '[' != *ch++)
			return this;
		if (Format::command == format && '"' != *ch++)
			return this;
		switch (*ch++) {
			case 'M': {
				skipCommaSpace('"');
				float x = nextFloatComma(&ch, chEnd);
				float y = nextFloatComma(&ch, chEnd);
				skipCommaSpace(']');
				moveTo(x, y);
				} break;
			case 'L': {
				skipCommaSpace('"');
				float x = nextFloatComma(&ch, chEnd);
				float y = nextFloatComma(&ch, chEnd);
				skipCommaSpace(']');
				lineTo(x, y);
				} break;
			case 'Q': {
				skipCommaSpace('"');
				float cx = nextFloatComma(&ch, chEnd);
				float cy = nextFloatComma(&ch, chEnd);
				float x = nextFloatComma(&ch, chEnd);
				float y = nextFloatComma(&ch, chEnd);
				skipCommaSpace(']');
				quadraticCurveTo(cx, cy, x, y);
				} break;
			case 'C': {
				skipCommaSpace('"');
				float c1x = nextFloatComma(&ch, chEnd);
				float c1y = nextFloatComma(&ch, chEnd);
				float c2x = nextFloatComma(&ch, chEnd);
				float c2y = nextFloatComma(&ch, chEnd);
				float x = nextFloatComma(&ch, chEnd);
				float y = nextFloatComma(&ch, chEnd);
				skipCommaSpace(']');
				bezierCurveTo(c1x, c1y, c2x, c2y, x, y);
				} break;
			case 'Z':
				skipCommaSpace('"');
				skipCommaSpace(']');
				closePath();
				break;
			default:
				OP_ASSERT(0);
				return this;
		}
	}
	return this;
}

std::string Path::toCommon(Format format) {
	std::string result;
	if (Format::command == format)
		result += '[';
	float* ordinal = &points.front();
	auto closeBracket = [format](bool final=true) {
		std::string s;
		if (Format::command == format) {
			if (final)
				s += ']';
			s += ',';
		}
		s += ' '; 
		return s;
	};
	auto ord = [&ordinal, closeBracket](int index, bool final=false) {
		std::string s;
		s = std::to_string(ordinal[index]);
		s.erase(s.find_last_not_of('0') + 1, std::string::npos); 
		s.erase(s.find_last_not_of('.') + 1, std::string::npos);
		s += closeBracket(final);
		return s;
	};
	auto command = [format](char cmd) {
		std::string s;
		if (Format::command == format)
			s = "[\"";
		s += cmd;
		if (Format::command == format)
			s += '"';
		s += ' '; 
		return s;
	};
	for (Types type : types) {
		switch (type) {
			case Types::move:
				result += command('M') + ord(0) + ord(1, true);
				ordinal += 2;
				break;
			case Types::line:
				result += command('L') + ord(0) + ord(1, true);
				ordinal += 2;
				break;
			case Types::quad:
				result += command('Q') + ord(0) + ord(1) + ord(2) + ord(3, true);
				ordinal += 4;
				break;
			case Types::cubic:
				result += command('C') + ord(0) + ord(1) + ord(2) + ord(3) + ord(4) + ord(5, true);
				ordinal += 4;
				break;
			case Types::close:
				result += command('Z') + closeBracket();
				break;
			default:
				OP_ASSERT(0);
		}
	}
	if (' ' == result.back())
		result.pop_back();
	if (',' == result.back())
		result.pop_back();
	if (Format::command == format)
		result += "]";
	return result;
}

void Path::addPath(Context* context, AddWinding winding) {
	if (!points.size())
		return;
	float* ord = &points.front();
	float closeLine[4] {0, 0, 0, 0};
	for (Types type : types) {
		switch (type) {
			case Types::move:
				closeLine[2] = ord[0];
				closeLine[3] = ord[1];
				break;
			case Types::line:
				Add({ (OpPoint*) ord, sizeof closeLine, (CurveType) Types::line }, winding);
				closeLine[0] = ord[2];
				closeLine[1] = ord[3];
				ord += 2;
				break;
			case Types::quad: {
				float q[6] { ord[0], ord[1], ord[4], ord[5], ord[2], ord[3] };
				AddQuads({ (OpPoint*) q, sizeof q, (CurveType) Types::quad }, winding);
				closeLine[0] = ord[4];
				closeLine[1] = ord[5];
				ord += 4;
				} break;
			case Types::cubic: {
				float c[8] { ord[0], ord[1], ord[6], ord[7], ord[2], ord[3], ord[4], ord[5] };
				AddQuads({ (OpPoint*) c, sizeof c, (CurveType) Types::cubic }, winding);
				closeLine[0] = ord[6];
				closeLine[1] = ord[7];
				ord += 6;
				} break;
			case Types::close:
				Add({ (OpPoint*) closeLine, sizeof closeLine, (CurveType) Types::line }, winding);
				closeLine[0] = closeLine[2];
				closeLine[1] = closeLine[3];
				continue;
			default:
				OP_ASSERT(0);
		}
	}
}

static Contour* GetBinary(Context* context, BinaryOperand operand, BinaryWindType windType, Ops ops) {
	// set winding callbacks
	BinaryOpData windingUserData { (BinaryOperation) ops, operand };
	Contour* contour = CreateContour({context, (ContourData*) &windingUserData,
			sizeof(windingUserData) } );
	WindingKeep operatorFunc = noWindKeepFunc;
	switch (ops) {
		case Ops::diff: operatorFunc = binaryWindingDifferenceFunc; break;
		case Ops::sect: operatorFunc = binaryWindingIntersectFunc; break;
		case Ops::_union: operatorFunc = binaryWindingUnionFunc; break;
		case Ops::revDiff: operatorFunc = binaryWindingReverseDifferenceFunc; break;
		case Ops::_xor: operatorFunc = binaryWindingExclusiveOrFunc; break;
		default: OP_ASSERT(0);
	}
	SetWindingCallBacks(contour, binaryWindingAddFunc, operatorFunc, 
			binaryWindingSubtractFunc, binaryWindingVisibleFunc, binaryWindingZeroFunc 
			OP_DEBUG_PARAMS(nullptr)
			OP_DEBUG_DUMP_PARAMS(binaryWindingDumpInFunc, binaryWindingDumpOutFunc, nullptr)
			OP_DEBUG_IMAGE_PARAMS(binaryWindingImageOutFunc, nullptr,
					nullptr, nullptr, nullptr)
	);
	return contour;
}

static Contour* GetUnary(Context* context) {
    Contour* contour = CreateContour({context, nullptr, 0});
    SetWindingCallBacks(contour, unaryWindingAddFunc, unaryWindingKeepFunc, 
            unaryWindingSubtractFunc, unaryWindingVisibleFunc, unaryWindingZeroFunc 
			OP_DEBUG_PARAMS(nullptr)
            OP_DEBUG_DUMP_PARAMS(unaryWindingDumpInFunc, unaryWindingDumpOutFunc, nullptr)
            OP_DEBUG_IMAGE_PARAMS(unaryWindingImageOutFunc, nullptr,
	                nullptr, nullptr, nullptr)
    );
    return contour;
}

void Path::commonOutput(Curve c, Types type, bool firstPt, bool lastPt, PathOutput output) {
	if (firstPt)
		moveTo(c.data->start.x, c.data->start.y);
	switch (type) {
		case Types::line:
			lineTo(c.data->end.x, c.data->end.y);
			break;
		case Types::quad: {
			float* ctrls = (float*) ((char*)(c.data) + sizeof(CurveData));
			quadraticCurveTo(ctrls[0], ctrls[1], c.data->end.x, c.data->end.y);
			} break;
		case Types::cubic: {
			float* ctrls = (float*) ((char*)(c.data) + sizeof(CurveData));
			bezierCurveTo(ctrls[0], ctrls[1], ctrls[2], ctrls[3], c.data->end.x, c.data->end.y);
			} break;
		case Types::move:
//		case Types::conic:
		case Types::close:
		default:
			OP_ASSERT(0);
	}
	if (lastPt)
		closePath();
}

void Path::reset() {	
	points.clear();
	types.clear();
}

static void EmptyFunc(PathOutput pathOutput) {
	((Path*) pathOutput)->reset();
}

static void LineOutput(Curve c, bool firstPt, bool lastPt, PathOutput pathOutput) {
	Path* output = (Path*) pathOutput;
	output->commonOutput(c, Types::line, firstPt, lastPt, output);
}

static void QuadOutput(Curve c, bool firstPt, bool lastPt, PathOutput pathOutput) {
	Path* output = (Path*) pathOutput;
	output->commonOutput(c, Types::quad, firstPt, lastPt, output);
}

#if CONIC_SUPPORT
static void ConicOutput(Curve c, bool firstPt, bool lastPt, PathOutput pathOutput) {
	Path* output = (Path*) pathOutput;
	output->commonOutput(c, Types::conic, firstPt, lastPt, output);
}
#endif

static void CubicOutput(Curve c, bool firstPt, bool lastPt, PathOutput pathOutput) {
	Path* output = (Path*) pathOutput;
	output->commonOutput(c, Types::cubic, firstPt, lastPt, output);
}

static Curve MakeLine(Curve c) {
	c.type = (CurveType) Types::line;
	c.size = sizeof(float) * 4;
	return c;
}

static PathOpsV0Lib::CurveType LineType(Curve ) {
	return (CurveType) Types::line;
}

static void SetupContext(Context* context) {
	// set context callbacks
	SetContextCallBacks(context, EmptyFunc, MakeLine, LineType, maxSignSwap,
			maxDepth, maxSplits, maxLimbs);	
	// set curve callbacks
	OP_DEBUG_CODE(CurveType lineType =) SetCurveCallBacks(context, lineAxisRawHit, noHull, 
			lineIsFinite, lineIsLine, noBounds, lineNormal, LineOutput, noPinCtrl, 
			noReverse, lineTangent, linesEqual, linePtAtT, linePtCount, noRotate, 
			lineSubDivide, lineXYAtT, lineCut, lineNormalLimit, lineInterceptLimit
			OP_DEBUG_PARAMS(nullptr)
			OP_DEBUG_DUMP_PARAMS(lineDebugDumpName, noDumpCurveExtra)
			OP_DEBUG_IMAGE_PARAMS(nullptr)
	);
	OP_ASSERT((int) lineType == (int) Types::line);
	OP_DEBUG_CODE(CurveType quadType =) SetCurveCallBacks(context, quadAxisRawHit, quadHull, 
			quadIsFinite, quadIsLine, quadSetBounds, quadNormal, QuadOutput, quadPinCtrl, 
			noReverse, quadTangent, quadsEqual, quadPtAtT, quadPtCount, quadRotate, 
			quadSubDivide, quadXYAtT, lineCut, lineNormalLimit, lineInterceptLimit
			OP_DEBUG_PARAMS(nullptr)
			OP_DEBUG_DUMP_PARAMS(quadDebugDumpName, noDumpCurveExtra)
			OP_DEBUG_IMAGE_PARAMS(nullptr)
	);
	OP_ASSERT((int) quadType == (int) Types::quad);
#if CONIC_SUPPORT
    OP_DEBUG_CODE(CurveType conicType =) SetCurveCallBacks(context, conicAxisRawHit, conicHull, 
            conicIsFinite, conicIsLine, conicSetBounds, conicNormal, ConicOutput, quadPinCtrl, 
			noReverse, conicTangent, conicsEqual, conicPtAtT, conicPtCount, conicRotate, 
			conicSubDivide, conicXYAtT, lineCut, lineNormalLimit, lineInterceptLimit
			OP_DEBUG_PARAMS(nullptr)
            OP_DEBUG_DUMP_PARAMS(conicDebugDumpName, conicDebugDumpExtra)
            OP_DEBUG_IMAGE_PARAMS(nullptr)
    );
	OP_ASSERT((int) conicType == (int) Types::conic);
#endif
    OP_DEBUG_CODE(CurveType cubicType =) SetCurveCallBacks(context, cubicAxisRawHit, cubicHull, 
            cubicIsFinite, cubicIsLine, cubicSetBounds, cubicNormal, CubicOutput, cubicPinCtrl, 
			cubicReverse, cubicTangent, cubicsEqual, cubicPtAtT, cubicPtCount, cubicRotate, 
			cubicSubDivide, cubicXYAtT, lineCut, lineNormalLimit, lineInterceptLimit
			OP_DEBUG_PARAMS(nullptr)
            OP_DEBUG_DUMP_PARAMS(cubicDebugDumpName, noDumpCurveExtra)
            OP_DEBUG_IMAGE_PARAMS(nullptr)
    );
	OP_ASSERT((int) cubicType == (int) Types::cubic);
}

void Path::opCommon(Path& path, Ops oper) {
	Context* context = CreateContext({ nullptr, 0 });
	SetupContext(context);
	Contour* left = GetBinary(context, BinaryOperand::left, BinaryWindType::windLeft, oper);
	int leftData[] = { 1, 0 };
	AddWinding leftWinding { left, leftData, sizeof(leftData) };
	addPath(context, leftWinding);
	Contour* right = GetBinary(context, BinaryOperand::left, BinaryWindType::windRight, oper);
	int rightData[] = { 0, 1 };
	AddWinding rightWinding { right, rightData, sizeof(rightData) };
	path.addPath(context, rightWinding);
	Path result;
	Normalize(context);
	Resolve(context, &result);
	*this = result;
}

void Path::simplify() {
	Context* context = CreateContext({ nullptr, 0 });
	SetupContext(context);
	Contour* simple = GetUnary(context);
    int simpleData[] = { 1 };
    AddWinding simpleWinding { simple, simpleData, sizeof(simpleData) };
    addPath(context, simpleWinding);
	Path result;
	Normalize(context);
	Resolve(context, &result);
	*this = result;
}

}
