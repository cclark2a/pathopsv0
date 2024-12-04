// (c) 2024, Cary Clark cclark2@gmail.com

#include "Path2D.h"
#include "curves/Line.h"
#include "curves/NoCurve.h"
#include "curves/QuadBezier.h"

using namespace PathOpsV0Lib;

namespace TwoD {

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
	points.push_back(cx);
	points.push_back(cy);
	points.push_back(x);
	points.push_back(y);
	types.push_back(Types::quad);
}

void Path::closePath() {
	types.push_back(Types::close);
}

char* Path::skipSpace(char* ch, const char* chEnd) {
	while (ch < chEnd && ' ' >= *ch)
		ch++;
	return ch;
}

float Path::nextFloat(char** chPtr, const char* chEnd) {
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

Path* Path::fromCommands(std::string s) {
	reset();
	size_t strLen = s.size();
	if (!strLen)
		return this;
	char* ch = &s.front();
	const char* chEnd = ch + strLen;
	ch = skipSpace(ch, chEnd);
	if ('[' != *ch++)
		return this;
	auto skipCommaSpace = [&ch, chEnd]() {
		ch = skipSpace(ch, chEnd);
		if (ch >= chEnd)
			return;
		if (',' == *ch)
			++ch;
		ch = skipSpace(ch, chEnd);
	};
	auto skipQuoteCommaSpace = [&ch, chEnd, skipCommaSpace]() {  // comma, space optional
		if (ch >= chEnd || '"' != *ch)
			return;
		++ch;
		skipCommaSpace();
	};
	auto skipBracketCommaSpace = [&ch, chEnd, skipCommaSpace]() {  // comma, space optional
		if (ch >= chEnd || ']' != *ch)
			return;
		++ch;
		skipCommaSpace();
	};
	auto nextFloatComma = [&ch, chEnd]() {
		float result = nextFloat(&ch, chEnd);
		if (ch >= chEnd)
			return result;
		if (',' == *ch)
			++ch;
		ch = skipSpace(ch, chEnd);
		return result;
	};
	while (ch < chEnd) {
		if ('[' != *ch++)
			return this;
		if ('"' != *ch++)
			return this;
		switch (*ch++) {
			case 'M': {
				skipQuoteCommaSpace();
				float x = nextFloatComma();
				float y = nextFloatComma();
				skipBracketCommaSpace();
				moveTo(x, y);
				} break;
			case 'L': {
				skipQuoteCommaSpace();
				float x = nextFloatComma();
				float y = nextFloatComma();
				skipBracketCommaSpace();
				lineTo(x, y);
				} break;
			case 'Q': {
				skipQuoteCommaSpace();
				float cx = nextFloatComma();
				float cy = nextFloatComma();
				float x = nextFloatComma();
				float y = nextFloatComma();
				skipBracketCommaSpace();
				quadraticCurveTo(cx, cy, x, y);
				} break;
			case 'Z':
				skipQuoteCommaSpace();
				skipBracketCommaSpace();
				closePath();
				break;
			default:
				OP_ASSERT(0);
				return this;
		}
	}
	return this;
}

Path* Path::fromSVG(std::string s) {
	reset();
	size_t strLen = s.size();
	if (!strLen)
		return this;
	char* ch = &s.front();
	const char* chEnd = ch + strLen;
	while (ch < chEnd) {
		switch (*ch++) {
			case 'M': {
				float x = nextFloat(&ch, chEnd);
				float y = nextFloat(&ch, chEnd);
				moveTo(x, y);
				} break;
			case 'L': {
				float x = nextFloat(&ch, chEnd);
				float y = nextFloat(&ch, chEnd);
				lineTo(x, y);
				} break;
			case 'Q': {
				float cx = nextFloat(&ch, chEnd);
				float cy = nextFloat(&ch, chEnd);
				float x = nextFloat(&ch, chEnd);
				float y = nextFloat(&ch, chEnd);
				quadraticCurveTo(cx, cy, x, y);
				} break;
			case 'Z':
				ch = skipSpace(ch, chEnd);
				closePath();
				break;
			default:
				OP_ASSERT(0);
				return this;
		}
	}
	return this;
}

std::string Path::toCommands() {
	std::string result = "[";
	float* ordinal = &points.front();
	for (Types type : types) {
		switch (type) {
			case Types::move:
				result += "[\"M\", " + std::to_string(ordinal[0]) + ", " + std::to_string(ordinal[1]) + "], ";
				ordinal += 2;
				break;
			case Types::line:
				result += "[\"L\", " + std::to_string(ordinal[0]) + ", " + std::to_string(ordinal[1]) + "], ";
				ordinal += 2;
				break;
			case Types::quad:
				result += "[\"Q\", " + std::to_string(ordinal[0]) + ", " + std::to_string(ordinal[1]) +  ", ";
				result +=              std::to_string(ordinal[2]) + ", " + std::to_string(ordinal[3]) + "], ";
				ordinal += 4;
				break;
			case Types::close:
				result += "[\"Z\"], ";
				break;
			default:
				OP_ASSERT(0);
		}
	}
	if (' ' == result.back())
		result.pop_back();
	if (',' == result.back())
		result.pop_back();
	return result + "]";
}

std::string Path::toSVG() {
	std::string result;
	float* ordinal = &points.front();
	for (Types type : types) {
		switch (type) {
			case Types::move:
				result += "M" + std::to_string(ordinal[0]) + " " + std::to_string(ordinal[1]) + " ";
				ordinal += 2;
				break;
			case Types::line:
				result += "L" + std::to_string(ordinal[0]) + " " + std::to_string(ordinal[1]) + " ";
				ordinal += 2;
				break;
			case Types::quad:
				result += "Q" + std::to_string(ordinal[0]) + " " + std::to_string(ordinal[1]) + " ";
				result +=       std::to_string(ordinal[2]) + " " + std::to_string(ordinal[3]) + " ";
				ordinal += 4;
				break;
			case Types::close:
				result += "Z ";
				break;
			default:
				OP_ASSERT(0);
		}
	}
	if (' ' == result.back())
		result.pop_back();
	return result;
}

void Path::addPath(Context* context, AddWinding winding) {
	if (!points.size())
		return;
	float* ordinal = &points.front();
	float closeLine[4] {0, 0, 0, 0};
	for (Types type : types) {
		switch (type) {
			case Types::move:
				closeLine[2] = ordinal[0];
				closeLine[3] = ordinal[1];
				break;
			case Types::line:
				Add({ (OpPoint*) ordinal, sizeof closeLine, (CurveType) Types::line }, winding);
				closeLine[0] = ordinal[2];
				closeLine[1] = ordinal[3];
				ordinal += 2;
				break;
			case Types::quad: {
				float q[6] { ordinal[0], ordinal[1], ordinal[4], ordinal[5], ordinal[2], ordinal[3] };
				AddQuads({ (OpPoint*) q, sizeof q, (CurveType) Types::quad }, winding);
				closeLine[0] = ordinal[2];
				closeLine[1] = ordinal[3];
				ordinal += 4;
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

#if OP_DEBUG_DUMP
std::string binaryDumpFunc(CallerData caller, DebugLevel debugLevel, DebugBase debugBase) {
    return "";
}
#endif

#if OP_DEBUG_IMAGE
void* debugOpPathFunc(CallerData data) {
	return nullptr;
}

bool debugOpGetDrawFunc(CallerData data) {
	return false;
}

void debugOpSetDrawFunc(CallerData data, bool draw) {
}

bool debugOpSetIsOppFunc(CallerData data) {
	return false;
}
#endif

Contour* Path::GetContour(Context* context, BinaryOperand operand, BinaryWindType windType, Ops ops) {
	// set winding callbacks
	BinaryOpData windingUserData { (BinaryOperation) ops, operand };
	Contour* contour = CreateContour({context, (ContourData*) &windingUserData,
			sizeof(windingUserData) } );
	WindingKeep operatorFunc = noWindKeepFunc;
	switch (ops) {
		case Ops::diff: operatorFunc = binaryWindingDifferenceFunc; break;
		case Ops::intersect: operatorFunc = binaryWindingIntersectFunc; break;
		case Ops::_union: operatorFunc = binaryWindingUnionFunc; break;
		case Ops::exor: operatorFunc = binaryWindingExclusiveOrFunc; break;
		default: OP_ASSERT(0);
	}
	SetWindingCallBacks(contour, binaryWindingAddFunc, operatorFunc, 
			binaryWindingSubtractFunc, binaryWindingVisibleFunc, binaryWindingZeroFunc 
			OP_DEBUG_PARAMS(nullptr)
			OP_DEBUG_DUMP_PARAMS(binaryWindingDumpInFunc, binaryWindingDumpOutFunc, binaryDumpFunc)
			OP_DEBUG_IMAGE_PARAMS(binaryWindingImageOutFunc, debugOpPathFunc,
					debugOpGetDrawFunc, debugOpSetDrawFunc, debugOpSetIsOppFunc)
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
		case Types::move:
		case Types::conic:
		case Types::cubic:
		case Types::close:
		default:
			OP_ASSERT(0);
	}
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

static Curve MakeLine(Curve c) {
	c.type = (CurveType) Types::line;
	c.size = sizeof(float) * 4;
	return c;
}

static PathOpsV0Lib::CurveType LineType(Curve ) {
	return (CurveType) Types::line;
}

#if OP_DEBUG
void debugLineScale(Curve curve, double scale, double offsetX, double offsetY) {
}

void debugQuadScale(Curve curve, double scale, double offsetX, double offsetY) {
}
#endif

static void SetupContext(Context* context) {
	// set context callbacks
	SetContextCallBacks(context, EmptyFunc, MakeLine, LineType, maxSignSwap,
			maxDepth, maxSplits, maxLimbs);	
	// set curve callbacks
	OP_DEBUG_CODE(CurveType lineType =) SetCurveCallBacks(context, lineAxisRawHit, noHull, 
			lineIsFinite, lineIsLine, noBounds, lineNormal, LineOutput, noPinCtrl, 
			noReverse, lineTangent, linesEqual, linePtAtT, linePtCount, noRotate, 
			lineSubDivide, lineXYAtT, lineCut, lineNormalLimit, lineInterceptLimit
			OP_DEBUG_PARAMS(debugLineScale)
			OP_DEBUG_DUMP_PARAMS(lineDebugDumpName, noDumpCurveExtra)
			OP_DEBUG_IMAGE_PARAMS(nullptr)
	);
	OP_ASSERT((int) lineType == (int) Types::line);
	OP_DEBUG_CODE(CurveType quadType =) SetCurveCallBacks(context, quadAxisRawHit, quadHull, 
			quadIsFinite, quadIsLine, quadSetBounds, quadNormal, QuadOutput, quadPinCtrl, 
			noReverse, quadTangent, quadsEqual, quadPtAtT, quadPtCount, quadRotate, 
			quadSubDivide, quadXYAtT, lineCut, lineNormalLimit, lineInterceptLimit
			OP_DEBUG_PARAMS(debugQuadScale)
			OP_DEBUG_DUMP_PARAMS(quadDebugDumpName, noDumpCurveExtra)
			OP_DEBUG_IMAGE_PARAMS(nullptr)
	);
	OP_ASSERT((int) quadType == (int) Types::quad);
}

Path PathOps::Op(Path& path1, Path& path2, Ops oper) {
	Context* context = CreateContext({ nullptr, 0 });
	SetupContext(context);
	Contour* left = Path::GetContour(context, BinaryOperand::left, BinaryWindType::windLeft, oper);
	int leftData[] = { 1, 0 };
	AddWinding leftWinding { left, leftData, sizeof(leftData) };
	path1.addPath(context, leftWinding);
	Contour* right = Path::GetContour(context, BinaryOperand::left, BinaryWindType::windRight, oper);
	int rightData[] = { 0, 1 };
	AddWinding rightWinding { right, rightData, sizeof(rightData) };
	path2.addPath(context, rightWinding);
	Path result;
	Normalize(context);
	Resolve(context, &result);
	return result;
}

Path PathOps::Intersect(Path& path1, Path& path2) {
	return Op(path1, path2, Ops::intersect);
}

}
