// (c) 2023, Cary Clark cclark2@gmail.com
// new interface idea
#if OP_TINY_SKIA
#include "TinySkia.h"
#else
#include "include/core/SkPathTypes.h"
#include "include/core/SkPath.h"
#endif

#include "SkiaPaths.h"

static const SkPathOp inverseOpMapping[5][2][2] {
    {{kDifference_SkPathOp, kIntersect_SkPathOp}, {kUnion_SkPathOp, kReverseDifference_SkPathOp }},
    {{kIntersect_SkPathOp, kDifference_SkPathOp}, {kReverseDifference_SkPathOp, kUnion_SkPathOp }},
    {{kUnion_SkPathOp, kReverseDifference_SkPathOp}, {kDifference_SkPathOp, kIntersect_SkPathOp }},
    {{kXOR_SkPathOp, kXOR_SkPathOp }, {kXOR_SkPathOp, kXOR_SkPathOp}},
    {{kReverseDifference_SkPathOp, kUnion_SkPathOp}, {kIntersect_SkPathOp, kDifference_SkPathOp }},
};

/* Given a PathOps operator, and if the operand fills are inverted, return the equivalent operator
   treating the operands as non-inverted.
 */
SkPathOp MapInvertedSkPathOp(SkPathOp op, bool leftOperandIsInverted, bool rightOperandIsInverted) {
    return inverseOpMapping[(int) op][leftOperandIsInverted][rightOperandIsInverted];
}

static const bool outInverse[5][2][2] {
    { { false, false }, { true, false } },  // diff
    { { false, false }, { false, true } },  // sect
    { { false, true }, { true, true } },    // union
    { { false, true }, { true, false } },   // xor
    { { false, true }, { false, false } },  // rev diff
};

/* Given a PathOps operator, and if the operand fills are inverted, return true if the output is
   inverted.
 */
bool SkPathOpInvertOutput(SkPathOp op, bool leftOperandIsInverted, bool rightOperandIsInverted) {
    return outInverse[(int) op][leftOperandIsInverted][rightOperandIsInverted];
}


namespace PathOpsV0Lib {    // !!! move to new PathOps.h
void Add(AddCurve , AddWinding );
}

#include "curves/Line.h"
#include "curves/NoCurve.h"
#include "curves/QuadBezier.h"
#include "curves/ConicBezier.h"
#include "curves/CubicBezier.h"
#include "curves/BinaryWinding.h"
#include "curves/UnaryWinding.h"

#include "PathOps.h"

using namespace PathOpsV0Lib;

struct SkiaSimplifyContourData {
    OP_DEBUG_CODE(const SkPath* pathPtr);
	OP_DEBUG_IMAGE_CODE(bool drawNativePath;)  // ; inside required by clang
};

struct SkiaOpContourData {
    BinaryOpData data;
    OP_DEBUG_CODE(const SkPath* pathPtr);
	OP_DEBUG_IMAGE_CODE(bool drawNativePath;)  // ; inside required by clang
};

void commonOutput(Curve c, SkPath::Verb type, bool firstPt, bool lastPt, PathOutput output) {
    SkPath& skpath = *(SkPath*)(output);
    if (firstPt) {
		skpath.setFillType(SkPathFillType::kEvenOdd);
        skpath.moveTo(c.data->start.x, c.data->start.y);
	}
    switch (type) {
        case SkPath::kLine_Verb:
            skpath.lineTo(c.data->end.x, c.data->end.y);
            break;
        case SkPath::kQuad_Verb: {
            OpPoint ctrlPt = quadControlPt(c);
            skpath.quadTo(ctrlPt.x, ctrlPt.y, c.data->end.x, c.data->end.y);
            break;
        }
        case SkPath::kConic_Verb: {
            PointWeight ctrl(c);
            skpath.conicTo(ctrl.pt.x, ctrl.pt.y, c.data->end.x, c.data->end.y, ctrl.weight);
            break;
        }
        case SkPath::kCubic_Verb: {
            CubicControls ctrls(c);
            skpath.cubicTo(ctrls.pts[0].x, ctrls.pts[0].y, ctrls.pts[1].x, ctrls.pts[1].y,
                    c.data->end.x, c.data->end.y);
            break;
        }
        default:
            OP_ASSERT(0);
    }
    if (lastPt)
        skpath.close();
}

void skiaLineOutput(Curve c, bool firstPt, bool lastPt, PathOutput output) {
    commonOutput(c, SkPath::kLine_Verb, firstPt, lastPt, output);
}

void skiaQuadOutput(Curve c, bool firstPt, bool lastPt, PathOutput output) {
    commonOutput(c, SkPath::kQuad_Verb, firstPt, lastPt, output);
}

void skiaConicOutput(Curve c, bool firstPt, bool lastPt, PathOutput output) {
    commonOutput(c, SkPath::kConic_Verb, firstPt, lastPt, output);
}

void skiaCubicOutput(Curve c, bool firstPt, bool lastPt, PathOutput output) {
    commonOutput(c, SkPath::kCubic_Verb, firstPt, lastPt, output);
}

#if OP_DEBUG
void debugCommonScale(Curve curve, int extra, double scale, double offsetX, double offsetY) {
	auto scaler = [scale, offsetX, offsetY](OpPoint& pt) {
		pt.x = (float) (pt.x * scale + offsetX);
		pt.y = (float) (pt.y * scale + offsetY);
	};
	scaler(curve.data->start);
	scaler(curve.data->end);
	if (1 == extra) {
        OpPoint ctrlPt = quadControlPt(curve);
		scaler(ctrlPt);
		quadSetControl(curve, ctrlPt);
	} else if (2 == extra) {
        CubicControls ctrls(curve);
		scaler(ctrls.pts[0]);
		scaler(ctrls.pts[1]);
		ctrls.copyTo(curve);
	}
}

void debugLineScale(Curve curve, double scale, double offsetX, double offsetY) {
	debugCommonScale(curve, 0, scale, offsetX, offsetY);
}

void debugQuadScale(Curve curve, double scale, double offsetX, double offsetY) {
	debugCommonScale(curve, 1, scale, offsetX, offsetY);
}

void debugConicScale(Curve curve, double scale, double offsetX, double offsetY) {
	debugCommonScale(curve, 1, scale, offsetX, offsetY);
}

void debugCubicScale(Curve curve, double scale, double offsetX, double offsetY) {
	debugCommonScale(curve, 2, scale, offsetX, offsetY);
}
#endif

enum class SkiaCurveType : int {
	skiaLineType = 1,
	skiaQuadType,
	skiaConicType,
	skiaCubicType
};

// start here;
// rearrange to allow nullptr as default
void SetSkiaCurveCallBacks(Context* context) {
    OP_DEBUG_CODE(CurveType lineType =) SetCurveCallBacks(context, lineAxisT, nullptr, 
			nullptr, nullptr, nullptr, skiaLineOutput, nullptr, 
			nullptr, lineTangent, nullptr, linePtAtT, nullptr, nullptr, 
			nullptr, lineXYAtT, lineCut, lineNormalLimit, lineInterceptLimit);
	OP_DEBUG_CODE(SetDebugCurveCallBacks(context, lineType, debugLineScale
            OP_DEBUG_DUMP_PARAMS(lineDebugDumpName, noDumpCurveExtra)
            OP_DEBUG_IMAGE_PARAMS(debugLineAddToSkPath)));
	OP_ASSERT((int) lineType == (int) SkiaCurveType::skiaLineType);
    OP_DEBUG_CODE(CurveType quadType =) SetCurveCallBacks(context, quadAxisT, quadHull, 
			quadIsFinite, quadIsLine, quadSetBounds, skiaQuadOutput, quadPinCtrl, 
            nullptr, quadTangent, quadsEqual, quadPtAtT, quadHullPtCount, quadRotate, 
			quadSubDivide, quadXYAtT, lineCut, lineNormalLimit, lineInterceptLimit);
	OP_DEBUG_CODE(SetDebugCurveCallBacks(context, quadType, debugQuadScale
            OP_DEBUG_DUMP_PARAMS(quadDebugDumpName, noDumpCurveExtra)
            OP_DEBUG_IMAGE_PARAMS(debugQuadAddToSkPath)));
	OP_ASSERT((int) quadType == (int) SkiaCurveType::skiaQuadType);
    OP_DEBUG_CODE(CurveType conicType =) SetCurveCallBacks(context, conicAxisT, conicHull, 
            conicIsFinite, conicIsLine, conicSetBounds, skiaConicOutput, quadPinCtrl, 
			nullptr, conicTangent, conicsEqual, conicPtAtT, quadHullPtCount, conicRotate, 
			conicSubDivide, conicXYAtT, lineCut, lineNormalLimit, lineInterceptLimit);
	OP_DEBUG_CODE(SetDebugCurveCallBacks(context, conicType, debugConicScale
            OP_DEBUG_DUMP_PARAMS(conicDebugDumpName, conicDebugDumpExtra)
            OP_DEBUG_IMAGE_PARAMS(debugConicAddToSkPath)));
	OP_ASSERT((int) conicType == (int) SkiaCurveType::skiaConicType);
    OP_DEBUG_CODE(CurveType cubicType =) SetCurveCallBacks(context, cubicAxisT, cubicHull, 
            cubicIsFinite, cubicIsLine, cubicSetBounds, skiaCubicOutput, cubicPinCtrl, 
			cubicReverse, cubicTangent, cubicsEqual, cubicPtAtT, cubicHullPtCount, cubicRotate, 
			cubicSubDivide, cubicXYAtT, lineCut, lineNormalLimit, lineInterceptLimit);
	OP_DEBUG_CODE(SetDebugCurveCallBacks(context, cubicType, debugCubicScale
            OP_DEBUG_DUMP_PARAMS(cubicDebugDumpName, noDumpCurveExtra)
            OP_DEBUG_IMAGE_PARAMS(debugCubicAddToSkPath)));
	OP_ASSERT((int) cubicType == (int) SkiaCurveType::skiaCubicType);
}

#if !OP_TINY_SKIA
#include "include/core/SkStream.h"

std::string dumpSkPath(const SkPath* path, bool inHex) {
    SkDynamicMemoryWStream memoryStream;
    path->dump(&memoryStream, inHex);
    std::string str;
    str.resize(memoryStream.bytesWritten());
    memoryStream.copyTo(str.data());
    str.pop_back();
    return str;
}
#endif

#if OP_DEBUG_DUMP
std::string unaryDumpFunc(CallerData caller, DebugLevel debugLevel, DebugBase debugBase) {
#if OP_TINY_SKIA
    return "";
#else
    OP_ASSERT(sizeof(SkiaSimplifyContourData) == caller.size);
    SkiaSimplifyContourData simplifyUserData;
    std::memcpy(&simplifyUserData, caller.data, caller.size);
    std::string s = dumpSkPath(simplifyUserData.pathPtr, DebugBase::hex == debugBase) + "\n";
    return s;
#endif
}

std::string binaryDumpFunc(CallerData caller, DebugLevel debugLevel, DebugBase debugBase) {
#if OP_TINY_SKIA
    return "";
#else
    OP_ASSERT(sizeof(SkiaOpContourData) == caller.size);
    SkiaOpContourData opUserData;
    std::memcpy(&opUserData, caller.data, caller.size);
    std::string s = dumpSkPath(opUserData.pathPtr, DebugBase::hex == debugBase) + "\n";
    std::vector<std::string> skPathOpNames { "Difference", "Intersect", "Union",  "XOR",
        "ReverseDifference"  };
    OP_ASSERT(BinaryOperation::Difference <= opUserData.data.operation 
            && opUserData.data.operation <= BinaryOperation::ReverseDifference);
    s += "SkPathOp:" + skPathOpNames[(int) opUserData.data.operation] + " ";
    OP_ASSERT(BinaryOperand::left == opUserData.data.operand ||
            BinaryOperand::right == opUserData.data.operand);
    s += "BinaryOperand:" 
            + std::string(BinaryOperand::left == opUserData.data.operand ? "left" : "right");
    return s;
#endif
}
#endif

#if OP_DEBUG_IMAGE
void* debugSimplifyPathFunc(CallerData data) {
    SkiaSimplifyContourData simplifyContourData;
    OP_ASSERT(sizeof(simplifyContourData) == data.size);
    std::memcpy(&simplifyContourData, data.data, data.size);
    return (void*) simplifyContourData.pathPtr;
}

bool debugSimplifyGetDrawFunc(CallerData data) {
    SkiaSimplifyContourData simplifyContourData;
    OP_ASSERT(sizeof(simplifyContourData) == data.size);
    std::memcpy(&simplifyContourData, data.data, data.size);
    return simplifyContourData.drawNativePath;
}

void debugSimplifySetDrawFunc(CallerData data, bool draw) {
    SkiaSimplifyContourData simplifyContourData;
    OP_ASSERT(sizeof(simplifyContourData) == data.size);
    std::memcpy(&simplifyContourData, data.data, data.size);
    simplifyContourData.drawNativePath = draw;
    std::memcpy(data.data, &simplifyContourData, data.size);
}

void* debugOpPathFunc(CallerData data) {
    SkiaOpContourData opContourData;
    OP_ASSERT(sizeof(opContourData) == data.size);
    std::memcpy(&opContourData, data.data, data.size);
    return (void*) opContourData.pathPtr;
}

bool debugOpGetDrawFunc(CallerData data) {
    SkiaOpContourData opContourData;
    OP_ASSERT(sizeof(opContourData) == data.size);
    std::memcpy(&opContourData, data.data, data.size);
    return opContourData.drawNativePath;
}

void debugOpSetDrawFunc(CallerData data, bool draw) {
    SkiaOpContourData opContourData;
    OP_ASSERT(sizeof(opContourData) == data.size);
    std::memcpy(&opContourData, data.data, data.size);
    opContourData.drawNativePath = draw;
    std::memcpy(data.data, &opContourData, data.size);
}

inline bool  debugOpSetIsOppFunc(CallerData data) {
    SkiaOpContourData opContourData;
    OP_ASSERT(sizeof(opContourData) == data.size);
    std::memcpy(&opContourData, data.data, data.size);
    return BinaryOperand::left != opContourData.data.operand;
}


#endif

void emptySkPathFunc(PathOutput output) {
    SkPath* skOutput = (SkPath*) output;
    skOutput->reset();
	skOutput->setFillType(SkPathFillType::kEvenOdd);
}

Curve skiaMakeLine(Curve c) {
    c.type = (CurveType) SkiaCurveType::skiaLineType;
    c.size = sizeof(SkPoint) * 2;
    return c;
}

PathOpsV0Lib::CurveType setSkiaLineType(PathOpsV0Lib::Curve ) {
    return (CurveType) SkiaCurveType::skiaLineType;
}

#if OP_DEBUG
// 0x00 (black) is 'on' ; 0xFF (white) is 'off' -- this reverses the intuitive operators
uint8_t skiaDebugBitOper(CallerData data, uint8_t src, uint8_t opp) {
    SkiaOpContourData opContourData;
    OP_ASSERT(sizeof(opContourData) == data.size);
    std::memcpy(&opContourData, data.data, data.size);
//	uint8_t constexpr blackBit = 0x00;	// aide memoire
	uint8_t constexpr whiteBit = 0xFF;
	switch (opContourData.data.operation) {
		case BinaryOperation::Difference:
			return src ? whiteBit : ~opp;
		case BinaryOperation::Intersect:
			return src | opp;
		case BinaryOperation::Union:
			return src & opp;
		case BinaryOperation::ExclusiveOr:
			return ~(src ^ opp);
		case BinaryOperation::ReverseDifference:
			return opp ? whiteBit : ~src;
	}
	OP_ASSERT(0);
	return 0;
}
#endif

#if OP_DEBUG
#include "OpSkiaTests.h"
#endif

void SetSkiaContextCallBacks(Context* context) {
#if OP_DEBUG && TEST_ANALYZE
	extern bool DebugAnalyze(Context* );
	if (DebugAnalyze(context))  // definition below
		return;
#endif
    SetContextCallBacks(context, emptySkPathFunc, skiaMakeLine, setSkiaLineType, maxSignSwap,
			maxDepth, maxSplits, maxLimbs);
}

Contour* SetSkiaSimplifyCallBacks(Context* context, 
        bool isWindingFill  OP_DEBUG_PARAMS(const SkPath& path)) {
    SkiaSimplifyContourData simplifyUserData { OP_DEBUG_CODE(&path)  OP_DEBUG_IMAGE_PARAMS(true) };
    Contour* contour = CreateContour({context, &simplifyUserData, 
            sizeof(SkiaSimplifyContourData) } );
    WindingAdd addFunc = isWindingFill ? unaryWindingAddFunc : unaryEvenOddFunc;
    WindingAdd subtractFunc = isWindingFill ? unaryWindingSubtractFunc : unaryEvenOddFunc;
    SetWindingCallBacks(contour, addFunc, unaryWindingKeepFunc, 
            subtractFunc, unaryWindingVisibleFunc, unaryWindingZeroFunc 
			OP_DEBUG_PARAMS(skiaDebugBitOper)
            OP_DEBUG_DUMP_PARAMS(unaryWindingDumpInFunc, unaryWindingDumpOutFunc, unaryDumpFunc)
            OP_DEBUG_IMAGE_PARAMS(unaryWindingImageOutFunc, debugSimplifyPathFunc,
	                debugSimplifyGetDrawFunc, debugSimplifySetDrawFunc, noIsOppFunc)
    );
    return contour;
}

Contour* SetSkiaOpCallBacks(Context* context, SkPathOp op,
        BinaryOperand operand, BinaryWindType windType  OP_DEBUG_PARAMS(const SkPath& path)) {
    SkiaOpContourData windingUserData { { (BinaryOperation) op, operand }
            OP_DEBUG_PARAMS(&path )  OP_DEBUG_IMAGE_PARAMS(true) };
    Contour* contour = CreateContour({context, (ContourData*) &windingUserData,
            sizeof(SkiaOpContourData) } );
    WindingKeep operatorFunc = noWindKeepFunc;
    switch (op) {
        case kDifference_SkPathOp: operatorFunc = binaryWindingDifferenceFunc; break;
        case kIntersect_SkPathOp: operatorFunc = binaryWindingIntersectFunc; break;
        case kUnion_SkPathOp: operatorFunc = binaryWindingUnionFunc; break;
        case kXOR_SkPathOp: operatorFunc = binaryWindingExclusiveOrFunc; break;
        case kReverseDifference_SkPathOp: operatorFunc = binaryWindingReverseDifferenceFunc; break;
        default: OP_ASSERT(0);
    }
    WindingAdd addFunc = noWindingOpFunc;
    switch (windType) {
        case BinaryWindType::evenOdd: addFunc = binaryEvenOddFunc; break;
        case BinaryWindType::windLeft: addFunc = binaryWindingAddLeftFunc; break;
        case BinaryWindType::windRight: addFunc = binaryWindingAddRightFunc; break;
        case BinaryWindType::windBoth: addFunc = binaryWindingAddFunc; break;
        default: OP_ASSERT(0);
    }
    WindingAdd subtractFunc = noWindingOpFunc;
    switch (windType) {
        case BinaryWindType::evenOdd: subtractFunc = binaryEvenOddFunc; break;
        case BinaryWindType::windLeft: subtractFunc = binaryWindingSubtractLeftFunc; break;
        case BinaryWindType::windRight: subtractFunc = binaryWindingSubtractRightFunc; break;
        case BinaryWindType::windBoth: subtractFunc = binaryWindingSubtractFunc; break;
        default: OP_ASSERT(0);
    }
    SetWindingCallBacks(contour, addFunc, operatorFunc, 
            subtractFunc, binaryWindingVisibleFunc, binaryWindingZeroFunc 
			OP_DEBUG_PARAMS(skiaDebugBitOper)
			OP_DEBUG_DUMP_PARAMS(binaryWindingDumpInFunc, binaryWindingDumpOutFunc, binaryDumpFunc)
            OP_DEBUG_IMAGE_PARAMS(binaryWindingImageOutFunc, debugOpPathFunc,
	                debugOpGetDrawFunc, debugOpSetDrawFunc, debugOpSetIsOppFunc)
    );
    return contour;
}

void AddSkiaPath(Context* context, AddWinding winding, const SkPath& path) {
	if (!path.isFinite()) {  // raw iter treats non-finite path as empty
		SetError(context, ContextError::finite);
		return;
	}
    SkPath::RawIter iter(path);
    OpPoint closeLine[2] = {{0, 0}, {0, 0}};  // initialize so first move doesn't add close line
    for (;;) {
        SkPoint pts[4];
        SkPath::Verb verb = iter.next(pts);
        switch (verb) {
        case SkPath::kMove_Verb:
            if (closeLine[0] != closeLine[1])
                Add({ closeLine, sizeof(closeLine), 
						(CurveType) SkiaCurveType::skiaLineType }, winding);
            closeLine[1] = { pts[0].fX, pts[0].fY };
            pts[1] = pts[0];
            break;
        case SkPath::kLine_Verb:
            if (pts[0] != pts[1])
                Add({ (OpPoint*) pts, sizeof(SkPoint) * 2, 
						(CurveType) SkiaCurveType::skiaLineType }, winding);
            break;
        case SkPath::kQuad_Verb:
            std::swap(pts[1], pts[2]);  // rearrange order from 0/1/2 to 0/2/1
            AddQuads({ (OpPoint*) pts, sizeof(SkPoint) * 3, 
					(CurveType) SkiaCurveType::skiaQuadType }, winding);
            break;
        case SkPath::kConic_Verb:
            std::swap(pts[1], pts[2]);  // rearrange order from 0/1/2 to 0/2/1
            pts[3].fX = iter.conicWeight(); // !!! hacky
            AddConics({ (OpPoint*) pts, sizeof(SkPoint) * 3 + sizeof(float), 
                    (CurveType) SkiaCurveType::skiaConicType }, winding);
            break;
        case SkPath::kCubic_Verb: {
		#if 0
			// This fails in GCC 13.2 release. It works in MSVS Visual C++ 2022 debug/release,
			// clang debug/release, GCC 13.2 debug. Replacing std::swap with temp=a, a=b, b=temp
			// also fails. Changing the indices also fails.
            std::swap(pts[1], pts[2]);  // rearrange order from 0/1/2/3 to 0/3/1/2
            std::swap(pts[1], pts[3]);
		#else
			SkPoint temp[4] { pts[0], pts[3], pts[1], pts[2] };  //  put start, end, up front
			std::memcpy(pts, temp, sizeof(temp));
		#endif
            AddCubics({ (OpPoint*) pts, sizeof(SkPoint) * 4, 
					(CurveType) SkiaCurveType::skiaCubicType }, winding);
            } break;
        case SkPath::kClose_Verb:
        case SkPath::kDone_Verb:
            if (closeLine[0] != closeLine[1])
                Add({ closeLine, sizeof(closeLine), 
						(CurveType) SkiaCurveType::skiaLineType }, winding);
            if (SkPath::kDone_Verb == verb)
                return;
            closeLine[0] = closeLine[1];
            continue;
        default:
            OP_ASSERT(0);
        }
        closeLine[0] = { pts[1].fX, pts[1].fY };
    }
}

#if OP_DEBUG_IMAGE

void debugLineAddToSkPath(Curve c, SkPath& path) {
	path.lineTo(c.data->end.x, c.data->end.y);
}

void debugQuadAddToSkPath(Curve c, SkPath& path) {
    OpPoint controlPt = *(OpPoint*) CurveUserData(c.data);
	path.quadTo(controlPt.x, controlPt.y, c.data->end.x, c.data->end.y);
}

void debugConicAddToSkPath(Curve c, SkPath& path) {
    PointWeight control = *(PointWeight*) CurveUserData(c.data);
	path.conicTo(control.pt.x, control.pt.y, c.data->end.x, c.data->end.y, control.weight);
}

void debugCubicAddToSkPath(Curve c, SkPath& path) {
    OpPoint* controls = (OpPoint*) CurveUserData(c.data);
	path.cubicTo(controls[0].x, controls[0].y, controls[1].x, controls[1].y, 
            c.data->end.x, c.data->end.y);
}

#endif

#if OP_DEBUG
#if TEST_ANALYZE
#include "OpContour.h"

inline int minMaxLimbs(Context* ) {
	return 120;
}

bool DebugAnalyze(Context* context) {
	OpContours* contours = (OpContours*) context;
	OpDebugData& debugData = contours->debugData;
	if (debugData.limitContours <= 0)
		return false;
    SetContextCallBacks(context, emptySkPathFunc, skiaMakeLine, setSkiaLineType, maxSignSwap,
			maxDepth, maxSplits, minMaxLimbs);
	return true;
}

void AddDebugSkiaPath(Context* context, AddWinding winding, const SkPath& path) {
	OpContours* contours = (OpContours*) context;
	OpDebugData& debugData = contours->debugData;
	OpPointBounds snag { 20, 0, 40, 10 };  // only snag contours that start in this bounds
	bool snagOn = false;
	int contourCount = 0;
	if (!path.isFinite()) {  // raw iter treats non-finite path as empty
		SetError(context, ContextError::finite);
		return;
	}
    SkPath::RawIter iter(path);
    OpPoint closeLine[2] = {{0, 0}, {0, 0}};  // initialize so first move doesn't add close line
    for (;;) {
        SkPoint pts[4];
        SkPath::Verb verb = iter.next(pts);
        switch (verb) {
        case SkPath::kMove_Verb:
            if (closeLine[0] != closeLine[1]) {
                if (snagOn) Add({ closeLine, sizeof(closeLine), 
						(CurveType) SkiaCurveType::skiaLineType }, winding);
				if (++contourCount >= debugData.limitContours)
					return;
			}			
            closeLine[1] = { pts[0].fX, pts[0].fY };
			snagOn = snag.contains(closeLine[1]);
            pts[1] = pts[0];
            break;
        case SkPath::kLine_Verb:
            if (pts[0] != pts[1])
                if (snagOn) Add({ (OpPoint*) pts, sizeof(SkPoint) * 2, 
						(CurveType) SkiaCurveType::skiaLineType }, winding);
            break;
        case SkPath::kQuad_Verb:
            std::swap(pts[1], pts[2]);  // rearrange order from 0/1/2 to 0/2/1
            if (snagOn) AddQuads({ (OpPoint*) pts, sizeof(SkPoint) * 3, 
					(CurveType) SkiaCurveType::skiaQuadType }, winding);
            break;
        case SkPath::kConic_Verb:
            std::swap(pts[1], pts[2]);  // rearrange order from 0/1/2 to 0/2/1
            pts[3].fX = iter.conicWeight(); // !!! hacky
            if (snagOn) AddConics({ (OpPoint*) pts, sizeof(SkPoint) * 3 + sizeof(float), 
                    (CurveType) SkiaCurveType::skiaConicType }, winding);
            break;
        case SkPath::kCubic_Verb: {
		#if 0
			// This fails in GCC 13.2 release. It works in MSVS Visual C++ 2022 debug/release,
			// clang debug/release, GCC 13.2 debug. Replacing std::swap with temp=a, a=b, b=temp
			// also fails. Changing the indices also fails.
            std::swap(pts[1], pts[2]);  // rearrange order from 0/1/2/3 to 0/3/1/2
            std::swap(pts[1], pts[3]);
		#else
			SkPoint temp[4] { pts[0], pts[3], pts[1], pts[2] };  //  put start, end, up front
			std::memcpy(pts, temp, sizeof(temp));
		#endif
            if (snagOn) AddCubics({ (OpPoint*) pts, sizeof(SkPoint) * 4, 
					(CurveType) SkiaCurveType::skiaCubicType }, winding);
            } break;
        case SkPath::kClose_Verb:
        case SkPath::kDone_Verb:
            if (closeLine[0] != closeLine[1])
                if (snagOn) Add({ closeLine, sizeof(closeLine), 
						(CurveType) SkiaCurveType::skiaLineType }, winding);
			if (++contourCount >= debugData.limitContours)
				return;
            if (SkPath::kDone_Verb == verb) {
				debugData.limitReached = true;
                return;
			}
            closeLine[0] = closeLine[1];
            continue;
        default:
            OP_ASSERT(0);
        }
        closeLine[0] = { pts[1].fX, pts[1].fY };
    }
}

#endif
#endif
