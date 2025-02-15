// (c) 2023, Cary Clark cclark2@gmail.com
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

#include "curves/Line.h"
#include "curves/QuadBezier.h"
#include "curves/ConicBezier.h"
#include "curves/CubicBezier.h"
#include "curves/BinaryWinding.h"
#include "curves/UnaryWinding.h"

#include "PathOps.h"

using namespace PathOpsV0Lib;

#if OP_DEBUG
struct UnaryContour {
    const SkPath* pathPtr;
    int contourIndex;
	bool drawNativePath;
};

struct BinaryContour : public UnaryContour {
	BinaryOperand operand;
};

struct UnaryContext {
};

struct BinaryContext : public UnaryContext {
	BinaryOperation operation;
};
#endif

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

enum class SkiaCurveType : int {
	skiaLineType = 1,
	skiaQuadType,
	skiaConicType,
	skiaCubicType
};

// start here;
// rearrange to allow nullptr as default
void SetSkiaCurveCallbacks(Context* context) {
    OP_DEBUG_CODE(CurveType lineType =) SetCurveCallbacks(context, { skiaLineOutput });
	OP_DEBUG_CODE(SetDebugCurveCallbacks(context, lineType, { debugLineScale
            OP_DEBUG_DUMP_PARAMS(lineDebugDumpName, nullptr)
            OP_DEBUG_IMAGE_PARAMS(debugLineAddToSkPath) }));
	OP_ASSERT((int) lineType == (int) SkiaCurveType::skiaLineType);
    OP_DEBUG_CODE(CurveType quadType =) SetCurveCallbacks(context, { skiaQuadOutput, quadAxisT,
			quadHull, quadIsFinite, quadIsLine, quadSetBounds, quadPinCtrl, 
            quadTangent, quadsEqual, quadPtAtT, quadHullPtCount, quadRotate, 
			quadSubDivide, quadXYAtT });
	OP_DEBUG_CODE(SetDebugCurveCallbacks(context, quadType, { debugQuadScale
            OP_DEBUG_DUMP_PARAMS(quadDebugDumpName, nullptr)
            OP_DEBUG_IMAGE_PARAMS(debugQuadAddToSkPath) }));
	OP_ASSERT((int) quadType == (int) SkiaCurveType::skiaQuadType);
    OP_DEBUG_CODE(CurveType conicType =) SetCurveCallbacks(context, { skiaConicOutput, conicAxisT,
			conicHull, conicIsFinite, conicIsLine, conicSetBounds, quadPinCtrl, 
			conicTangent, conicsEqual, conicPtAtT, quadHullPtCount, conicRotate, 
			conicSubDivide, conicXYAtT });
	OP_DEBUG_CODE(SetDebugCurveCallbacks(context, conicType, { debugConicScale
            OP_DEBUG_DUMP_PARAMS(conicDebugDumpName, conicDebugDumpExtra)
            OP_DEBUG_IMAGE_PARAMS(debugConicAddToSkPath) }));
	OP_ASSERT((int) conicType == (int) SkiaCurveType::skiaConicType);
    OP_DEBUG_CODE(CurveType cubicType =) SetCurveCallbacks(context, { skiaCubicOutput, cubicAxisT,
			cubicHull, cubicIsFinite, cubicIsLine, cubicSetBounds, cubicPinCtrl, 
			cubicTangent, cubicsEqual, cubicPtAtT, cubicHullPtCount, cubicRotate, 
			cubicSubDivide, cubicXYAtT, cubicReverse });
	OP_DEBUG_CODE(SetDebugCurveCallbacks(context, cubicType, { debugCubicScale
            OP_DEBUG_DUMP_PARAMS(cubicDebugDumpName, nullptr)
            OP_DEBUG_IMAGE_PARAMS(debugCubicAddToSkPath) }));
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


std::string dumpSkContour(const SkPath* path, int contour, bool inHex) {
	std::string s = dumpSkPath(path, inHex);
	size_t pos = 0;
	while (contour > 0) {
		pos = s.find("path.close();", pos);
		pos = s.find("path.moveTo(", pos);
		--contour;
	}
	size_t endPos = s.find("path.close();", pos);
	endPos = s.find("path.moveTo(", endPos);
	if (std::string::npos == endPos)
		endPos = s.size();
	return s.substr(pos, endPos);
}
#endif

#if OP_DEBUG_DUMP
std::string dumpUnaryContourFunc(DebugContourData caller, DebugLevel , DebugBase debugBase) {
#if OP_TINY_SKIA
    return "";
#else
    OP_ASSERT(sizeof(UnaryContour) <= caller.size);
    UnaryContour callerData;
    std::memcpy(&callerData, caller.data, caller.size);
    return dumpSkContour(callerData.pathPtr, callerData.contourIndex,
			DebugBase::hex == debugBase) + "\n";
#endif
}

std::string dumpBinaryContourFunc(DebugContourData caller, DebugLevel l, DebugBase b) {
	std::string s = dumpUnaryContourFunc(caller, l, b);
#if !OP_TINY_SKIA
    BinaryContour callerData;
    std::memcpy(&callerData, caller.data, caller.size);
    OP_ASSERT(BinaryOperand::left == callerData.operand ||
            BinaryOperand::right == callerData.operand);
    s += "BinaryOperand:" 
            + std::string(BinaryOperand::left == callerData.operand ? "left" : "right");
#endif
    return s;
}

std::string dumpUnaryContextFunc(DebugContextData caller, DebugLevel l, DebugBase b) {
#if OP_TINY_SKIA
    return "";
#else
    OP_ASSERT(sizeof(UnaryContext) <= caller.size);
    UnaryContext callerData;
    std::memcpy(&callerData, caller.data, caller.size);
	// !!!ignore draw native path?
#endif
	return "";
}

std::string dumpBinaryContextFunc(DebugContextData caller, DebugLevel l, DebugBase b) {
#if OP_TINY_SKIA
    return "";
#else
    std::vector<std::string> skPathOpNames { "Difference", "Intersect", "Union",  "XOR",
        "ReverseDifference"  };
    BinaryContext callerData;
    std::memcpy(&callerData, caller.data, caller.size);
    OP_ASSERT(BinaryOperation::Difference <= callerData.operation 
            && callerData.operation <= BinaryOperation::ReverseDifference);
    std::string s = "SkPathOp:" + skPathOpNames[(int) callerData.operation] + " ";
    return s;
#endif
}
#endif

#if OP_DEBUG_IMAGE
void* debugSimplifyPathFunc(DebugContourData data) {
    UnaryContour simplifyContourData;
    OP_ASSERT(sizeof(simplifyContourData) == data.size);
    std::memcpy(&simplifyContourData, data.data, data.size);
    return (void*) simplifyContourData.pathPtr;
}

bool debugSimplifyGetDrawFunc(DebugContourData data) {
    UnaryContour simplifyContourData;
    OP_ASSERT(sizeof(simplifyContourData) == data.size);
    std::memcpy(&simplifyContourData, data.data, data.size);
    return simplifyContourData.drawNativePath;
}

void debugSimplifySetDrawFunc(DebugContourData data, bool draw) {
    UnaryContour simplifyContourData;
    OP_ASSERT(sizeof(simplifyContourData) == data.size);
    std::memcpy(&simplifyContourData, data.data, data.size);
    simplifyContourData.drawNativePath = draw;
    std::memcpy(data.data, &simplifyContourData, data.size);
}

void* debugOpPathFunc(DebugContourData data) {
    BinaryContour opContourData;
    OP_ASSERT(sizeof(opContourData) == data.size);
    std::memcpy(&opContourData, data.data, data.size);
    return (void*) opContourData.pathPtr;
}

bool debugOpGetDrawFunc(DebugContourData data) {
    BinaryContour opContourData;
    OP_ASSERT(sizeof(opContourData) == data.size);
    std::memcpy(&opContourData, data.data, data.size);
    return opContourData.drawNativePath;
}

void debugOpSetDrawFunc(DebugContourData data, bool draw) {
    BinaryContour opContourData;
    OP_ASSERT(sizeof(opContourData) == data.size);
    std::memcpy(&opContourData, data.data, data.size);
    opContourData.drawNativePath = draw;
    std::memcpy(data.data, &opContourData, data.size);
}

inline bool debugOpSetIsOppFunc(DebugContourData data, int opp) {
    BinaryContour opContourData;
    OP_ASSERT(sizeof(opContourData) == data.size);
    std::memcpy(&opContourData, data.data, data.size);
    return BinaryOperand::left != opContourData.operand;
}
#endif

void emptySkPathFunc(PathOutput output) {
    SkPath* skOutput = (SkPath*) output;
    skOutput->reset();
	skOutput->setFillType(SkPathFillType::kEvenOdd);
}

PathOpsV0Lib::CurveType setSkiaLineType(PathOpsV0Lib::Curve ) {
    return (CurveType) SkiaCurveType::skiaLineType;
}

#if OP_DEBUG
// 0x00 (black) is 'on' ; 0xFF (white) is 'off' -- this reverses the intuitive operators
uint8_t skiaDebugBitOper(DebugContourData data, uint8_t src, uint8_t opp) {
    BinaryContext opContextData;
    OP_ASSERT(sizeof(opContextData) == data.size);
    std::memcpy(&opContextData, data.data, data.size);
//	uint8_t constexpr blackBit = 0x00;	// aide memoire
	uint8_t constexpr whiteBit = 0xFF;
	switch (opContextData.operation) {
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

void SetSkiaContextCallbacks(Context* context) {
#if OP_DEBUG && TEST_ANALYZE
	extern bool DebugAnalyze(Context* );
	if (DebugAnalyze(context))  // definition below
		return;
#endif
    SetContextCallbacks(context, { setSkiaLineType, emptySkPathFunc });
}

Contour* SetSkiaSimplifyCallbacks(Context* context, Winding winding,
        bool isWindingFill  OP_DEBUG_PARAMS(const SkPath& path)) {
    Contour* contour = CreateContour(context, winding);
    WindingAdd addFunc = isWindingFill ? unaryWindingAddFunc : unaryEvenOddFunc;
    WindingAdd subtractFunc = isWindingFill ? unaryWindingSubtractFunc : unaryEvenOddFunc;
    SetWindingCallbacks(context, { addFunc, unaryWindingKeepFunc, unaryWindingVisibleFunc, 
			unaryWindingZeroFunc, subtractFunc });

#if OP_DEBUG
    UnaryContour simplifyUserData { &path, 0, true };
	SetDebugContourData(contour, { &simplifyUserData, sizeof(simplifyUserData) } );
	SetDebugContourCallbacks(contour, { 
            OP_DEBUG_DUMP_CODE(dumpUnaryContourFunc)
            OP_DEBUG_IMAGE_PARAMS(debugSimplifyPathFunc,
	        debugSimplifyGetDrawFunc, debugSimplifySetDrawFunc) }
    );
#endif
    return contour;
}

void SetSkiaOpContextCallbacks(Context* context, SkPathOp op, BinaryWindType windType) {
    WindingKeep operatorFunc = nullptr;
    switch (op) {
        case kDifference_SkPathOp: operatorFunc = binaryWindingDifferenceFunc; break;
        case kIntersect_SkPathOp: operatorFunc = binaryWindingIntersectFunc; break;
        case kUnion_SkPathOp: operatorFunc = binaryWindingUnionFunc; break;
        case kXOR_SkPathOp: operatorFunc = binaryWindingExclusiveOrFunc; break;
        case kReverseDifference_SkPathOp: operatorFunc = binaryWindingReverseDifferenceFunc; break;
        default: OP_ASSERT(0);
    }
    WindingAdd addFunc = nullptr;
    switch (windType) {
        case BinaryWindType::evenOdd: addFunc = binaryEvenOddFunc; break;
        case BinaryWindType::windLeft: addFunc = binaryWindingAddLeftFunc; break;
        case BinaryWindType::windRight: addFunc = binaryWindingAddRightFunc; break;
        case BinaryWindType::windBoth: addFunc = binaryWindingAddFunc; break;
        default: OP_ASSERT(0);
    }
    WindingAdd subtractFunc = nullptr;
    switch (windType) {
        case BinaryWindType::evenOdd: break;
        case BinaryWindType::windLeft: subtractFunc = binaryWindingSubtractLeftFunc; break;
        case BinaryWindType::windRight: subtractFunc = binaryWindingSubtractRightFunc; break;
        case BinaryWindType::windBoth: subtractFunc = binaryWindingSubtractFunc; break;
        default: OP_ASSERT(0);
    }
    SetWindingCallbacks(context, { addFunc, operatorFunc, binaryWindingVisibleFunc, 
			binaryWindingZeroFunc, subtractFunc });
#if OP_DEBUG
    BinaryContext windingUserData { {}, (BinaryOperation) op };
	SetDebugContextData(context, { &windingUserData, sizeof(windingUserData) } );
#endif
}

Contour* SetSkiaOpContourCallbacks(Context* context, Winding winding,
        BinaryOperand operand  OP_DEBUG_PARAMS(const SkPath& path)) {
    Contour* contour = CreateContour(context, winding);
#if OP_DEBUG
    BinaryContour windingUserData { { &path, 0, true }, operand };
	SetDebugContourData(contour, { &windingUserData, sizeof(windingUserData) } );
	SetDebugContourCallbacks(contour, {
			OP_DEBUG_DUMP_CODE(dumpBinaryContourFunc)
            OP_DEBUG_IMAGE_PARAMS(debugOpPathFunc,
	        debugOpGetDrawFunc, debugOpSetDrawFunc, debugOpSetIsOppFunc) }
    );
	SetDebugContextCallbacks(context, { skiaDebugBitOper
			OP_DEBUG_DUMP_PARAMS(dumpBinaryContextFunc, binaryWindingDumpOutFunc)
            OP_DEBUG_IMAGE_PARAMS(binaryWindingImageOutFunc) }
    );
#endif
    return contour;
}

void AddSkiaPath(Context* context, Contour* contour, const SkPath& path) {
	if (!path.isFinite()) {  // raw iter treats non-finite path as empty
		SetError(context, ContextError::finite);
		return;
	}
    SkPath::RawIter iter(path);
    OpPoint closeLine[2] = {{0, 0}, {0, 0}};  // initialize so first move doesn't add close line
#if WINDER_CONTOUR_EXPERIMENT
	int contourIndex = 0;
#endif
    for (;;) {
        SkPoint pts[4];
        SkPath::Verb verb = iter.next(pts);
        switch (verb) {
        case SkPath::kMove_Verb:
            if (closeLine[0] != closeLine[1])
                Add(contour, { closeLine, sizeof(closeLine), 
						(CurveType) SkiaCurveType::skiaLineType } );
            closeLine[1] = { pts[0].fX, pts[0].fY };
            pts[1] = pts[0];
	#if WINDER_CONTOUR_EXPERIMENT
			contour = Clone(contour);
	#if OP_DEBUG
		{	DebugContourData debugCaller = GetDebugContourData(contour);
			OP_ASSERT(sizeof(UnaryContour) <= debugCaller.size);
			UnaryContour windingUserData;
			std::memcpy(&windingUserData, debugCaller.data, sizeof(windingUserData));
			windingUserData.contourIndex = contourIndex;
			std::memcpy(debugCaller.data, &windingUserData, sizeof(windingUserData));
		}
	#endif
			++contourIndex;
	#endif
            break;
        case SkPath::kLine_Verb:
            if (pts[0] != pts[1])
                Add(contour, { (OpPoint*) pts, sizeof(SkPoint) * 2, 
						(CurveType) SkiaCurveType::skiaLineType } );
            break;
        case SkPath::kQuad_Verb:
            std::swap(pts[1], pts[2]);  // rearrange order from 0/1/2 to 0/2/1
            AddQuads(contour, { (OpPoint*) pts, sizeof(SkPoint) * 3, 
					(CurveType) SkiaCurveType::skiaQuadType } );
            break;
        case SkPath::kConic_Verb:
            std::swap(pts[1], pts[2]);  // rearrange order from 0/1/2 to 0/2/1
            pts[3].fX = iter.conicWeight(); // !!! hacky
            AddConics(contour, { (OpPoint*) pts, sizeof(SkPoint) * 3 + sizeof(float), 
                    (CurveType) SkiaCurveType::skiaConicType } );
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
            AddCubics(contour, { (OpPoint*) pts, sizeof(SkPoint) * 4, 
					(CurveType) SkiaCurveType::skiaCubicType } );
            } break;
        case SkPath::kClose_Verb:
        case SkPath::kDone_Verb:
            if (closeLine[0] != closeLine[1])
                Add(contour, { closeLine, sizeof(closeLine), 
						(CurveType) SkiaCurveType::skiaLineType } );
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

#if OP_DEBUG
#if TEST_ANALYZE
#include "OpContour.h"

inline int minMaxLimbs(Context* ) {
	return 120;
}

bool DebugAnalyze(Context* context) {
	OpContext* contours = (OpContext*) context;
	OpDebugData& debugData = contours->debugData;
	if (debugData.limitContours <= 0)
		return false;
    SetContextCallbacks(context, { setSkiaLineType, emptySkPathFunc, nullptr,
			nullptr, nullptr, minMaxLimbs });
	return true;
}

void AddDebugSkiaPath(Context* context, Contour* contour, const SkPath& path) {
	OpContext* contours = (OpContext*) context;
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
                if (snagOn) Add(contour, { closeLine, sizeof(closeLine), 
						(CurveType) SkiaCurveType::skiaLineType } );
				if (++contourCount >= debugData.limitContours)
					return;
			}			
            closeLine[1] = { pts[0].fX, pts[0].fY };
			snagOn = snag.contains(closeLine[1]);
            pts[1] = pts[0];
            break;
        case SkPath::kLine_Verb:
            if (pts[0] != pts[1])
                if (snagOn) Add(contour, { (OpPoint*) pts, sizeof(SkPoint) * 2, 
						(CurveType) SkiaCurveType::skiaLineType } );
            break;
        case SkPath::kQuad_Verb:
            std::swap(pts[1], pts[2]);  // rearrange order from 0/1/2 to 0/2/1
            if (snagOn) AddQuads(contour, { (OpPoint*) pts, sizeof(SkPoint) * 3, 
					(CurveType) SkiaCurveType::skiaQuadType } );
            break;
        case SkPath::kConic_Verb:
            std::swap(pts[1], pts[2]);  // rearrange order from 0/1/2 to 0/2/1
            pts[3].fX = iter.conicWeight(); // !!! hacky
            if (snagOn) AddConics(contour, { (OpPoint*) pts, sizeof(SkPoint) * 3 + sizeof(float), 
                    (CurveType) SkiaCurveType::skiaConicType } );
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
            if (snagOn) AddCubics(contour, { (OpPoint*) pts, sizeof(SkPoint) * 4, 
					(CurveType) SkiaCurveType::skiaCubicType } );
            } break;
        case SkPath::kClose_Verb:
        case SkPath::kDone_Verb:
            if (closeLine[0] != closeLine[1])
                if (snagOn) Add(contour, { closeLine, sizeof(closeLine), 
						(CurveType) SkiaCurveType::skiaLineType } );
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
