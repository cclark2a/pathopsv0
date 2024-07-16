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
	OP_DEBUG_IMAGE_CODE(bool drawNativePath);
};

struct SkiaOpContourData {
    BinaryOpData data;
    OP_DEBUG_CODE(const SkPath* pathPtr);
	OP_DEBUG_IMAGE_CODE(bool drawNativePath);
};

void commonOutput(Curve c, SkPath::Verb type, bool firstPt, bool lastPt, PathOutput output) {
    SkPath& skpath = *(SkPath*)(output);
    if (firstPt)
        skpath.moveTo(c.data->start.x, c.data->start.y);
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

OpType skiaLineType;
OpType skiaQuadType;
OpType skiaConicType;
OpType skiaCubicType;

// new interface
void SetSkiaCurveCallBacks(Context* context) {
    skiaLineType = SetCurveCallBacks(context, lineAxisRawHit, noNearly, noHull, lineIsFinite, 
            lineIsLine, noLinear, noBounds, lineNormal, skiaLineOutput, noPinCtrl, noReverse,
            lineTangent, linesEqual, linePtAtT, /* double not required */ linePtAtT, 
            linePtCount, noRotate, lineSubDivide, lineXYAtT
            OP_DEBUG_DUMP_PARAMS(noDumpCurveExtra)
            OP_DEBUG_IMAGE_PARAMS(debugLineAddToSkPath)
    );
    skiaQuadType = SetCurveCallBacks(context, quadAxisRawHit, quadNearly, quadHull, quadIsFinite, 
            quadIsLine, quadIsLinear, quadSetBounds, quadNormal, skiaQuadOutput, quadPinCtrl, 
            noReverse, quadTangent, quadsEqual, quadPtAtT, /* double not required */ quadPtAtT, 
            quadPtCount, quadRotate, quadSubDivide, quadXYAtT
            OP_DEBUG_DUMP_PARAMS(noDumpCurveExtra)
            OP_DEBUG_IMAGE_PARAMS(debugQuadAddToSkPath)
    );
    skiaConicType = SetCurveCallBacks(context, conicAxisRawHit, conicNearly, conicHull, 
            conicIsFinite, conicIsLine, 
            conicIsLinear, conicSetBounds, conicNormal, skiaConicOutput, quadPinCtrl, noReverse,
            conicTangent, conicsEqual, conicPtAtT, /* double not required */ conicPtAtT, 
            conicPtCount, conicRotate, conicSubDivide, conicXYAtT
            OP_DEBUG_DUMP_PARAMS(conicDebugDumpExtra)
            OP_DEBUG_IMAGE_PARAMS(debugConicAddToSkPath)
    );
    skiaCubicType = SetCurveCallBacks(context, cubicAxisRawHit, cubicNearly, cubicHull, 
            cubicIsFinite, cubicIsLine, 
            cubicIsLinear, cubicSetBounds, cubicNormal, skiaCubicOutput, cubicPinCtrl, cubicReverse,
            cubicTangent, cubicsEqual, cubicPtAtT, /* double not required */ cubicPtAtT, 
            cubicPtCount, cubicRotate, cubicSubDivide, cubicXYAtT
            OP_DEBUG_DUMP_PARAMS(noDumpCurveExtra)
            OP_DEBUG_IMAGE_PARAMS(debugCubicAddToSkPath)
    );
}

#if OP_DEBUG_DUMP && !OP_TINY_SKIA
#include "include/core/SkStream.h"

std::string dumpSkPath(const SkPath* path, DebugBase debugBase) {
    SkDynamicMemoryWStream memoryStream;
    path->dump(&memoryStream, DebugBase::hex == debugBase);
    std::string str;
    str.resize(memoryStream.bytesWritten());
    memoryStream.copyTo(str.data());
    str.pop_back();
    return "skPath:" + str;
}

std::string unaryDumpFunc(CallerData caller, DebugLevel debugLevel, DebugBase debugBase) {
    OP_ASSERT(sizeof(SkiaSimplifyContourData) == caller.size);
    SkiaSimplifyContourData simplifyUserData;
    std::memcpy(&simplifyUserData, caller.data, caller.size);
    std::string s = dumpSkPath(simplifyUserData.pathPtr, debugBase) + "\n";
    return s;
}

std::string binaryDumpFunc(CallerData caller, DebugLevel debugLevel, DebugBase debugBase) {
    OP_ASSERT(sizeof(SkiaOpContourData) == caller.size);
    SkiaOpContourData opUserData;
    std::memcpy(&opUserData, caller.data, caller.size);
    std::string s = dumpSkPath(opUserData.pathPtr, debugBase) + "\n";
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
}
#elif OP_DEBUG_DUMP && OP_TINY_SKIA
std::string unaryDumpFunc(CallerData caller, DebugLevel debugLevel, DebugBase debugBase) {
    return "";
}

std::string binaryDumpFunc(CallerData caller, DebugLevel debugLevel, DebugBase debugBase) {
    return "";
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
}

void SetSkiaContextCallBacks(Context* context) {
    SetContextCallBacks(context, emptySkPathFunc);
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
    WindingKeep operatorFunc;
    switch(op) {
        case kDifference_SkPathOp: operatorFunc = binaryWindingDifferenceFunc; break;
        case kIntersect_SkPathOp: operatorFunc = binaryWindingIntersectFunc; break;
        case kUnion_SkPathOp: operatorFunc = binaryWindingUnionFunc; break;
        case kXOR_SkPathOp: operatorFunc = binaryWindingExclusiveOrFunc; break;
        case kReverseDifference_SkPathOp: operatorFunc = binaryWindingReverseDifferenceFunc; break;
        default: OP_ASSERT(0);
    }
    WindingAdd addFunc;
    switch (windType) {
        case BinaryWindType::evenOdd: addFunc = binaryEvenOddFunc; break;
        case BinaryWindType::windLeft: addFunc = binaryWindingAddLeftFunc; break;
        case BinaryWindType::windRight: addFunc = binaryWindingAddRightFunc; break;
        case BinaryWindType::windBoth: addFunc = binaryWindingAddFunc; break;
        default: OP_ASSERT(0);
    }
    WindingAdd subtractFunc;
    switch (windType) {
        case BinaryWindType::evenOdd: subtractFunc = binaryEvenOddFunc; break;
        case BinaryWindType::windLeft: subtractFunc = binaryWindingSubtractLeftFunc; break;
        case BinaryWindType::windRight: subtractFunc = binaryWindingSubtractRightFunc; break;
        case BinaryWindType::windBoth: subtractFunc = binaryWindingSubtractFunc; break;
        default: OP_ASSERT(0);
    }
    SetWindingCallBacks(contour, addFunc, operatorFunc, 
            subtractFunc, binaryWindingVisibleFunc, binaryWindingZeroFunc 
            OP_DEBUG_DUMP_PARAMS(binaryWindingDumpInFunc, binaryWindingDumpOutFunc, binaryDumpFunc)
            OP_DEBUG_IMAGE_PARAMS(binaryWindingImageOutFunc, debugOpPathFunc,
	                debugOpGetDrawFunc, debugOpSetDrawFunc, debugOpSetIsOppFunc)
    );
    return contour;
}

void AddSkiaPath(AddWinding winding, const SkPath& path) {
    SkPath::RawIter iter(path);
    OpPoint closeLine[2];
    for (;;) {
        SkPoint pts[4];
        SkPath::Verb verb = iter.next(pts);
        switch (verb) {
        case SkPath::kMove_Verb:
            closeLine[1] = (OpPoint&) pts[0];
            pts[1] = pts[0];
            break;
        case SkPath::kLine_Verb:
            Add({ (OpPoint*) pts, sizeof(SkPoint) * 2, skiaLineType }, winding);
            break;
        case SkPath::kQuad_Verb:
            std::swap(pts[1], pts[2]);  // rearrange order from 0/1/2 to 0/2/1
            AddQuads({ (OpPoint*) pts, sizeof(SkPoint) * 3, skiaQuadType }, winding);
            break;
        case SkPath::kConic_Verb:
            std::swap(pts[1], pts[2]);  // rearrange order from 0/1/2 to 0/2/1
            pts[3].fX = iter.conicWeight(); // !!! hacky
            AddConics({ (OpPoint*) pts, sizeof(SkPoint) * 3 + sizeof(float), 
                    skiaConicType }, winding);
            break;
        case SkPath::kCubic_Verb:
            std::swap(pts[1], pts[2]);  // rearrange order from 0/1/2/3 to 0/3/1/2
            std::swap(pts[1], pts[3]);
            AddCubics({ (OpPoint*) pts, sizeof(SkPoint) * 4, skiaCubicType }, winding);
            break;
        case SkPath::kClose_Verb:
            Add({ closeLine, sizeof(closeLine), skiaLineType }, winding);
            break;
        case SkPath::kDone_Verb:
            return;
        default:
            OP_ASSERT(0);
        }
        closeLine[0] = (OpPoint&) pts[1];
    }
}

#if OP_DEBUG_IMAGE

void debugLineAddToSkPath(Curve c, SkPath& path) {
	path.lineTo(c.data->end.x, c.data->end.y);
}

void debugQuadAddToSkPath(Curve c, SkPath& path) {
    OpPoint controlPt = *(OpPoint*) c.data->optionalAdditionalData;
	path.quadTo(controlPt.x, controlPt.y, c.data->end.x, c.data->end.y);
}

void debugConicAddToSkPath(Curve c, SkPath& path) {
    PointWeight control = (PointWeight&) c.data->optionalAdditionalData;
	path.conicTo(control.pt.x, control.pt.y, c.data->end.x, c.data->end.y, control.weight);
}

void debugCubicAddToSkPath(Curve c, SkPath& path) {
    OpPoint* controls = (OpPoint*) c.data->optionalAdditionalData;
	path.cubicTo(controls[0].x, controls[0].y, controls[1].x, controls[1].y, 
            c.data->end.x, c.data->end.y);
}

#endif
