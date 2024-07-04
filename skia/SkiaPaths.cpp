// (c) 2023, Cary Clark cclark2@gmail.com
// new interface idea
#include "SkiaPaths.h"
#include "include/core/SkPath.h"

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

void commonOutput(Curve c, SkPath::Verb type, bool firstPt, bool lastPt, 
        PathOutput output) {
    SkPath& skpath = *(SkPath*)(output);
    if (firstPt)
        skpath.moveTo(c.data->start.x, c.data->start.y);
    if (SkPath::kLine_Verb == type)
        skpath.lineTo(c.data->end.x, c.data->end.y);
    else if (SkPath::kQuad_Verb == type) {
        OpPoint ctrlPt = quadControlPt(c);
        skpath.quadTo(ctrlPt.x, ctrlPt.y, c.data->end.x, c.data->end.y);
    } else if (SkPath::kConic_Verb == type) {
        PathOpsV0Lib::PointWeight ctrl(c);
        skpath.conicTo(ctrl.pt.x, ctrl.pt.y, c.data->end.x, c.data->end.y, ctrl.weight);
    } else if (SkPath::kCubic_Verb == type) {
        CubicControls ctrls(c);
        skpath.cubicTo(ctrls.pts[0].x, ctrls.pts[0].y, ctrls.pts[1].x, ctrls.pts[1].y,
                c.data->end.x, c.data->end.y);
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
void SetSkiaCurveCallBacks(PathOpsV0Lib::Context* context) {
    skiaLineType = SetCurveCallBacks(context, lineAxisRawHit, noNearly, noHull, lineIsFinite, 
            lineIsLine, noLinear, noBounds, lineNormal, skiaLineOutput, noReverse,
            lineTangent, linesEqual, linePtAtT, /* double not required */ linePtAtT, 
            linePtCount, noRotate, lineSubDivide, lineXYAtT
            OP_DEBUG_DUMP_PARAMS(lineDebugDumpSize, noDumpCurveExtra, noDumpCurveSet, 
                    noDumpCurveSetExtra)
            OP_DEBUG_IMAGE_PARAMS(debugLineAddToSkPath)
    );
    skiaQuadType = SetCurveCallBacks(context, quadAxisRawHit, quadNearly, quadHull, quadIsFinite, 
            quadIsLine, quadIsLinear, quadSetBounds, quadNormal, skiaQuadOutput, noReverse,
            quadTangent, quadsEqual, quadPtAtT, /* double not required */ quadPtAtT, 
            quadPtCount, quadRotate, quadSubDivide, quadXYAtT
            OP_DEBUG_DUMP_PARAMS(quadDebugDumpSize, noDumpCurveExtra, quadDumpSet, 
                    noDumpCurveSetExtra)
            OP_DEBUG_IMAGE_PARAMS(debugQuadAddToSkPath)
    );
    skiaConicType = SetCurveCallBacks(context, conicAxisRawHit, conicNearly, conicHull, 
            conicIsFinite, conicIsLine, 
            conicIsLinear, conicSetBounds, conicNormal, skiaConicOutput, noReverse,
            conicTangent, conicsEqual, conicPtAtT, /* double not required */ conicPtAtT, 
            conicPtCount, conicRotate, conicSubDivide, conicXYAtT
            OP_DEBUG_DUMP_PARAMS(conicDebugDumpSize, conicDebugDumpExtra, conicDumpSet, 
                    conicDumpSetExtra)
            OP_DEBUG_IMAGE_PARAMS(debugConicAddToSkPath)
    );
    skiaCubicType = SetCurveCallBacks(context, cubicAxisRawHit, cubicNearly, cubicHull, 
            cubicIsFinite, cubicIsLine, 
            cubicIsLinear, cubicSetBounds, cubicNormal, skiaConicOutput, noReverse,
            cubicTangent, cubicsEqual, cubicPtAtT, /* double not required */ cubicPtAtT, 
            cubicPtCount, cubicRotate, cubicSubDivide, cubicXYAtT
            OP_DEBUG_DUMP_PARAMS(cubicDebugDumpSize, noDumpCurveExtra, cubicDumpSet, 
                    noDumpCurveSetExtra)
            OP_DEBUG_IMAGE_PARAMS(debugCubicAddToSkPath)
    );
}

#if OP_DEBUG_DUMP
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
    OP_ASSERT(0 <= opUserData.op && opUserData.op <= kReverseDifference_SkPathOp);
    s += "SkPathOp:" + skPathOpNames[opUserData.op] + " ";
    OP_ASSERT(BinaryOperand::left == opUserData.operand ||
            BinaryOperand::right == opUserData.operand);
    s += "BinaryOperand:" + std::string(BinaryOperand::left == opUserData.operand ? "left" : "right");
    return s;
}
#endif

#if OP_DEBUG_IMAGE
void* debugNativePath(CallerData caller) {
    return (void*) (caller.data + offsetof(ContourData, nativePath));
}

bool* debugDrawFunc(CallerData caller) {
    return (bool*) (caller.data + offsetof(ContourData, drawNativePath));
}

inline uint32_t nativeOutColorFunc() {
    return OpDebugAlphaColor(20, green);
}
#endif

void SetSkiaContextCallBacks(PathOpsV0Lib::Context* context) {
    SetContextCallBacks(context
        OP_DEBUG_IMAGE_PARAMS(nativeOutColorFunc)
    );
}

PathOpsV0Lib::Contour* SetSkiaSimplifyCallBacks(PathOpsV0Lib::Context* context, 
        bool isWindingFill  OP_DEBUG_PARAMS(const SkPath& path)) {
    SkiaSimplifyContourData simplifyUserData { (Operation) 0, (Operand) 0
            OP_DEBUG_PARAMS(&path)  OP_DEBUG_IMAGE_PARAMS(true) };
    Contour* contour = CreateContour({context, (ContourData*) &simplifyUserData, 
            sizeof(SkiaSimplifyContourData) } );
    WindingAdd addFunc = isWindingFill ? unaryWindingAddFunc : unaryEvenOddFunc;
    WindingAdd subtractFunc = isWindingFill ? unaryWindingSubtractFunc : unaryEvenOddFunc;
    SetWindingCallBacks(contour, addFunc, unaryWindingKeepFunc, 
            subtractFunc, unaryWindingVisibleFunc, unaryWindingZeroFunc 
            OP_DEBUG_DUMP_PARAMS(unaryWindingDumpInFunc, unaryWindingDumpOutFunc, unaryDumpFunc)
            OP_DEBUG_IMAGE_PARAMS(unaryWindingImageOutFunc, unaryCCOverlapsColorFunc,
	                unaryCurveCurveColorFunc, unaryNativeFillColorFunc,
	                unaryNativeInColorFunc, unaryNativePathFunc,
	                unaryContourDrawFunc, noIsOppFunc)
    );
    return contour;
}

PathOpsV0Lib::Contour* SetSkiaOpCallBacks(PathOpsV0Lib::Context* context, SkPathOp op,
        BinaryOperand operand, bool isWindingFill  OP_DEBUG_PARAMS(const SkPath& path)) {
    SkiaOpContourData windingUserData { op, operand  OP_DEBUG_PARAMS(&path )  
            OP_DEBUG_IMAGE_PARAMS(true) };
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
    WindingAdd addFunc = isWindingFill ? binaryWindingAddFunc : binaryEvenOddFunc;
    WindingAdd subtractFunc = isWindingFill ? binaryWindingSubtractFunc : binaryEvenOddFunc;
    SetWindingCallBacks(contour, addFunc, operatorFunc, 
            subtractFunc, binaryWindingVisibleFunc, binaryWindingZeroFunc 
            OP_DEBUG_DUMP_PARAMS(binaryWindingDumpInFunc, binaryWindingDumpOutFunc, binaryDumpFunc)
            OP_DEBUG_IMAGE_PARAMS(binaryWindingImageOutFunc, binaryCCOverlapsColorFunc,
	                binaryCurveCurveColorFunc, binaryNativeFillColorFunc,
	                binaryNativeInColorFunc, binaryNativePathFunc,
	                binaryContourDrawFunc, binaryIsOppFunc)
    );
    return contour;
}

void AddSkiaPath(PathOpsV0Lib::AddWinding winding, const SkPath& path) {
    SkPath::RawIter iter(path);
    SkPath::Verb verb;
    do {
        SkPoint pts[4];
        verb = iter.next(pts);
        switch (verb) {
        case SkPath::kMove_Verb:
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
            break;
        case SkPath::kDone_Verb:
            break;
        }
    } while (verb != SkPath::kDone_Verb);
}

#if OP_DEBUG_IMAGE

void debugLineAddToSkPath(PathOpsV0Lib::Curve c, SkPath& path) {
	path.lineTo(c.data->end.x, c.data->end.y);
}

void debugQuadAddToSkPath(PathOpsV0Lib::Curve c, SkPath& path) {
    OpPoint controlPt = *(OpPoint*) c.data->optionalAdditionalData;
	path.quadTo(controlPt.x, controlPt.y, c.data->end.x, c.data->end.y);
}

void debugConicAddToSkPath(PathOpsV0Lib::Curve c, SkPath& path) {
    PointWeight control = (PointWeight&) c.data->optionalAdditionalData;
	path.conicTo(control.pt.x, control.pt.y, c.data->end.x, c.data->end.y, control.weight);
}

void debugCubicAddToSkPath(PathOpsV0Lib::Curve c, SkPath& path) {
    OpPoint* controls = (OpPoint*) c.data->optionalAdditionalData;
	path.cubicTo(controls[0].x, controls[0].y, controls[1].x, controls[1].y, c.data->end.x, c.data->end.y);
}

#endif
