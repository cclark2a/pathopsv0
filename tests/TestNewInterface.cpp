// (c) 2023, Cary Clark cclark2@gmail.com
#include "PathOpsTypes.h"

namespace PathOpsV0Lib {    // !!! move to new PathOps.h
void Add(AddCurve , AddWinding );
}

#include "curves/Line.h"
#include "curves/NoCurve.h"
#include "curves/QuadBezier.h"
#include "curves/UnaryWinding.h"

#include "PathOps.h"

// curve types
OpType lineType = OpType::no;  // unset
OpType quadType = OpType::no;

void commonOutput(PathOpsV0Lib::Curve c, OpType type, bool firstPt, bool lastPt, 
        PathOpsV0Lib::PathOutput output) {
    if (firstPt)
        OpDebugOut("contour start --\n");
    std::string outStr = lineType == type ? "line: " : "quad: ";
    auto addPtStr = [&outStr](const OpPoint& pt, std::string delimiter) {
        outStr += "{ " + std::to_string(pt.x) + ", " + std::to_string(pt.y) + " }" + delimiter;
    };
    addPtStr(c.data->start, ", ");
    if (quadType == type)
        addPtStr(quadControlPt(c), ", ");
    addPtStr(c.data->end, "\n");
    OpDebugOut(outStr);
    if (lastPt)
        OpDebugOut("-- contour end\n");
}

void lineOutput(PathOpsV0Lib::Curve c, bool firstPt, bool lastPt, 
        PathOpsV0Lib::PathOutput output) {
    commonOutput(c, lineType, firstPt, lastPt, output);
}

void quadOutput(PathOpsV0Lib::Curve c, bool firstPt, bool lastPt, 
        PathOpsV0Lib::PathOutput output) {
    commonOutput(c, quadType, firstPt, lastPt, output);
}

void noEmptyPath(PathOpsV0Lib::PathOutput ) {
}

void testNewInterface() {
    using namespace PathOpsV0Lib;

    Context* context = CreateContext({nullptr, 0});
    SetContextCallBacks(context, noEmptyPath);

#if OP_DEBUG
    OpDebugData debugData(false);
    debugData.debugCurveCurve1 = 2;
    debugData.debugCurveCurve2 = 7;
    debugData.debugCurveCurveDepth = 6;
    Debug(context, debugData);
#endif

    lineType = SetCurveCallBacks(context, lineAxisRawHit, noNearly, noHull, lineIsFinite, 
            lineIsLine, noLinear, noBounds, lineNormal, lineOutput, noPinCtrl, noReverse,
            lineTangent, linesEqual, linePtAtT, /* double not required */ linePtAtT, 
            linePtCount, noRotate, lineSubDivide, lineXYAtT
            OP_DEBUG_DUMP_PARAMS(noDumpCurveExtra)
            OP_DEBUG_IMAGE_PARAMS(noAddToSkPathFunc)
    );
    quadType = SetCurveCallBacks(context, quadAxisRawHit, quadNearly, quadHull, quadIsFinite, 
            quadIsLine, quadIsLinear, quadSetBounds, quadNormal, quadOutput, quadPinCtrl, noReverse,
            quadTangent, quadsEqual, quadPtAtT, /* double not required */ quadPtAtT, 
            quadPtCount, quadRotate, quadSubDivide, quadXYAtT
            OP_DEBUG_DUMP_PARAMS(noDumpCurveExtra)
            OP_DEBUG_IMAGE_PARAMS(noAddToSkPathFunc)
    );

    // example: given points describing a pair of closed loops with quadratic Beziers, find
    //          their intersection
    Contour* contour = CreateContour({context, nullptr, 0});
    SetWindingCallBacks(contour, unaryWindingAddFunc, unaryWindingKeepFunc, 
            unaryWindingSubtractFunc, unaryWindingVisibleFunc, unaryWindingZeroFunc 
            OP_DEBUG_DUMP_PARAMS(unaryWindingDumpInFunc, unaryWindingDumpOutFunc, noDumpFunc)
            OP_DEBUG_IMAGE_PARAMS(noWindingImageOutFunc, noNativePathFunc,
                    noDebugGetDrawFunc, noDebugSetDrawFunc, noIsOppFunc)
    );
    int windingData[] = { 1 };
    AddWinding addWinding { contour, windingData, sizeof(windingData) };
    constexpr size_t lineSize = sizeof(OpPoint) * 2;
    constexpr size_t quadSize = sizeof(OpPoint) * 3;

    // note that the data below omits start points for curves that match the previous end point
                      // start     end      control
    OpPoint contour1[] { { 2, 0 }, { 1, 2 }, { 0, 2 },  // quad: start, end, control
                                   { 1, 2 }, { 2, 3 },  // line: start, end
                                             { 2, 0 },  // line: end
    };
    // break the quads so that their control points lie inside the bounds
    // formed by the end points (i.e., find the quads' extrema)
    AddQuads({ &contour1[0], quadSize, quadType }, addWinding );
    Add(     { &contour1[3], lineSize, lineType }, addWinding );
    Add(     { &contour1[4], lineSize, lineType }, addWinding );

    OpPoint contour2[] { { 0, 0 }, { 1, 1 },            // line: start, end
                                   { 1, 3 }, { 0, 3 },  // quad: end, control
                                   { 1, 3 }, { 0, 0 },  // line: start, end
    };
    Add(     { &contour2[0], lineSize, lineType }, addWinding );
    AddQuads({ &contour2[1], quadSize, quadType }, addWinding );
    Add(     { &contour2[4], lineSize, lineType }, addWinding );

    Resolve(context, nullptr);
    DeleteContext(context);

    if (Error(context)) {
        exit(1);
    }
}
