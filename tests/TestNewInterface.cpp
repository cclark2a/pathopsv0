// (c) 2023, Cary Clark cclark2@gmail.com

#include "curves/Line.h"
#include "curves/NoCurve.h"
#include "curves/QuadBezier.h"
#include "curves/UnaryWinding.h"

// curve types
PathOpsV0Lib::CurveType lineType = (PathOpsV0Lib::CurveType) 0;  // unset
PathOpsV0Lib::CurveType quadType = (PathOpsV0Lib::CurveType) 0;
constexpr size_t lineSize = sizeof(OpPoint) * 2;
constexpr size_t quadSize = sizeof(OpPoint) * 3;

void commonOutput(PathOpsV0Lib::Curve c, PathOpsV0Lib::CurveType type, bool firstPt, bool lastPt, 
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

PathOpsV0Lib::CurveType testNewSetLineType(PathOpsV0Lib::Curve ) {
    return lineType;
}

void testNewInterface() {
    using namespace PathOpsV0Lib;

    Context* context = CreateContext();
    SetContextCallBacks(context, { testNewSetLineType });

#if OP_DEBUG
    OpDebugData debugData(false);
    debugData.curveCurve1 = 2;
    debugData.curveCurve2 = 7;
    debugData.curveCurveDepth = 6;
    Debug(context, debugData);
#endif

    lineType = SetCurveCallBacks(context, { lineOutput });
    quadType = SetCurveCallBacks(context, { quadOutput, quadAxisT,
			quadHull, quadIsFinite, quadIsLine, 
			quadSetBounds, quadPinCtrl, 
			quadTangent, quadsEqual, quadPtAtT,
            quadHullPtCount, quadRotate, quadSubDivide, quadXYAtT });

    // example: given points describing a pair of closed loops with quadratic Beziers, find
    //          their intersection
    UnaryWinding windingData(1);
    Winding winding { &windingData, sizeof(windingData) };
    Contour* contour = CreateContour(context, winding);
    SetWindingCallBacks(contour, { unaryWindingAddFunc, unaryWindingKeepFunc, unaryWindingVisibleFunc,
			unaryWindingZeroFunc, unaryWindingSubtractFunc });
#if OP_DEBUG
	SetDebugWindingCallBacks(contour, { nullptr, 0 }, noDebugBitOper
            OP_DEBUG_DUMP_PARAMS(unaryWindingDumpInFunc, unaryWindingDumpOutFunc, noDumpFunc)
            OP_DEBUG_IMAGE_PARAMS(noWindingImageOutFunc, noNativePathFunc,
            noDebugGetDrawFunc, noDebugSetDrawFunc, noIsOppFunc)
    );
#endif

    // note that the data below omits start points for curves that match the previous end point
                      //  start      end      control
    OpPoint contour1[] { { 2, 0 }, { 1, 2 }, { 0, 2 },  // quad: start, end, control
                         { 1, 2 }, { 2, 3 },            // line: start, end
                                   { 2, 0 },            // line:        end
    };
    // break the quads so that their control points lie inside the bounds
    // formed by the end points (i.e., find the quads' extrema)
    AddQuads(contour, { &contour1[0], quadSize, quadType } );
    Add(     contour, { &contour1[3], lineSize, lineType } );
    Add(     contour, { &contour1[4], lineSize, lineType } );

    OpPoint contour2[] { { 0, 0 }, { 1, 1 },            // line: start, end
                                   { 1, 3 }, { 0, 3 },  // quad:        end, control
                         { 1, 3 }, { 0, 0 },            // line: start, end
    };
    Add(     contour, { &contour2[0], lineSize, lineType } );
    AddQuads(contour, { &contour2[1], quadSize, quadType } );
    Add(     contour, { &contour2[4], lineSize, lineType } );

	Normalize(context);
    Resolve(context, nullptr);
	ContextError error = Error(context);
    DeleteContext(context);
    if (ContextError::none != error)
        exit(1);
}

#if OP_DEBUG && OP_TINY_TEST
bool OpDebugSkipBreak() {
	return true;
}
#endif
