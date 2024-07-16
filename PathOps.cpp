// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "PathOps.h"

bool PathOps(OpInPath& left, OpInPath& right, OpOperator opOperator, OpOutPath& result
        OP_DEBUG_PARAMS(OpDebugData& debugData)) {
#if OP_TEST_NEW_INTERFACE
    OP_ASSERT(0);
    return false;
#else
    OpContours contourList(left, right, opOperator);
#if OP_DEBUG
    contourList.debugExpect = debugData.debugExpect;
#if !OP_TEST_NEW_INTERFACE
    contourList.debugResult = &result;
#endif
    contourList.debugInPathOps = true;
    contourList.debugInClearEdges = false;
    contourList.debugTestname = debugData.debugTestname;
    OP_DEBUG_DUMP_CODE(contourList.dumpCurve1 = debugData.debugCurveCurve1);
    OP_DEBUG_DUMP_CODE(contourList.dumpCurve2 = debugData.debugCurveCurve2);
    OP_DEBUG_DUMP_CODE(contourList.debugBreakDepth = debugData.debugCurveCurveDepth);
#endif
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = &contourList;
#endif
#if OP_DEBUG_IMAGE
    OpDebugImage::init();
//    oo();
#endif
    bool success = contourList.pathOps(result);
#if OP_DEBUG
    debugData.debugWarnings = contourList.debugWarnings;
#endif
    return success;
#endif
}

// new interface

namespace PathOpsV0Lib {

Context* CreateContext(AddContext callerData) {
    OpContours* contours = new OpContours();
    contours->addCallerData(callerData);
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = contours;
#endif
#if OP_DEBUG_IMAGE
    OpDebugImage::init();
    oo();
#endif
    return (Context*) contours;
}

void Add(AddCurve curve, AddWinding windings) {
    OP_ASSERT(curve.points[0] != curve.points[1]);
    OpContour* contour = (OpContour*) windings.contour;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = contour->contours;
#endif
    contour->segments.emplace_back(curve, windings);
}

Contour* CreateContour(AddContour callerData) {
    // reuse existing contour
    OpContours* contours = (OpContours*) callerData.context;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = contours;
#endif
    OpContour* contour = contours->makeContour();
    contour->addCallerData(callerData);
    return (Contour*) contour;
}

void DeleteContext(Context* context) {
    OpContours* contours = (OpContours*) context;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = contours;
#endif
    delete contours;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = nullptr;
#endif
}

int Error(Context* context) {
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    OpContours* contours = (OpContours*) context;
    debugGlobalContours = contours;
#endif
    // !!! incomplete
    return 0;
}

void Resolve(Context* context, PathOutput output) {
    OpContours* contours = (OpContours*) context;
    contours->callerOutput = output;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = contours;
#endif
    // !!! change this to record error instead of success
    /* bool success = */ contours->pathOps();
}

void SetContextCallBacks(Context* context, EmptyNativePath emptyNativePath) {
    OpContours* contours = (OpContours*) context;
    contours->contextCallBacks = {
        emptyNativePath
    };
}

OpType SetCurveCallBacks(Context* context, AxisRawHit axisFunc, ControlNearlyEnd nearlyFunc,
        CurveHull hullFunc, CurveIsFinite isFiniteFunc, CurveIsLine isLineFunc, 
        CurveIsLinear isLinearFunc, SetBounds setBoundsFunc, CurveNormal normalFunc, 
        CurveOutput outputFunc, CurvePinCtrl curvePinFunc, CurveReverse reverseFunc, 
        CurveTangent tangentFunc, CurvesEqual equalFunc, PtAtT ptAtTFunc, 
        DoublePtAtT doublePtAtTFunc, PtCount ptCountFunc, Rotate rotateFunc, 
        SubDivide subDivideFunc, XYAtT xyAtTFunc
        OP_DEBUG_DUMP_PARAMS(DebugDumpCurveExtra debugDumpExtraFunc)
		OP_DEBUG_IMAGE_PARAMS(DebugAddToPath debugAddToPathFunc)
) {
    OpContours* contours = (OpContours*) context;
    contours->callBacks.push_back( { axisFunc, nearlyFunc, hullFunc, isFiniteFunc, isLineFunc, 
            isLinearFunc, setBoundsFunc, normalFunc, outputFunc, curvePinFunc, reverseFunc, 
            tangentFunc, equalFunc, ptAtTFunc, doublePtAtTFunc, ptCountFunc, rotateFunc, 
            subDivideFunc, xyAtTFunc 
		    OP_DEBUG_DUMP_PARAMS(debugDumpExtraFunc)
		    OP_DEBUG_IMAGE_PARAMS(debugAddToPathFunc)
            } );
    return (OpType) contours->callBacks.size();
}

void SetWindingCallBacks(Contour* ctour, WindingAdd addFunc, WindingKeep keepFunc,
        WindingSubtract subtractFunc, WindingVisible visibleFunc, WindingZero zeroFunc
		OP_DEBUG_DUMP_PARAMS(DebugDumpContourIn dumpInFunc, DebugDumpContourOut dumpOutFunc, 
                DebugDumpContourExtra dumpFunc)
        OP_DEBUG_IMAGE_PARAMS(DebugImageOut dumpImageOutFunc, 
                DebugNativePath debugNativePathFunc, 
                DebugGetDraw debugGetDrawFunc, DebugSetDraw debugSetDrawFunc,
                DebugIsOpp debugIsOppFunc)
) {
    OpContour* contour = (OpContour*) ctour;
    contour->callBacks = { addFunc, keepFunc, subtractFunc, visibleFunc, zeroFunc 
            OP_DEBUG_DUMP_PARAMS(dumpInFunc, dumpOutFunc, dumpFunc)
            OP_DEBUG_IMAGE_PARAMS(dumpImageOutFunc,
                    debugNativePathFunc, debugGetDrawFunc, debugSetDrawFunc, debugIsOppFunc)
            };
}

#if OP_DEBUG
void Debug(Context* context, OpDebugData& debugData) {
    OpContours* contours = (OpContours*) context;
    contours->debugTestname = debugData.debugTestname;
    OP_DEBUG_DUMP_CODE(contours->dumpCurve1 = debugData.debugCurveCurve1);
    OP_DEBUG_DUMP_CODE(contours->dumpCurve2 = debugData.debugCurveCurve2);
    OP_DEBUG_DUMP_CODE(contours->debugBreakDepth = debugData.debugCurveCurveDepth);
}
#endif

} // namespace PathOpsV0Lib
