// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "PathOps.h"

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
    const OpSegment& seg = contour->segments.back();
    if (!seg.isFinite())
        contour->contours->setError(ContextError::segmentBounds  OP_DEBUG_PARAMS(seg.id));
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

ContextError Error(Context* context) {
    OpContours* contours = (OpContours*) context;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = contours;
#endif
    return contours->error;
}

void ResetContour(Contour* c) {
    OpContour* contour = (OpContour*) c;
    contour->segments.clear();
}

void Resolve(Context* context, PathOutput output) {
    OpContours* contours = (OpContours*) context;
    if (ContextError::none != contours->error) {
        OP_DEBUG_CODE(contours->debugData.success = false);
        return;
    }
    contours->callerOutput = output;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = contours;
#endif
    // !!! change this to record error instead of success
    /* bool success = */ contours->pathOps();
}

void SetContextCallBacks(Context* context, EmptyNativePath emptyNativePath, MakeLine makeLine,
        SetLineType setLineType) {
    OpContours* contours = (OpContours*) context;
    contours->contextCallBacks = {
        emptyNativePath,
        makeLine,
        setLineType
    };
}

CurveType SetCurveCallBacks(Context* context, AxisRawHit axisFunc, /* ControlNearlyEnd nearlyFunc, */
        CurveHull hullFunc, CurveIsFinite isFiniteFunc, CurveIsLine isLineFunc, 
        /* CurveIsLinear isLinearFunc, */ SetBounds setBoundsFunc,
        CurveNormal normalFunc, 
        CurveOutput outputFunc, CurvePinCtrl curvePinFunc, CurveReverse reverseFunc, 
        CurveTangent tangentFunc, CurvesEqual equalFunc, PtAtT ptAtTFunc, 
        DoublePtAtT doublePtAtTFunc, PtCount ptCountFunc, Rotate rotateFunc, 
        SubDivide subDivideFunc, XYAtT xyAtTFunc
        OP_DEBUG_DUMP_PARAMS(DebugDumpCurveName debugDumpNameFunc, 
        DebugDumpCurveExtra debugDumpExtraFunc)
		OP_DEBUG_IMAGE_PARAMS(DebugAddToPath debugAddToPathFunc)
) {
    OpContours* contours = (OpContours*) context;
    contours->callBacks.push_back( { axisFunc, hullFunc, isFiniteFunc, isLineFunc, 
            setBoundsFunc, normalFunc, outputFunc, curvePinFunc, reverseFunc, 
            tangentFunc, equalFunc, ptAtTFunc, doublePtAtTFunc, ptCountFunc, rotateFunc, 
            subDivideFunc, xyAtTFunc 
		    OP_DEBUG_DUMP_PARAMS(debugDumpNameFunc, debugDumpExtraFunc)
		    OP_DEBUG_IMAGE_PARAMS(debugAddToPathFunc)
            } );
    return (CurveType) contours->callBacks.size();
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
    contours->debugData = debugData;
}
#endif

} // namespace PathOpsV0Lib
