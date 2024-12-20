// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "PathOps.h"

namespace PathOpsV0Lib {

Context* CreateContext() {
    OpContours* contours = new OpContours();
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

Contour* CreateContour(Context* context) {
    // reuse existing contour
    OpContours* contours = (OpContours*) context;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = contours;
#endif
    OpContour* contour = contours->makeContour();
//    contour->addCallerData(callerData);
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

void SetError(Context* context, ContextError error) {
    OpContours* contours = (OpContours*) context;
	contours->error = error;
}

void SetErrorHandler(Context* context, ErrorDispatch errorDispatch) {
    OpContours* contours = (OpContours*) context;
	contours->errorHandler.errorDispatchFuncPtr = errorDispatch;
}

void Normalize(Context* context) {
    OpContours* contours = (OpContours*) context;
    if (ContextError::none != contours->error) {
        OP_DEBUG_CODE(contours->debugData.success = false);
        return;
    }
	contours->opsInit();
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
        SetLineType setLineType, MaxSignSwap signSwap, 
		MaxCurveCurve depth, MaxCurveCurve splits, MaxLimbs limbs) {
    OpContours* contours = (OpContours*) context;
    contours->contextCallBacks = {
        emptyNativePath,
        makeLine,
        setLineType,
		signSwap,
		depth,
		splits,
		limbs
    };
}

CurveType SetCurveCallBacks(Context* context, CurveOutput outputFunc, AxisT axisFunc,
        CurveHull hullFunc, CurveIsFinite isFiniteFunc, CurveIsLine isLineFunc, 
        SetBounds setBoundsFunc, CurvePinCtrl curvePinFunc, 
		CurveTangent tangentFunc, CurvesEqual equalFunc, PtAtT ptAtTFunc, HullPtCount ptCountFunc, 
		Rotate rotateFunc, SubDivide subDivideFunc, XYAtT xyAtTFunc, CurveReverse reverseFunc, 
		CurveConst cutFunc, CurveConst normalLimitFunc, CurveConst interceptLimitFunc
) {
    OpContours* contours = (OpContours*) context;
    contours->callBacks.push_back( { outputFunc, axisFunc, hullFunc, isFiniteFunc, isLineFunc, 
            setBoundsFunc, curvePinFunc, tangentFunc, equalFunc, ptAtTFunc, ptCountFunc, rotateFunc, 
            subDivideFunc, xyAtTFunc, reverseFunc, cutFunc, normalLimitFunc, interceptLimitFunc
            } );
    return (CurveType) contours->callBacks.size();
}

void SetWindingCallBacks(Contour* ctour, WindingAdd addFunc, WindingKeep keepFunc, 
		WindingVisible visibleFunc, WindingZero zeroFunc, WindingSubtract subtractFunc
) {
    OpContour* contour = (OpContour*) ctour;
	if (!subtractFunc)
		subtractFunc = addFunc;
    contour->callBacks = { addFunc, keepFunc, visibleFunc, zeroFunc, subtractFunc };
}

} // namespace PathOpsV0Lib
