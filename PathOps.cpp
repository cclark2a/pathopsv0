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

void Add(Contour* libContour, AddCurve curve) {
    OP_ASSERT(curve.points[0] != curve.points[1]);
    OpContour* contour = (OpContour*) libContour;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = contour->contours;
#endif
    contour->segments.emplace_back(libContour, curve);
}

Contour* CreateContour(Context* context, Winding winding) {
    // reuse existing contour
    OpContours* contours = (OpContours*) context;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = contours;
#endif
    OpContour* contour = contours->makeContour();
	contour->winding = winding;
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

void SetContextCallBacks(Context* context, ContextCallBacks contextCallBacks) {
    OpContours* contours = (OpContours*) context;
    contours->contextCallBacks = contextCallBacks;
}

CurveType SetCurveCallBacks(Context* context, CurveCallBacks curveCallBacks) {
    OpContours* contours = (OpContours*) context;
    contours->callBacks.push_back(curveCallBacks);
    return (CurveType) contours->callBacks.size();
}

void SetWindingCallBacks(Contour* ctour, WindingCallBacks windingCallBacks) {
    OpContour* contour = (OpContour*) ctour;
	if (!windingCallBacks.windingSubtractFuncPtr)
		windingCallBacks.windingSubtractFuncPtr = windingCallBacks.windingAddFuncPtr;
    contour->callBacks = windingCallBacks;
}

} // namespace PathOpsV0Lib
