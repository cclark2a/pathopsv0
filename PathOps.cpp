// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "PathOps.h"

namespace PathOpsV0Lib {

Context* CreateContext() {
    OpContext* contours = new OpContext();
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
    debugGlobalContours = contour->context;
#endif
    contour->segments.emplace_back(libContour, curve);
}

Contour* CreateContour(Context* context, Winding winding) {
    // reuse existing contour
    OpContext* contours = (OpContext*) context;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = contours;
#endif
    OpContour* contour = contours->makeContour();
	contour->winding = winding;
    return (Contour*) contour;
}

#if WINDER_CONTOUR_EXPERIMENT
Contour* Clone(Contour* contour) {
	OpContour* original = (OpContour*) contour;
	if (original->isEmpty())
		return (Contour*) original;
    OpContour* clone = original->context->makeContour();
	clone->winding = original->winding;
#if OP_DEBUG
	clone->debugCallBacks = original->debugCallBacks;
	clone->debugCaller = original->debugCaller;
#endif
    return (Contour*) clone;
}
#endif

void DeleteContext(Context* context) {
    OpContext* contours = (OpContext*) context;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = contours;
#endif
    delete contours;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = nullptr;
#endif
}

ContextError Error(Context* context) {
    OpContext* contours = (OpContext*) context;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContours = contours;
#endif
    return contours->error;
}

void SetError(Context* context, ContextError error) {
    OpContext* contours = (OpContext*) context;
	contours->error = error;
}

void SetErrorHandler(Context* context, ErrorDispatch errorDispatch) {
    OpContext* contours = (OpContext*) context;
	contours->errorHandler.errorDispatchFuncPtr = errorDispatch;
}

void Normalize(Context* context) {
    OpContext* contours = (OpContext*) context;
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
    OpContext* contours = (OpContext*) context;
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
    OpContext* contours = (OpContext*) context;
    contours->contextCallBacks = contextCallBacks;
}

CurveType SetCurveCallBacks(Context* context, CurveCallBacks curveCallBacks) {
    OpContext* contours = (OpContext*) context;
    contours->callBacks.push_back(curveCallBacks);
    return (CurveType) contours->callBacks.size();
}

void SetWindingCallBacks(Context* ctext, WindingCallBacks windingCallBacks) {
    OpContext* context = (OpContext*) ctext;
	if (!windingCallBacks.windingSubtractFuncPtr)
		windingCallBacks.windingSubtractFuncPtr = windingCallBacks.windingAddFuncPtr;
    context->windingCallBacks = windingCallBacks;
}

} // namespace PathOpsV0Lib
