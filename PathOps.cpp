// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "PathOps.h"

namespace PathOpsV0Lib {

Context* CreateContext() {
    OpContext* contours = new OpContext();
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContext = contours;
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
    debugGlobalContext = contour->context;
#endif
    contour->segments.emplace_back(libContour, curve);
}

Contour* CreateContour(Context* context, Winding winding) {
    // reuse existing contour
    OpContext* contours = (OpContext*) context;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContext = contours;
#endif
    OpContour* contour = contours->makeContour();
	contour->winding = winding;
    return (Contour*) contour;
}

Contour* Clone(Contour* contour) {
	OpContour* original = (OpContour*) contour;
	if (original->isEmpty())
		return (Contour*) original;
    OpContour* clone = original->context->makeContour();
	clone->winding = original->winding;
#if OP_DEBUG
	clone->debugCallbacks = original->debugCallbacks;
	clone->debugCaller = original->debugCaller;
#endif
    return (Contour*) clone;
}

void DeleteContext(Context* context) {
    OpContext* contours = (OpContext*) context;
	OP_DEBUG_VALIDATE_CODE(contours->debugJoiner = nullptr);
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContext = contours;
#endif
    delete contours;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContext = nullptr;
#endif
}

ContextError Error(Context* context) {
    OpContext* contours = (OpContext*) context;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContext = contours;
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
    debugGlobalContext = contours;
#endif
    // !!! change this to record error instead of success
    /* bool success = */ contours->pathOps();
}

void SetContextCallbacks(Context* context, ContextCallbacks contextCallbacks) {
    OpContext* contours = (OpContext*) context;
    contours->contextCallbacks = contextCallbacks;
}

CurveType SetCurveCallbacks(Context* context, CurveCallbacks curveCallbacks) {
    OpContext* contours = (OpContext*) context;
    contours->callbacks.push_back(curveCallbacks);
    return (CurveType) contours->callbacks.size();
}

void SetWindingCallbacks(Context* ctext, WindingCallbacks windingCallbacks) {
    OpContext* context = (OpContext*) ctext;
	if (!windingCallbacks.windingSubtractFuncPtr)
		windingCallbacks.windingSubtractFuncPtr = windingCallbacks.windingAddFuncPtr;
    context->windingCallbacks = windingCallbacks;
}

} // namespace PathOpsV0Lib
