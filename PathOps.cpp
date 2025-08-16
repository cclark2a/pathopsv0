// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContext.h"
#include "PathOps.h"

namespace PathOpsV0Lib {

static OpContext* toImplementation(Context* interfaceContext) {
	return (OpContext*) interfaceContext;
}

static OpContour* toImplementation(Contour* interfaceContour) {
	return (OpContour*) interfaceContour;
}

static Context* toInterface(OpContext* implementationContext) {
	return (Context*) implementationContext;
}

static Contour* toInterface(OpContour* implementationContour) {
	return (Contour*) implementationContour;
}

Context* CreateContext(ContextUserData* userData) {
    OpContext* context = new OpContext(userData);
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContext = context;
#endif
#if OP_DEBUG_IMAGE
    OpDebugImage::init();
    oo();
#endif
    return toInterface(context);
}

ContextUserData* UserData(Context* context) {
    return (ContextUserData*) toImplementation(context)->userData;
}

void Add(Contour* interfaceContour, AddCurve curve) {
    OP_ASSERT(curve.points[0] != curve.points[1]);
    OpContour* contour = toImplementation(interfaceContour);
    contour->context->curveIndex(curve);
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContext = contour->context;
#endif
    contour->segments.emplace_back(interfaceContour, curve);
}

void Add(Contour* interfaceContour, Curve curve) {
    AddCurve addCurve { curve.context, &curve.data->start, curve.size, curve.type };
    Add(interfaceContour, addCurve);
}

Contour* CreateContour(Context* interfaceContext, Winding winding) {
    // reuse existing contour
    OpContext* context = toImplementation(interfaceContext);
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContext = context;
#endif
    OpContour* contour = context->makeContour();
	contour->winding = winding;
    return toInterface(contour);
}

Contour* Clone(Contour* interfaceContour) {
	OpContour* original = toImplementation(interfaceContour);
	if (original->isEmpty())
		return interfaceContour;
    OpContour* clone = original->context->makeContour();
	clone->winding = original->winding;
#if OP_DEBUG
	clone->debugCallbacks = original->debugCallbacks;
	clone->debugContourData = original->debugContourData;
#endif
    return toInterface(clone);
}

void DeleteContext(Context* interfaceContext) {
    OpContext* context = toImplementation(interfaceContext);
	OP_DEBUG_VALIDATE_CODE(context->debugJoiner = nullptr);
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContext = context;
#endif
    delete context;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContext = nullptr;
#endif
}

ContextError Error(Context* interfaceContext) {
    OpContext* context = toImplementation(interfaceContext);
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContext = context;
#endif
    return context->error;
}

void SetError(Context* interfaceContext, ContextError error) {
    OpContext* context = toImplementation(interfaceContext);
	context->error = error;
}

void SetErrorHandler(Context* interfaceContext, ErrorDispatch errorDispatch) {
    OpContext* context = toImplementation(interfaceContext);
	context->errorHandler.errorDispatchFuncPtr = errorDispatch;
}

void Normalize(Context* interfaceContext) {
    OpContext* context = toImplementation(interfaceContext);
    if (ContextError::none != context->error) {
        OP_DEBUG_CODE(context->debugData.success = false);
        return;
    }
	context->opsInit();
}

void ResetContour(Contour* interfaceContour) {
    OpContour* contour = toImplementation(interfaceContour);
    contour->segments.clear();
}

WindingCondition Resolve(Context* interfaceContext, PathOutput output) {
    OpContext* context = toImplementation(interfaceContext);
    if (ContextError::none != context->error) {
        OP_DEBUG_CODE(context->debugData.success = false);
        return -1;
    }
    context->callerOutput = output;
#if OP_DEBUG_IMAGE || OP_DEBUG_DUMP
    debugGlobalContext = context;
#endif
    return context->pathOps();
}

void SetContextCallbacks(Context* interfaceContext, ContextCallbacks contextCallbacks) {
    OpContext* context = toImplementation(interfaceContext);
    context->contextCallbacks = contextCallbacks;
}

void SetCurveCallbacks(Context* interfaceContext, int nativeCurveType,
        CurveCallbacks curveCallbacks) {
    OpContext* context = toImplementation(interfaceContext);
    context->callbacks.push_back(curveCallbacks);
    context->nativeCurveTypes.push_back(nativeCurveType);
}

void SetWindingCallbacks(Context* interfaceContext, WindingCallbacks windingCallbacks) {
    OpContext* context = toImplementation(interfaceContext);
	if (!windingCallbacks.windingSubtractFuncPtr)
		windingCallbacks.windingSubtractFuncPtr = windingCallbacks.windingAddFuncPtr;
    context->windingCallbacks = windingCallbacks;
}

} // namespace PathOpsV0Lib
