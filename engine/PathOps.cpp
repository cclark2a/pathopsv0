// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpContext.h"
#include "PathOps.h"

namespace PathOpsV0Lib {

static OpContext* toImplementation(Context* interfaceContext) {
    OpContext* context = (OpContext*) interfaceContext;
	return context;
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

Context* CreateContext() {
    OpContext* context = new OpContext();
#if OP_DEBUGGER || OP_DEBUG_DUMP
    debugGlobalContext = context;
#endif
    return toInterface(context);
}

void AddUserData(Context* context, const ContextUserData& userData) {
    toImplementation(context)->addUserData(userData);
}

ContextUserData UserData(Context* context, UserDataType type) {
    return toImplementation(context)->findUserData(type);
}

void Add(Contour* interfaceContour, const AddCurve& curve) {
    OpContour* contour = toImplementation(interfaceContour);
    contour->context->initialized = false;
    AddCurve temp(curve);
    temp.type = contour->context->curveIndex(temp.type);
    contour->segments.emplace_back(interfaceContour, temp);
}

void Add(Contour* interfaceContour, const Curve& curve) {
    AddCurve addCurve { curve.context, &curve.data->start, curve.size, curve.type };
    Add(interfaceContour, addCurve);
}

Contour* CreateContour(Context* interfaceContext, WindingData winding, size_t size) {
    // reuse existing contour
    OpContext* context = toImplementation(interfaceContext);
    OpContour* contour = context->makeContour(winding, size);
    return toInterface(contour);
}

Contour* Clone(Contour* interfaceContour) {
	OpContour* original = toImplementation(interfaceContour);
	if (original->isEmpty())
		return interfaceContour;
    OpContour* clone = original->context->makeContour(&original->windingStorage.front(), 
            original->windingStorage.size());
    return toInterface(clone);
}

void DeleteContext(Context* interfaceContext) {
    OpContext* context = toImplementation(interfaceContext);
	OP_DEBUG_VALIDATE_CODE(context->debugJoiner = nullptr);
    delete context;
#if OP_DEBUGGER || OP_DEBUG_DUMP
    debugGlobalContext = nullptr;
#endif
}

ContextError Error(Context* interfaceContext) {
    OpContext* context = toImplementation(interfaceContext);
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

void ResetContour(Contour* interfaceContour) {
    OpContour* contour = toImplementation(interfaceContour);
    contour->segments.clear();
}

WindingCondition Resolve(Context* interfaceContext) {
    OpContext* context = toImplementation(interfaceContext);
    if (ContextError::none != context->error) {
        OP_DEBUG_CODE(context->debugData.success = false);
        return -1;
    }
    return context->pathOps();
}

void SetContextCallbacks(Context* interfaceContext, const ContextCallbacks& contextCallbacks) {
    OpContext* context = toImplementation(interfaceContext);
    context->contextCallbacks = contextCallbacks;
}

#if 0
void SetCurveCallbacksX(Context* interfaceContext, int nativeCurveType,
        const CurveCallbacks& curveCallbacks) {
    OpContext* context = toImplementation(interfaceContext);
    context->callbacks.emplace_back();
    context->callbacks.back() = curveCallbacks;
    context->nativeCurveTypes.push_back(nativeCurveType);
}
#endif

void SetCurveCallbacks(Context* interfaceContext, int nativeCurveType,
        const CurveCallbacks& curveCallbacks) {
    OpContext* context = toImplementation(interfaceContext);
    context->callbacks.push_back(curveCallbacks);
    context->nativeCurveTypes.push_back(nativeCurveType);
}

void SetWindingCallbacks(Context* interfaceContext, const WindingCallbacks& windingCallbacks) {
    OpContext* context = toImplementation(interfaceContext);
    WindingCallbacks temp(windingCallbacks);
	if (!temp.windingSubtractFuncPtr)
		temp.windingSubtractFuncPtr = temp.windingAddFuncPtr;
    context->windingCallbacks = temp;
}

} // namespace PathOpsV0Lib
