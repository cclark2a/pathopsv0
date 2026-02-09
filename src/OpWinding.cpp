// (c) 2024, Cary Clark cclark2@gmail.com
#include "OpContext.h"
#include "OpWinding.h"

OpWinding::OpWinding(WindingUninitialized )
	: OP_DEBUG_CODE(w({ nullptr, nullptr, 0 }), )
	type(WindingType::uninitialized) 
	OP_DEBUG_PARAMS(debugType(DebugWindingType::uninitialized)) {
}

OpWinding::OpWinding(const PathOpsV0Lib::Winding& copy)
	: w({ copy.contour, copy.data, copy.size })
#if 0  // !!! looks wrong; instead, allow copyOnDemand() to copy when needed?
	, type(WindingType::copy) {
    w = copyData();
#else
	, type(WindingType::caller) {
#endif
	OP_DEBUG_CODE(debugType = DebugWindingType::winding);
}

OpWinding::OpWinding(OpEdge* edge, WindingSum )
	: w({ (ContourPtr) edge->segment->contour, edge->winding.w.data, edge->winding.w.size })
	, type(WindingType::caller) {  // always copy
	OP_ASSERT(WindingType::uninitialized != edge->winding.type);
	zero();
	OP_DEBUG_CODE(debugType = DebugWindingType::sum);
}

#if OP_TEST
OpWinding::OpWinding(OpContour* contour, PathOpsV0Lib::WindingData wind, size_t size)
	: w({ (ContourPtr) contour, wind, size })
	, type(WindingType::copy) { 
	w = copyData();
	OP_DEBUG_CODE(debugType = DebugWindingType::winding);
}
#endif

#if OP_TEST_RASTER
OpWinding::OpWinding(DebugWindingRaster )
	: OP_DEBUG_CODE(w({ nullptr, nullptr, 0 }), )
	type(WindingType::uninitialized) 
	OP_DEBUG_PARAMS(debugType(DebugWindingType::uninitialized)) 
	, usedByRaster(true) {
}

OpWinding::OpWinding(const OpWinding& winding, DebugWindingSum ) 
	: w({ (ContourPtr) winding.w.contour, winding.w.data, winding.w.size })
	, type(WindingType::caller)  // always copy
	, usedByRaster(true) {
	zero();
	OP_DEBUG_CODE(debugType = DebugWindingType::sum);
}

OpWinding::OpWinding(OpContext* context, DebugWindingZero)
	: w({ nullptr, nullptr, 0 })
	, type(WindingType::caller) // always copy
	, usedByRaster(true) {
	PathOpsV0Lib::ContextCount countFuncPtr = context->contextCallbacks.windingBytesFuncPtr;
	OP_ASSERT(context->contourStorage);  // some contour is required so code can retrieve context
	OP_ASSERT(context->contourStorage->used);  // there must be at least one contour
	w.contour = (ContourPtr) context->contourStorage->storage;
	if (countFuncPtr)
		w.size = (*countFuncPtr)((ContextPtr) context);
	else {  // if windings bytes function is not provided: then
		w.size = ((OpContour*) w.contour)->windingStorage.size();
		for (OpContour* test : context->contours)  // all contours must have the same winding size
			OP_ASSERT(w.size == test->windingStorage.size());
	}
	w.data = context->allocateWinding(w.size, true);
	zeroCommon();
}

OpWinding::OpWinding(const PathOpsV0Lib::Winding& wind, DebugWindingRef )
	: w(wind)
	, type(WindingType::copy)
	, debugType(DebugWindingType::winding)    // treat as already copied
	, usedByRaster(true) {
}

#endif

void OpWinding::add(const PathOpsV0Lib::Winding& winding) {
	copyOnDemand();
    OpContext* context = ((OpContour*) winding.contour)->context;
	context->windingCallbacks.windingAddFuncPtr(w, winding);
}

void OpWinding::add(const OpWinding& winding) {
    add(winding.w);
}

PathOpsV0Lib::Winding OpWinding::copyData() const {
	OpContext* context = ((OpContour*) w.contour)->context;
	PathOpsV0Lib::Winding copy { w.contour, 
            context->allocateWinding(w.size  OP_DEBUG_RASTER_PARAMS(usedByRaster)), w.size };
	std::memcpy(copy.data, w.data, w.size);
	return copy;
}

#if OP_TEST_RASTER
void OpWinding::copyExisting(const OpWinding& existing) {
	if (WindingType::caller == type || w.size != existing.w.size) {
		w = existing.copyData();
		type = WindingType::copy;
		return;
	}
	std::memcpy(w.data, existing.w.data, w.size);
}
#endif

bool OpWinding::copyOnDemand() {
	OP_ASSERT(WindingType::uninitialized != type);
	if (WindingType::copy == type)
		return false;
	w = copyData();
	OP_ASSERT(w.size);
	type = WindingType::copy;
	return true;
}

#if OP_TEST_RASTER
void OpWinding::debugZero() {
	OP_ASSERT(WindingType::uninitialized == type);
    zeroCommon();
}
#endif

bool OpWinding::isWound() const {
    OpContext* context = ((OpContour*) w.contour)->context;
    PathOpsV0Lib::WindingVisible woundFunc = context->windingCallbacks.windingWoundFuncPtr;
    return woundFunc ? (*woundFunc)(w) : true;
}

PathOpsV0Lib::WindKeep OpWinding::keep(const OpWinding& sum) const {
    OpContext* context = ((OpContour*) w.contour)->context;
    PathOpsV0Lib::WindingKeep keepFunc = context->windingCallbacks.windingKeepFuncPtr;
    OP_ASSERT(keepFunc);
    return (*keepFunc)(w, sum.w);
}

void OpWinding::setWind(const OpWinding& fromSegment) {
	w = fromSegment.w;
	OP_ASSERT(WindingType::uninitialized == type);
	type = WindingType::caller;  // copy before modify
}

void OpWinding::subtract(const PathOpsV0Lib::Winding& winding) {
	copyOnDemand();
    OpContext* context = ((OpContour*) winding.contour)->context;
	context->windingCallbacks.windingSubtractFuncPtr(w, winding);
}

void OpWinding::subtract(const OpWinding& winding) {
    subtract(winding.w);
}

static const uint8_t zeroes[8] {};

bool OpWinding::visible() const {
    OpContext* context = ((OpContour*) w.contour)->context;
    PathOpsV0Lib::WindingVisible visibleFunc = context->windingCallbacks.windingVisibleFuncPtr;
    if (visibleFunc)
	    return (*visibleFunc)(w);
    // default windings (binary, unary, frame) fit in 8 bytes
    const uint8_t* data = (const uint8_t*) w.data;
    int size = (int) w.size;  // limit size of winding to 2 gigabytes
    while (size > 0) {
        if (std::memcmp(data, zeroes, std::min(size, (int) sizeof(zeroes))))
            return true;
        size -= sizeof(zeroes);
        data += sizeof(zeroes);
    }
    return false;
}

void OpWinding::zeroCommon() {
    OpContext* context = ((OpContour*) w.contour)->context;
    PathOpsV0Lib::WindingZero zeroFunc = context->windingCallbacks.windingZeroFuncPtr;
    if (zeroFunc)
	    return (*zeroFunc)(w);
    // default windings (binary, unary, frame) fit in 8 bytes
    uint8_t* data = (uint8_t*) w.data;
    int size = (int) w.size;  // limit size of winding to 2 gigabytes
    while (size > 0) {
        std::memcpy(data, zeroes, std::min(size, (int) sizeof(zeroes)));
        size -= sizeof(zeroes);
        data += sizeof(zeroes);
    }
}

void OpWinding::zero() {
	copyOnDemand();
	zeroCommon();
}

#if 0
void OpWinding::zeroUninitialized(const PathOpsV0Lib::Winding& winding) {
	if (WindingType::copy == type)
		return;
	OP_ASSERT(WindingType::uninitialized == type);
	OP_ASSERT(winding.size);
    OpContext* context = ((OpContour*) winding.contour)->context;
	w = { winding.contour, context->allocateWinding(winding.size), winding.size };
	type = WindingType::copy;
	context->windingCallbacks.windingZeroFuncPtr(w);
}

void OpWinding::zeroUninitialized(const OpWinding& winding) {
    zeroUninitialized(winding.w);
}
#endif

void OpWinding::move(const OpWinding& opp, bool backwards) {
	if (backwards)
		subtract(opp);
	else
		add(opp);
}
