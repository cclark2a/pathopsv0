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
	, type(WindingType::copy) {
    w = copyData();
	OP_DEBUG_CODE(debugType = DebugWindingType::winding);
}

OpWinding::OpWinding(OpEdge* edge, WindingSum )
	: w({ (ContourPtr) edge->segment->contour, edge->winding.w.data, edge->winding.w.size })
	, type(WindingType::caller) {  // always copy
	OP_ASSERT(WindingType::uninitialized != edge->winding.type);
	zero();
	OP_DEBUG_CODE(debugType = DebugWindingType::sum);
}

#if 0 && TEST_RASTER
OpWinding::OpWinding(const PathOpsV0Lib::Winding* copy, bool curveDown)
	: w({ copy->contour, copy->data, copy->size })
	, type(WindingType::copy) {
	if (!curveDown) {
		zero();
		subtract(*copy);
	}
}
#endif

#if 0
OpWinding::OpWinding(const OpWinding& from) {
	w = from.copyData();
	OP_ASSERT(w.size);
	type = WindingType::copy;
	OP_DEBUG_CODE(debugType = from.debugType);
}
#endif

void OpWinding::add(const PathOpsV0Lib::Winding& winding) {
	copyOnDemand();
    OpContext* context = ((OpContour*) winding.contour)->context;
	context->windingCallbacks.windingAddFuncPtr((ContextPtr) context, w, winding);
}

void OpWinding::add(const OpWinding& winding) {
    add(winding.w);
}

PathOpsV0Lib::Winding OpWinding::copyData() const {
	PathOpsV0Lib::Winding copy { w.contour, 
            ((OpContour*) w.contour)->context->allocateWinding(w.size), w.size };
	std::memcpy(copy.data, w.data, w.size);
	return copy;
}

void OpWinding::copyOnDemand() {
	OP_ASSERT(WindingType::uninitialized != type);
	if (WindingType::copy == type)
		return;
	w = copyData();
	OP_ASSERT(w.size);
	type = WindingType::copy;
}

bool OpWinding::isWound() const {
    OpContext* context = ((OpContour*) w.contour)->context;
    PathOpsV0Lib::WindingVisible woundFunc = context->windingCallbacks.windingWoundFuncPtr;
    return woundFunc ? (*woundFunc)((ContextPtr) context, w) : true;
}

PathOpsV0Lib::WindKeep OpWinding::keep(const OpWinding& sum) const {
    OpContext* context = ((OpContour*) w.contour)->context;
    PathOpsV0Lib::WindingKeep keepFunc = context->windingCallbacks.windingKeepFuncPtr;
    OP_ASSERT(keepFunc);
    return (*keepFunc)((ContextPtr) context, w, sum.w);
}

void OpWinding::setWind(const OpWinding& fromSegment) {
	w = fromSegment.w;
	OP_ASSERT(WindingType::uninitialized == type);
	type = WindingType::caller;  // copy before modify
}

void OpWinding::subtract(const PathOpsV0Lib::Winding& winding) {
	copyOnDemand();
    OpContext* context = ((OpContour*) winding.contour)->context;
	context->windingCallbacks.windingSubtractFuncPtr((ContextPtr) context, w, winding);
}

void OpWinding::subtract(const OpWinding& winding) {
    subtract(winding.w);
}

static const uint8_t zeroes[8] {};

bool OpWinding::visible() const {
    OpContext* context = ((OpContour*) w.contour)->context;
    PathOpsV0Lib::WindingVisible visibleFunc = context->windingCallbacks.windingVisibleFuncPtr;
    if (visibleFunc)
	    return (*visibleFunc)((ContextPtr) context, w);
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

void OpWinding::zero() {
	copyOnDemand();
    OpContext* context = ((OpContour*) w.contour)->context;
    PathOpsV0Lib::WindingZero zeroFunc = context->windingCallbacks.windingZeroFuncPtr;
    if (zeroFunc)
	    return (*zeroFunc)((ContextPtr) context, w);
    // default windings (binary, unary, frame) fit in 8 bytes
    uint8_t* data = (uint8_t*) w.data;
    int size = (int) w.size;  // limit size of winding to 2 gigabytes
    while (size > 0) {
        std::memcpy(data, zeroes, std::min(size, (int) sizeof(zeroes)));
        size -= sizeof(zeroes);
        data += sizeof(zeroes);
    }
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
	context->windingCallbacks.windingZeroFuncPtr((ContextPtr) context, w);
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
