// (c) 2024, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "OpWinding.h"

OpWinding::OpWinding(WindingUninitialized )
	: OP_DEBUG_CODE(w({ nullptr, 0 }), )
	type(WindingType::uninitialized) 
	OP_DEBUG_PARAMS(debugType(DebugWindingType::uninitialized)) {
}

OpWinding::OpWinding(OpContext* context, PathOpsV0Lib::Winding copy)
	: w({ copy.data, copy.size })
	, type(WindingType::copy) {
	w = copyData(context);
	OP_DEBUG_CODE(debugType = DebugWindingType::winding);
}

OpWinding::OpWinding(OpEdge* edge, WindingSum )
	: w({ edge->winding.w.data, edge->winding.w.size })
	, type(WindingType::caller) {  // always copy
	OP_ASSERT(WindingType::uninitialized != edge->winding.type);
	zero(edge->segment->contour->context);
	OP_DEBUG_CODE(debugType = DebugWindingType::sum);
}

OpWinding::OpWinding(OpContext* context, const OpWinding& from) {
	w = from.copyData(context);
	OP_ASSERT(w.size);
	type = WindingType::copy;
	OP_DEBUG_CODE(debugType = from.debugType);
}

void OpWinding::add(OpContext* context, const OpWinding& winding) {
	copyOnDemand(context);
	context->windingCallbacks.windingAddFuncPtr(w, winding.w);
}

PathOpsV0Lib::Winding OpWinding::copyData(OpContext* context) const {
	PathOpsV0Lib::Winding copy { context->allocateWinding(w.size), w.size };
	std::memcpy(copy.data, w.data, w.size);
	return copy;
}

void OpWinding::copyOnDemand(OpContext* context) {
	OP_ASSERT(WindingType::uninitialized != type);
	if (WindingType::copy == type)
		return;
	w = copyData(context);
	OP_ASSERT(w.size);
	type = WindingType::copy;
}

void OpWinding::setWind(const OpWinding& fromSegment) {
	w = fromSegment.w;
	OP_ASSERT(WindingType::uninitialized == type);
	type = WindingType::caller;  // copy before modify
}

void OpWinding::subtract(OpContext* context, const OpWinding& winding) {
	copyOnDemand(context);
	context->windingCallbacks.windingSubtractFuncPtr(w, winding.w);
}

bool OpWinding::visible(OpContext* context) const {
	return context->windingCallbacks.windingVisibleFuncPtr(w);
}

void OpWinding::zero(OpContext* context) {
	copyOnDemand(context);
	context->windingCallbacks.windingZeroFuncPtr(w);
}

void OpWinding::zeroUninitialized(OpContext* context, const OpWinding& winding) {
	if (WindingType::copy == type)
		return;
	OP_ASSERT(WindingType::uninitialized == type);
	OP_ASSERT(winding.w.size);
	w = { context->allocateWinding(winding.w.size), winding.w.size };
	type = WindingType::copy;
	context->windingCallbacks.windingZeroFuncPtr(w);
}

void OpWinding::move(OpContext* context, const OpWinding& opp, bool backwards) {
	if (backwards)
		subtract(context, opp);
	else
		add(context, opp);
}
