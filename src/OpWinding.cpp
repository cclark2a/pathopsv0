// (c) 2024, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "OpWinding.h"

OpWinding::OpWinding(OpContour* c, PathOpsV0Lib::Winding copy)
	: contour(c)
	OP_DEBUG_PARAMS(debugType(WindingType::copy)) {
	w.data = contour->contours->allocateWinding(copy.size);
	std::memcpy(w.data, copy.data, copy.size);
	w.size = copy.size;
}

OpWinding::OpWinding(OpEdge* edge, WindingSum )
	: contour(edge->segment->contour)
	OP_DEBUG_PARAMS(debugType(WindingType::sum)) {
	w = edge->winding.copyData();
	zero();
}

OpWinding& OpWinding::operator=(const OpWinding& from) {
	contour = from.contour;
	w = from.copyData();
	return *this;
}

OpWinding::OpWinding(const OpWinding& from) {
	contour = from.contour;
	w = from.copyData();
}

void OpWinding::add(const OpWinding& winding) {
	contour->callBacks.windingAddFuncPtr(w, winding.w);
}

#if 0
// returns true if not equal
bool OpWinding::equal(PathOpsV0Lib::Winding comp) const {
	return w.size == comp.size && !memcmp(w.data, comp.data, w.size);
}
#endif

PathOpsV0Lib::Winding OpWinding::copyData() const {
	OpContours* contours = contour->contours;
	PathOpsV0Lib::Winding copy { contours->allocateWinding(w.size), w.size };
	std::memcpy(copy.data, w.data, w.size);
	return copy;
}

void OpWinding::subtract(const OpWinding& winding) {
	contour->callBacks.windingSubtractFuncPtr(w, winding.w);
}

bool OpWinding::visible() const {
	return contour->callBacks.windingVisibleFuncPtr(w);
}

void OpWinding::zero() {
	contour->callBacks.windingZeroFuncPtr(w);
}

void OpWinding::move(const OpWinding& opp, bool backwards) {
	if (backwards)
		contour->callBacks.windingSubtractFuncPtr(w, opp.w);
	else
		contour->callBacks.windingAddFuncPtr(w, opp.w);
}
