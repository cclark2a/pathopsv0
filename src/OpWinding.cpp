// (c) 2024, Cary Clark cclark2@gmail.com
#include "OpContour.h"
#include "OpWinding.h"

OpWinding::OpWinding(OpContour* c, PathOpsV0Lib::Winding copy)
    : contour(c)
    OP_DEBUG_PARAMS(debugType(WindingType::copy)) {
    w.data = contour->contours->allocateWinding(copy.size);
	memcpy(w.data, copy.data, copy.size);
    w.size = copy.size;
}

OpWinding::OpWinding(OpEdge* edge, WindingSum )
    : contour(edge->segment->contour)
    OP_DEBUG_PARAMS(debugType(WindingType::sum)) {
    w = edge->winding.copyData();
    zero();
}

void OpWinding::add(const OpWinding& winding) {
    w = winding.contour->callBacks.windingAddFuncPtr(w, winding.w);
}

// returns true if not equal
bool OpWinding::compare(PathOpsV0Lib::Winding comp) const {
    return w.size != comp.size || memcmp(w.data, comp.data, w.size);
}

PathOpsV0Lib::Winding OpWinding::copyData() const {
    OpContours* contours = contour->contours;
    PathOpsV0Lib::Winding copy { contours->allocateWinding(w.size), w.size };
    memcpy(copy.data, w.data, w.size);
    return copy;
}

void OpWinding::subtract(const OpWinding& winding) {
    w = winding.contour->callBacks.windingSubtractFuncPtr(w, winding.w);
}

bool OpWinding::visible() const {
    return contour->callBacks.windingVisibleFuncPtr(w);
}

void OpWinding::zero() {
    contour->callBacks.windingZeroFuncPtr(w);
}

#if OP_TEST_NEW_INTERFACE
void OpWinding::move(const OpWinding& opp, bool backwards) {
	if (backwards)
		contour->callBacks.windingSubtractFuncPtr(w, opp.w);
	else
		contour->callBacks.windingAddFuncPtr(w, opp.w);
}
#else
void OpWinding::move(OpWinding opp, const OpContours* contours, bool backwards) {
	OP_ASSERT(WindingType::winding == debugType);
	OP_ASSERT(WindingType::winding == opp.debugType);
	if (OpFillType::winding == contours->left)
		left_impl += backwards ? -opp.left() : opp.left();
	else
		left_impl ^= opp.left();
	if (OpFillType::winding == contours->right)
		right_impl += backwards ? -opp.right() : opp.right();
	else
		right_impl ^= opp.right();
}
#endif

#if !OP_TEST_NEW_INTERFACE
void OpWinding::setSum(OpWinding winding, const OpContours* contours) {
	OP_ASSERT(WindingType::uninitialized == debugType);
	OP_ASSERT(WindingType::temp == winding.debugType);
	OP_DEBUG_CODE(debugType = WindingType::sum);
	left_impl = winding.left() & contours->leftFillTypeMask();
	right_impl = winding.right() & contours->rightFillTypeMask();
}
#endif
