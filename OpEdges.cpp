#include "OpContour.h"
#include "OpEdgeIntersect.h"
#include "OpEdges.h"
#include "OpSegment.h"

OpEdges::OpEdges(OpContours& contours, EdgesToSort edgesToSort) {
	for (auto& contour : contours.contours) {
		for (auto& segment : contour.segments) {
			if (pointType == segment.c.type)
				continue;
			for (auto& edge : segment.edges) {
				addEdge(&edge, edgesToSort);
			}
		}
	}
	sort(edgesToSort);
}

OpEdges::OpEdges(OpEdge* sEdge, OpEdge* oEdge) {
	addEdge(sEdge, EdgesToSort::byCenter);
	addEdge(oEdge, EdgesToSort::byCenter);
#if OP_DEBUG_IMAGE
	OpDebugImage::add(sEdge);
	OpDebugImage::add(oEdge);
#endif
}

void OpEdges::addEdge(OpEdge* edge, EdgesToSort edgesToSort) {
	if (!edge->winding.visible())	// skip edges no longer visible
		return;
	if (EdgesToSort::byBox == edgesToSort || edge->ptBounds.height())
		inX.push_back(edge);
	if (EdgesToSort::byCenter == edgesToSort && edge->ptBounds.width())
		inY.push_back(edge);
}

IntersectResult OpEdges::CoincidentCheck(OpPtT aPtT, OpPtT bPtT, OpPtT cPtT, OpPtT dPtT,
		OpSegment* segment, OpSegment* oppSegment) {
	OpVector abDiff = aPtT.pt - bPtT.pt;
	XyChoice xyChoice = fabsf(abDiff.dx) < fabsf(abDiff.dy) ? XyChoice::inY : XyChoice::inX;
	float A = aPtT.pt.choice(xyChoice);
	float B = bPtT.pt.choice(xyChoice);
	assert(A != B);
	float C = cPtT.pt.choice(xyChoice);
	float D = dPtT.pt.choice(xyChoice);
	assert(C != D);
	bool flipped = A < B != C < D;
	bool AinCD = OpMath::Between(C, A, D);
	bool BinCD = OpMath::Between(C, B, D);
	if (AinCD && BinCD)
		return AddPair(xyChoice, aPtT, bPtT, cPtT, dPtT, flipped, segment, oppSegment);
	bool CinAB = OpMath::Between(A, C, B);
	bool DinAB = OpMath::Between(A, D, B);
	if (CinAB && DinAB)
		return AddPair(xyChoice, cPtT, dPtT, aPtT, bPtT, flipped, oppSegment, segment);
	if (!AinCD && !BinCD)
		return IntersectResult::no;
	assert(CinAB || DinAB);
	float AorB = AinCD ? A : B;
	OpPtT ptTAorB = AinCD ? aPtT : bPtT;
	float CorD = CinAB ? C : D;
	OpPtT ptTCorD = CinAB ? cPtT : dPtT;
	if (AorB == CorD) {
		segment->add(ptTAorB, oppSegment  OP_DEBUG_PARAMS(IntersectMaker::addCoincidentCheck1));
		oppSegment->add(ptTCorD, segment  OP_DEBUG_PARAMS(IntersectMaker::addCoincidentCheck2));
		return IntersectResult::yes;
	}
	// pass a mix of seg and opp; construct one t for each
	int coinID = segment->coinID(flipped);
	AddMix(xyChoice, ptTAorB, flipped, cPtT, dPtT, segment, oppSegment, coinID);
	AddMix(xyChoice, ptTCorD, flipped, aPtT, bPtT, oppSegment, segment, coinID);
	return IntersectResult::yes;
}

// this edge has points A, B; opp edge has points C, D
// adds 0: no intersection; 1: end point only; 2: partial or full coincidence
IntersectResult OpEdges::CoincidentCheck(const OpEdge& edge, const OpEdge& opp) {
	return OpEdges::CoincidentCheck(edge.start, edge.end, opp.start, opp.end,
			const_cast<OpSegment*>(edge.segment), const_cast<OpSegment*>(opp.segment));
}

void OpEdges::AddMix(XyChoice xyChoice, OpPtT ptTAorB, bool flipped,
		OpPtT cPtT, OpPtT dPtT, OpSegment* segment, OpSegment* oppSegment, int coinID) {
	float eStart = ptTAorB.pt.choice(xyChoice);
	if (flipped)
		std::swap(cPtT, dPtT);
	float oStart = cPtT.pt.choice(xyChoice);
	float oEnd = dPtT.pt.choice(xyChoice);
	float oTRange = dPtT.t - cPtT.t;
	OpPtT oCoinStart { ptTAorB.pt, cPtT.t + (eStart - oStart) / (oEnd - oStart) * oTRange };
	assert(OpMath::Between(cPtT.t, oCoinStart.t, dPtT.t));
	segment->add(ptTAorB, oppSegment, coinID  OP_DEBUG_PARAMS(IntersectMaker::addMix1));
	oppSegment->add(oCoinStart, segment, coinID  OP_DEBUG_PARAMS(IntersectMaker::addMix2));
}

IntersectResult OpEdges::AddPair(XyChoice xyChoice, OpPtT aPtT, OpPtT bPtT, OpPtT cPtT, OpPtT dPtT,
		bool flipped, OpSegment* segment, OpSegment* oppSegment) {
	float eStart = aPtT.pt.choice(xyChoice);
	float eEnd = bPtT.pt.choice(xyChoice);
	if (flipped)
		std::swap(cPtT, dPtT);
	float oStart = cPtT.pt.choice(xyChoice);
	float oEnd = dPtT.pt.choice(xyChoice);
	float oTRange = dPtT.t - cPtT.t;
	OpPtT oCoinStart { aPtT.pt, cPtT.t + (eStart - oStart) / (oEnd - oStart) * oTRange };
	OpPtT oCoinEnd { bPtT.pt, cPtT.t + (eEnd - oStart) / (oEnd - oStart) * oTRange };
	assert(OpMath::Between(cPtT.t, oCoinStart.t, dPtT.t));
	assert(OpMath::Between(cPtT.t, oCoinEnd.t, dPtT.t));
	int coinID = segment->coinID(flipped);
	segment->add(aPtT, oppSegment, coinID  OP_DEBUG_PARAMS(IntersectMaker::addPair1));
	segment->add(bPtT, oppSegment, coinID  OP_DEBUG_PARAMS(IntersectMaker::addPair2));
	oppSegment->add(oCoinStart, segment, coinID  OP_DEBUG_PARAMS(IntersectMaker::addPair3));
	oppSegment->add(oCoinEnd, segment, coinID  OP_DEBUG_PARAMS(IntersectMaker::addPair4));
	return IntersectResult::yes;
}

IntersectResult OpEdges::AddIntersection(OpEdge& opp, const OpEdge& edge) {
	auto alreadyContains = [](const std::vector<OpIntersection>& sects, const OpPtT& edgePtT,
			const OpSegment* segment) {
		for (auto& sect : sects) {
			if (segment == sect.segment && (edgePtT.pt == sect.ptT.pt || edgePtT.t == sect.ptT.t))
				return true;
		}
		return false;
	};
	if (opp.segment == edge.segment && edge.ptBounds.touches(opp.ptBounds)
			&& (opp.start == edge.end || opp.end == edge.start)) {
		return IntersectResult::no;
	}
//	OpDebugBreak(&edge, 1, 2 == opp.id);
	OpRoots septs;
	assert(edge.isLine_impl);
	std::array<OpPoint, 2> edgePts { edge.start.pt, edge.end.pt };
	const OpCurve& oppCurve = opp.setCurve();
	septs.count = oppCurve.rayIntersect(edgePts, septs.roots);
//	assert(septs.count < 3);	// while it should never be 3, it's OK to accept those intersections
	if (!septs.count)
		return IntersectResult::no;
	// Note that coincident check does not receive intercepts as a parameter; in fact, the intercepts
	// were not calculated (the roots are uninitialized). This is because coincident check will 
	// compute the actual coincident start and end without the roots introducing error.
	if (opp.setLinear() && 2 == septs.count)
		return CoincidentCheck(edge, opp);
	int foundCount = 0;
	for (unsigned index = 0; index < septs.count; ++index) {
		float oppT = OpMath::Interp(opp.start.t, opp.end.t, septs.get(index));
		OpPtT oppPtT { oppCurve.ptAtT(septs.get(index)), oppT };
		float edgeT = edge.findPtT(oppPtT.pt);
		if (OpMath::Between(0, edgeT, 1)) {
            // pin point to both bounds, but only if it is on edge
			OpSegment* eSegment = const_cast<OpSegment*>(edge.segment);
			OpSegment* oSegment = const_cast<OpSegment*>(opp.segment);
            oSegment->tightBounds.pin(&oppPtT.pt);
            eSegment->tightBounds.pin(&oppPtT.pt);
			OpPtT edgePtT { oppPtT.pt, OpMath::Interp(edge.start.t, edge.end.t, edgeT) };
#if OP_DEBUG
			if (148 == eSegment->contour->contours->id)
				OpDebugOut("");
#endif
			if (!alreadyContains(eSegment->intersections, edgePtT, oSegment))
				eSegment->add(edgePtT, oSegment  OP_DEBUG_PARAMS(IntersectMaker::addIntersection1));
//			OpDebugBreak(oSegment, 4, 265 == debugGlobalContours->id);
			if (!alreadyContains(oSegment->intersections, oppPtT, eSegment))
				oSegment->add(oppPtT, eSegment  OP_DEBUG_PARAMS(IntersectMaker::addIntersection2));
			++foundCount;
		}
	}
	//	assert(foundCount < 2);
	return foundCount ? IntersectResult::yes : IntersectResult::no;
}

FoundIntersections OpEdges::findIntersections() {
#if OP_DEBUG_COMPARE
	OpDebugCompare debugCompare("innerFindSect");
#endif
	for (auto edgeIter = inX.begin(); edgeIter != inX.end(); ++edgeIter) {
		OpEdge* edge = const_cast<OpEdge*>(*edgeIter);
		for (auto oppIter = edgeIter + 1; oppIter != inX.end(); ++oppIter) {
			OpEdge* opp = const_cast<OpEdge*>(*oppIter);
#if OP_DEBUG
			if ((edge->id == 57 || edge->id == 49) && (opp->id == 57 || opp->id == 49))
				OpDebugOut("");
#endif
#if OP_DEBUG_COMPARE
			debugCompare.edges(edge, opp);
#endif
			if (edge->ptBounds.right < opp->ptBounds.left)
				break;
			if (!edge->ptBounds.intersects(opp->ptBounds))
				continue;
			if (edge->segment == opp->segment && cubicType != edge->segment->c.type)	
				continue;  // non-cubic handled in opsegment makeedges
			// for line-curve intersection we can directly intersect
			if (lineType != edge->segment->c.type && edge->setLinear()) {
				(void)AddIntersection(*opp, *edge);
				continue;
			} else if (lineType != opp->segment->c.type && opp->setLinear()) {
				(void)AddIntersection(*edge, *opp);
				continue;
			}
			// if the curves' bounds share only a single intersection edge, check endpoints only
			if (edge->ptBounds.touches(opp->ptBounds)) {
				OpSegment* eSegment = const_cast<OpSegment*>(edge->segment);
				OpSegment* oSegment = const_cast<OpSegment*>(opp->segment);
				for (auto& edgePtT : { edge->start, edge->end }) {
					for (auto& oppPtT : { opp->start, opp->end }) {
						// end matching end was handled earlier in op segments find intersection
						if (edgePtT.pt != oppPtT.pt)
							continue;
						if (eSegment == oSegment && edgePtT.t == oppPtT.t)
							continue;
						if (OpMath::IsInt(edgePtT.t) && OpMath::IsInt(oppPtT.t))
							continue;
						if (eSegment->containsSect(edgePtT.t, oSegment)
								&& oSegment->containsSect(oppPtT.t, eSegment))
							continue;
						eSegment->add(edgePtT, oSegment
								OP_DEBUG_PARAMS(IntersectMaker::findIntersections1));
						oSegment->add(oppPtT, eSegment
								OP_DEBUG_PARAMS(IntersectMaker::findIntersections2));
					}
				}
				continue;
			}
			if (lineType == edge->segment->c.type || lineType == opp->segment->c.type)
				continue;
			OpEdgeIntersect opEdgeIntersect(edge, opp);
			SectFound result = opEdgeIntersect.divideAndConquer();
			if (SectFound::fail == result)
				return FoundIntersections::fail;
		}
	}
#if OP_DEBUG_COMPARE
	debugCompare.close();
#endif
	return FoundIntersections::yes;
}

struct EdgeDistance {
	EdgeDistance(OpEdge* e, float d, float _t)
		: edge(e)
		, distance(d)
		, t(_t) {
	}

	OpEdge* edge;
	float distance;
	float t;
};

// note: sorts from high to low
static bool compareDistance(const EdgeDistance& s1, const EdgeDistance& s2) {
	return s1.distance > s2.distance;
}

// at some point, do some math or rigorous testing to figure out how extreme this can be
// for now, keep making it smaller until it breaks
#define WINDING_NORMAL_LIMIT  0.001 // !!! no idea what this should be

// if horizontal axis, look at rect top/bottom
void OpEdges::setSumChain(std::vector <OpEdge*>& inArray, size_t inIndex, Axis axis) {
	// see if normal at center point is in direction of ray
	OpEdge* edge = inArray[inIndex];
	assert(edge->winding.visible());
	const OpSegment* edgeSeg = edge->segment;
	OpVector ray = Axis::horizontal == axis ? OpVector{ 1, 0 } : OpVector{ 0, 1 };
	float NxR = edgeSeg->c.tangent(edge->center.t).normalize().cross(ray);
	if (fabs(NxR) < WINDING_NORMAL_LIMIT) {
		if (ray.dy) {
			// This is the second pass. If the edge is tiny, it is reasonable to ignore it.
			edge->winding.zero(ZeroReason::failWinding);
			// !!! strict 'is tiny' check -- figure out what this should be
			if (edge->end.t - edge->start.t <= OpEpsilon)
				edge->unsortable = true;
			else 
				edge->fail = EdgeFail::winding;
		}
		return;
	}
	// intersect normal with every edge in the direction of ray until we run out 
	Axis perpendicular = !axis;
	OpVector backRay = -ray;
	float center = edge->center.pt.choice(perpendicular);
	float normal = edge->center.pt.choice(axis);
	float newMid = .5;
	float newMidEnd = 1;
	// mark all possible edges
	inIndex += 1;
	for (; inIndex < inArray.size(); ++inIndex) {
		OpEdge* advance = inArray[inIndex];
		if (advance->ptBounds.ltChoice(perpendicular) > center)
			break;
	}
	std::vector<EdgeDistance> distance;
	distance.emplace_back(edge, 0, edge->center.t);
	// start at edge with left equal to or left of center
	while (inIndex != 0) {
		OpEdge* test = inArray[--inIndex];
		if (test == edge)
			continue;
		if (!test->winding.visible())
			continue;
		if (test->ptBounds.ltChoice(axis) > normal)
			continue;
		if (test->ptBounds.rbChoice(axis) < normal)
			continue;
		OpRoots cepts;
		do {	// try to find ray; look up to eight times
//			OpDebugBreak(edge, 129, 92 == test->id);
			const OpCurve& curve = test->setCurve();
			cepts.count = curve.axisRayHit(axis, normal, cepts.roots);
			// get the normal at the intersect point and see if it is usable
			if (1 != cepts.count) {
				// !!! if intercepts is 2 or 3, figure out why (and what to do)
				// !!! likely need to try a different ray
				assert(0 == cepts.count);
				break;
			}
			float cept = cepts.get(0);
			if (!OpMath::IsNaN(cept) && 0 != cept && 1 != cept) {
				float tNxR = curve.tangent(cept).normalize().cross(backRay);
				if (fabs(tNxR) >= WINDING_NORMAL_LIMIT
						|| (test->setLinear() && test->start.t < cept && cept < test->end.t)) {
					OpPoint pt = curve.ptAtT(cept);
					distance.emplace_back(test, center - pt.choice(perpendicular), cept);
					break;	// i.e., no need to look for a better ray intersection
				}
			}
			// recalc center; restart search for winding with a different ray
			const OpCurve& edgeCurve = edge->setCurve();
			newMid /= 2;
			newMidEnd = newMidEnd < .5 ? 1 - newMid : newMid;
			float middle = OpMath::Interp(edge->ptBounds.ltChoice(axis),
					edge->ptBounds.rbChoice(axis), newMidEnd);
			float t = edgeCurve.center(axis, middle);
			center = edgeCurve.ptAtT(t).choice(perpendicular);
			normal = edgeCurve.ptAtT(t).choice(axis);
			if (OpMath::IsNaN(t) || newMid <= 1.f/256.f) {	// give it at most eight tries
				edge->fail = EdgeFail::recalcCenter;
				edge->winding.zero(ZeroReason::failCenter);
				return;
			}
			assert(!OpMath::IsNaN(normal));
		} while (true);
	}
	std::sort(distance.begin(), distance.end(), compareDistance);	// sorts from high to low
	EdgeDistance* last = &distance.front();
	// leftmost edge may have no prior, no next; if the right edge has no effect
	while (last->edge != edge) {
		EdgeDistance* next = last + 1;
		OpEdge* lastEdge = last->edge;
		OpEdge* nextEdge = next->edge;
		float delta = last->distance - next->distance;
		if (!lastEdge->nextSum) {
//			OpDebugBreak(lastEdge, 129, 92 == nextEdge->id);
			lastEdge->nextSum = nextEdge;
			lastEdge->nextAxis = axis;
			lastEdge->unsortable |= delta <= OpEpsilon;
		}
		if (!nextEdge->priorSum) {
//			OpDebugBreak(nextEdge, 129, 92 == lastEdge->id);
			nextEdge->priorSum = lastEdge;
			nextEdge->priorAxis = axis;
			nextEdge->priorT = last->t;
			nextEdge->unsortable |= delta <= OpEpsilon;
		}
		last = next;
	}
	if (last != &distance.back()) {
//		OpDebugBreak(edge, 129, 92 == (last + 1)->edge->id);
		edge->nextSum = (last + 1)->edge;
		edge->nextAxis = axis;
	}
}

FoundWindings OpEdges::setWindings() {
	OP_DEBUG_CODE(OpDebugIntersectSave save(OpDebugIntersect::edge));
	for (size_t index = 0; index < inX.size(); ++index)
		setSumChain(inX, index, Axis::horizontal);
	for (size_t index = 0; index < inX.size(); ++index) {
		OpEdge* edge = inX[index];
		if (!edge->winding.visible())
			continue;
		OpDebugBreak(edge, 294, true);
		bool loops;
		do {
			loops = false;
			// seen prior, seen next are set so edge is checked only once for looping
			if (!edge->seenPrior && (loops = edge->priorSumLoops()))
				edge->markFailPrior(inX, Axis::horizontal);
			if (!edge->seenNext && (loops |= edge->nextSumLoops()))
				edge->markFailNext(inX, Axis::horizontal);
			if (loops && edge->winding.visible())
				setSumChain(inX, index, Axis::horizontal);
		} while (loops);
	}
	for (auto& edge : inX) {
		if (edge->sum.unset()) {
			OP_DEBUG_CODE(int debugWindingLimiter = 0);
			edge->findWinding(Axis::horizontal  OP_DEBUG_PARAMS(&debugWindingLimiter));
		}
	}
	for (auto& edge : inX)
		if (edge->sum.unset() && !edge->unsortable)
			return FoundWindings::fail;
	for (size_t index = 0; index < inY.size(); ++index) {
		OpEdge* edge = inY[index];
		if (edge->winding.visible() && edge->sum.unset())
			setSumChain(inY, index, Axis::vertical);
	}
	for (size_t index = 0; index < inY.size(); ++index) {
		OpEdge* edge = inY[index];
		if (!edge->winding.visible())
			continue;
		if (!edge->sum.unset())
			continue;
		bool loops;
		do {
			loops = false;
			if (!edge->seenPrior && (loops = edge->priorSumLoops()))
				edge->markFailPrior(inY, Axis::vertical);
			if (!edge->seenNext && (loops |= edge->nextSumLoops()))
				edge->markFailNext(inY, Axis::vertical);
			if (loops && edge->winding.visible())
				setSumChain(inY, index, Axis::vertical);
		} while (loops);
	}
	for (auto& edge : inY) {
		if (edge->sum.unset()) {
			OP_DEBUG_CODE(int debugWindingLimiter = 0);
			edge->findWinding(Axis::vertical  OP_DEBUG_PARAMS(&debugWindingLimiter));
		}
	}
	for (auto& edge : inY)
		if (edge->sum.unset() && !edge->unsortable)
			return FoundWindings::fail;
	return FoundWindings::yes;
}

static bool compareXBox(const OpEdge* s1, const OpEdge* s2) {
	const OpRect& r1 = s1->ptBounds;
	const OpRect& r2 = s2->ptBounds;
	if (r1.left < r2.left)
		return true;
	if (r1.left > r2.left)
		return false;
	if (r1.left == r2.left && r1.right < r2.right)
		return true;
	if (r1.left == r2.left && r1.right > r2.right)
		return false;
	return s1->id < s2->id;
}

// starting at left (-x), increasing
static bool compareXCenter(const OpEdge* s1, const OpEdge* s2) {
	return s1->ptBounds.left < s2->ptBounds.left;
}

// starting at top (-y), increasing
static bool compareYCenter(const OpEdge* s1, const OpEdge* s2) {
	return s1->ptBounds.top < s2->ptBounds.top;
}

void OpEdges::sort(EdgesToSort sortBy) {
	if (EdgesToSort::byBox == sortBy) {
		std::sort(inX.begin(), inX.end(), compareXBox);
		return;
	}
	assert(EdgesToSort::byCenter == sortBy);
	std::sort(inX.begin(), inX.end(), compareXCenter);
	std::sort(inY.begin(), inY.end(), compareYCenter);
}
