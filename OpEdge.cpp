#include "OpEdge.h"
#include "OpContour.h"

OpEdge::OpEdge(const OpEdge* e, OpPtT newPtT, NewEdge isLeftRight  
		OP_DEBUG_PARAMS(EdgeMaker maker, int line, std::string file, const OpIntersection* i1, 
		const OpIntersection* i2))
	: OpEdge() {
	segment = e->segment;
	start = NewEdge::isLeft == isLeftRight ? e->start : newPtT;
	end = NewEdge::isLeft == isLeftRight ? newPtT : e->end;
#if OP_DEBUG
	debugStart = i1;
	debugEnd = i2;
	debugMaker = maker;
	debugMakerLine = line;
	debugMakerFile = file;
	debugParentID = e->id;
#endif	
	complete();
}

// called after winding has been modified by other coincident edges
OpEdge::OpEdge(const OpEdge* edge, const OpPtT& s, const OpPtT& e  
		OP_DEBUG_PARAMS(EdgeMaker maker, int line, std::string file, const OpIntersection* i1,
		const OpIntersection* i2))
	: OpEdge() {
	id = segment->contour->contours->id++;
	segment = edge->segment;
	start = s;
	end = e;
#if OP_DEBUG
	debugStart = i1;
	debugEnd = i2;
	debugMaker = maker;
	debugMakerLine = line;
	debugMakerFile = file;
	debugParentID = edge->id;
#endif	
	winding = edge->winding;
	subDivide();	// uses already computed points stored in edge
}

CalcFail OpEdge::addIfUR(Axis axis, float t, OpWinding* sumWinding) {
	NormalDirection NdotR = normalDirection(axis, t);
	if (NormalDirection::upRight == NdotR)
		*sumWinding += winding;
	else if (NormalDirection::downLeft != NdotR)
		OP_DEBUG_FAIL(*this, CalcFail::fail);
	return CalcFail::none;
}

// given an intersecting ray and edge t, add or subtract edge winding to sum winding
// but don't change edge's sum, since an unsectable edge does not allow that accumulation
CalcFail OpEdge::addSub(Axis axis, float t, OpWinding* sumWinding) {
	NormalDirection NdotR = normalDirection(axis, t);
	if (NormalDirection::upRight == NdotR)
		*sumWinding += winding;
	else if (NormalDirection::downLeft == NdotR)
		*sumWinding -= winding;
	else
		OP_DEBUG_FAIL(*this, CalcFail::fail);
	return CalcFail::none;
}

#if 0
// start here;
// !!! replace sort with sort-and-remove-duplicates ?
void OpEdge::addMatchingEnds(const OpEdge& opp) const {
	if (opp.segment == segment && (opp.start == end || opp.end == start))
		return;
	OP_ASSERT(start.pt != end.pt);
	OP_ASSERT(opp.start.pt != opp.end.pt);
	OpSegment* _segment = const_cast<OpSegment*>(segment);
	OpSegment* oppSegment = const_cast<OpSegment*>(opp.segment);
	OpIntersection* segSect = nullptr;
	OpIntersection* oppSect = nullptr;
	if (start.pt == opp.start.pt) {
		segSect = _segment->addIntersection(start  
				OP_DEBUG_PARAMS(SECT_MAKER(addMatchingStart),
				SectReason::sharedEdgePoint, nullptr, this, &opp));
		oppSect = oppSegment->addIntersection(opp.start  
				OP_DEBUG_PARAMS(SECT_MAKER(addMatchingStartOpp),
				SectReason::sharedEdgePoint, nullptr, &opp, this));
	} else if (start.pt == opp.end.pt) {
		segSect = _segment->addIntersection(start  
				OP_DEBUG_PARAMS(SECT_MAKER(addMatchingStartOEnd),
				SectReason::sharedEdgePoint, nullptr, this, &opp));
		oppSect = oppSegment->addIntersection(opp.end  
				OP_DEBUG_PARAMS(SECT_MAKER(addMatchingStartOEndOpp),
				SectReason::sharedEdgePoint, nullptr, &opp, this));
	}
	if (segSect) {
		segSect->pair(oppSect);
		segSect = nullptr;
	}
	if (end.pt == opp.start.pt) {
		segSect = _segment->addIntersection(end  
				OP_DEBUG_PARAMS(SECT_MAKER(addMatchingEndOStart),
				SectReason::sharedEdgePoint, nullptr, this, &opp));
		oppSect = oppSegment->addIntersection(opp.start  
				OP_DEBUG_PARAMS(SECT_MAKER(addMatchingEndOStartOpp),
				SectReason::sharedEdgePoint, nullptr, &opp, this));
	} else if (end.pt == opp.end.pt) {
		segSect = _segment->addIntersection(end  
				OP_DEBUG_PARAMS(SECT_MAKER(addMatchingEnd),
				SectReason::sharedEdgePoint, nullptr, this, &opp));
		oppSect = oppSegment->addIntersection(opp.end   
				OP_DEBUG_PARAMS(SECT_MAKER(addMatchingEndOpp),
				SectReason::sharedEdgePoint, nullptr, &opp, this));
	}
	if (segSect)
		segSect->pair(oppSect);
}
#endif

/* table of winding states that the op types use to keep an edge
	left op (first path)	right op (second path)		keep if:
			0					0					---
			0					flipOff				union, rdiff, xor
			0					flipOn				union, rdiff, xor
			0					1					---
		    flipOff				0					union, diff, xor
		    flipOff				flipOff				intersect, union
		    flipOff				flipOn				diff, rdiff
		    flipOff				1					intersect, rdiff, xor
		    flipOn				0					union, diff, xor
		    flipOn				flipOff				diff, rdiff
		    flipOn				flipOn				intersect, union
		    flipOn				1					intersect, rdiff, xor
			1					0					---
			1					flipOff				intersect, diff, xor
			1					flipOn				intersect, diff, xor
			1					1					---
*/
void OpEdge::apply() {
	if (!winding.visible())
		return;
	if (many.isSet()) {
		OP_ASSERT(ZeroReason::uninitialized == many.debugReason);
		std::swap(many, sum);
		OP_DEBUG_CODE(many.debugReason = ZeroReason::swapped);
	}
	OpContours* contours = segment->contour->contours;
	WindState leftState = contours->windState(winding.left(), sum.left(), OpOperand::left);
	WindState rightState = contours->windState(winding.right(), sum.right(), OpOperand::right);
	if (leftState != WindState::flipOff && leftState != WindState::flipOn
			&& rightState != WindState::flipOff && rightState != WindState::flipOn) {
		winding.zero(ZeroReason::noFlip);
		return;
	}
	bool bothFlipped = (leftState == WindState::flipOff || leftState == WindState::flipOn)
			&& (rightState == WindState::flipOff || rightState == WindState::flipOn);
	bool keep = false;
	switch (contours->_operator) {
	case OpOperator::Subtract:
		keep = bothFlipped ? leftState != rightState :
				WindState::one == leftState || WindState::zero == rightState;
		if (keep)
			windZero = sum.right() || 0 == sum.left() ? WindZero::normal : WindZero::opp;
		break;
	case OpOperator::Intersect:
		keep = bothFlipped ? leftState == rightState :
				WindState::zero != leftState && WindState::zero != rightState;
		if (keep)
			windZero = !sum.left() || !sum.right() ? WindZero::normal : WindZero::opp;
		break;
	case OpOperator::Union:
		keep = bothFlipped ? leftState == rightState :
				WindState::one != leftState && WindState::one != rightState;
		if (keep)
			windZero = WindZero::opp;
		break;
	case OpOperator::ExclusiveOr:
		keep = !bothFlipped;
		if (keep)
			windZero = (0 == sum.left()) == (0 == sum.right()) ? WindZero::normal : WindZero::opp;
		break;
	case OpOperator::ReverseSubtract:
		keep = bothFlipped ? leftState == rightState : 
				WindState::zero == leftState || WindState::one == rightState;
		if (keep)
			windZero = sum.left() || 0 == sum.right() ? WindZero::normal : WindZero::opp;
		break;
	default:
		OP_ASSERT(0);
	}
	if (!keep)
		winding.zero(ZeroReason::applyOp);
}

// for center t (and center t only), use edge geometry since center is on edge even if there is error
// and using segment instead might return more than one intersection
void OpEdge::calcCenterT() {
	const OpRect& r = ptBounds;
	if (setLinear()) {
		center.t = OpMath::Interp(start.t, end.t, .5);
		center.pt = r.center();
	} else {
		Axis axis = r.width() >= r.height() ? Axis::vertical : Axis::horizontal;
		float middle = (r.ltChoice(axis) + r.rbChoice(axis)) / 2;
		const OpCurve& curve = setCurve();
		float t = curve.center(axis, middle);
		if (OpMath::IsNaN(t)) {
#if OP_DEBUG
			curve.center(axis, middle);	// here to allow tracing through fail case
#endif
			winding.zero(ZeroReason::centerNaN);
			fail = EdgeFail::center;
			return;
		}
		center.t = OpMath::Interp(start.t, end.t, t);
		center.pt = curve.ptAtT(t);
		center.pt.pin(ptBounds);
	}
	OP_ASSERT(OpMath::Between(ptBounds.left, center.pt.x, ptBounds.right));
	OP_ASSERT(OpMath::Between(ptBounds.top, center.pt.y, ptBounds.bottom));
}

#if 0
CalcFail OpEdge::calcPrior(Axis axis, float priorSumT, OpWinding* prev) {
	if (EdgeFail::none != fail && EdgeFail::priorDistance != fail)
		return CalcFail::none;
	// have winding of previous edge
	*prev = sum;
	OP_ASSERT(prev->left() != OpMax);
	OP_ASSERT(prev->right() != OpMax);
	OP_ASSERT(priorSumT);  // should have been initialized to some value off end of curve
	NormalDirection nd = normalDirection(-axis, priorSumT);
	if (NormalDirection::overflow == nd || NormalDirection::underflow == nd)
		OP_DEBUG_FAIL(*this, CalcFail::fail);
	if (NormalDirection::upRight == nd)
		*prev -= winding;
	return CalcFail::none;
}
#endif

CalcFail OpEdge::calcWinding(Axis axis, float t) {
	OpWinding prev(WindingTemp::dummy);
//	if (priorSum())
//		priorSum()->calcPrior(axis, sumT, &prev);
	// look at direction of edge relative to ray and figure winding/oppWinding contribution
	if (CalcFail::fail == addIfUR(axis, t, &prev))
		OP_DEBUG_FAIL(*this, CalcFail::fail);
	OP_EDGE_SET_SUM(this, prev);
	return CalcFail::none;
}

// function so that setting breakpoints is easier
// !!! if this is not inlined in release, do something more cleverer...
void OpEdge::clearActive() {
	OpDebugBreak(this, 454);
	active_impl = false;
}

void OpEdge::clearNextEdge() {
	nextEdge = nullptr;
	nextLink = EdgeLink::unlinked;
}

void OpEdge::clearPriorEdge() {
	priorEdge = nullptr;
	priorLink = EdgeLink::unlinked;
}

void OpEdge::complete() {
	winding.setWind(segment->winding.left(), segment->winding.right());
	subDivide();	// uses already computed points stored in edge
	id = segment->contour->contours->id++;
}

bool OpEdge::containsLink(const OpEdge* edge) const {
	const OpEdge* chain = this;
	std::vector<const OpEdge*> seen;
	for (;;) {
		if (edge == chain)
			return true;
		seen.push_back(chain);
		chain = /* EdgeLoop::link == loopType ? */ chain->nextEdge /* : chain->priorSum_impl */;
		if (!chain)
			break;
		auto seenIter = std::find(seen.begin(), seen.end(), chain);
		if (seen.end() != seenIter)
			break;
	}
	return false;
}

float OpEdge::findT(Axis axis, float oppXY) const {
	float result;
    FoundPtT foundPtT = segment->findPtT(axis, start.t, end.t, oppXY, &result);
	OP_ASSERT(FoundPtT::single == foundPtT);
	OP_ASSERT(OpNaN != result);
	return result;
}

#if 0
// for each edge: recurse until priorSum is null or sum winding has value 
// or -- sort the edges first ? the sort doesn't seem easy or obvious -- may need to think about it
// if horizontal axis, look at rect top/bottom
ResolveWinding OpEdge::findWinding(Axis axis, float t  OP_DEBUG_PARAMS(int* debugWindingLimiter)) {
	OP_ASSERT(++(*debugWindingLimiter) < 100);
	OP_ASSERT(!sum.isSet());	// second pass or uninitialized
	if (priorSum_impl) {
		if (!priorSum_impl->sum.isSet()) {
			ResolveWinding priorWinding = priorSum_impl->findWinding(axis, sumT
					OP_DEBUG_PARAMS(debugWindingLimiter));
			if (ResolveWinding::resolved != priorWinding)
				return priorWinding;
		}
	}
	CalcFail calcFail = calcWinding(axis, t);
	if (CalcFail::none != calcFail)
		return ResolveWinding::fail;
	return ResolveWinding::resolved;
}
#endif

bool OpEdge::inLinkLoop(const OpEdge* match) {
	OpEdge* loopy = this;
	while ((loopy = loopy->priorEdge) != this) {
		if (!loopy)
			break;
		if (match == loopy)
			return true;
	}
	while ((loopy = loopy->nextEdge) != this) {
		if (!loopy)
			return false;
		if (match == loopy)
			return true;
	}
	return false;
}

#if 0
bool OpEdge::inSumLoop(const OpEdge* match) {
	OP_ASSERT(EdgeSum::loop == sumType);
	OpEdge* loopy = this;
	OP_DEBUG_CODE(OpEdge* last = this);
	while ((loopy = loopy->priorSum_impl) != this) {
		if (!loopy) {
			OP_ASSERT(last->loopStart);
			break;
		}
		if (match == loopy)
			return true;
		OP_DEBUG_CODE(last = loopy);
	}
	return false;
}
#endif

// note that caller clears active flag if loop is closed
bool OpEdge::isClosed(OpEdge* test) {
	if (isLoop(WhichLoop::prior, LeadingLoop::will) 
			|| isLoop(WhichLoop::next, LeadingLoop::will))
		return true;
	if (flipPtT(EdgeMatch::start).pt == test->whichPtT().pt) {
		OP_ASSERT(!nextEdge || nextEdge == test);
		setNextEdge(test);
		OP_ASSERT(!test->priorEdge || test->priorEdge == this);
		test->setPriorEdge(this);
		winding.zero(ZeroReason::looped);
		test->winding.zero(ZeroReason::looped);
		return true;
	}
	return false;
}

// keep this in sync with op edge : debug dump chain
// ignore axis changes when detecting sum loops (for now)
// !!! if the axis change is required to detect for sum loops, document why!
const OpEdge* OpEdge::isLoop(WhichLoop which, LeadingLoop leading) const {
	if (!(WhichLoop::prior == which ? priorChain() : nextChain()))
		return nullptr;
	const OpEdge* chain = this;
	std::vector<const OpEdge*> seen;
	for (;;) {
		seen.push_back(chain);
		chain = WhichLoop::prior == which ? chain->priorChain() : chain->nextChain();
		if (!chain)
			break;
		auto seenIter = std::find(seen.begin(), seen.end(), chain);
		if (seen.end() == seenIter)
			continue;
		if (LeadingLoop::will == leading)
			return this;
		OP_ASSERT(LeadingLoop::in == leading);
		return chain;
	}
	return nullptr;
}

void OpEdge::linkNextPrior(OpEdge* first, OpEdge* last) {
    last->setNextEdge(this);
    first->setPriorEdge(this);
    setNextEdge(first);
    setPriorEdge(last);
    whichEnd = whichPtT(EdgeMatch::start).pt == end.pt ? EdgeMatch::start : EdgeMatch::end;

}

/* relationship between edge: (start, end) and whichEnd: (start, end)
   prev: (?, a) which:start		this: (a, b) which:start		next: (b, ?) which: start
   prev: (a, ?) which:end		this: (a, b) which:start		next: (b, ?) which: start
   prev: (?, b) which:start		this: (a, b) which:end		    next: (a, ?) which: start
   prev: (b, ?) which:end		this: (a, b) which:end  		next: (a, ?) which: start
   prev: (?, a) which:start		this: (a, b) which:start		next: (?, b) which: end		etc...
*/
// caller clears active flag
OpEdge* OpEdge::linkUp(EdgeMatch match, OpEdge* firstEdge) {
	std::vector<FoundEdge> edges;
	segment->activeAtT(this, match, edges, AllowReversal::no);
	if (edges.size() > 1)
		(EdgeMatch::start == match ? priorLink : nextLink) = EdgeLink::multiple;
	if (1 != edges.size())
		return this;
	FoundEdge found = edges.front();
	OpEdge* oppEdge = found.edge;
	OP_ASSERT(!oppEdge->hasLinkTo(match));
	OP_ASSERT(oppEdge != this);
	if (EdgeMatch::start == match) {
		OP_ASSERT(!priorEdge);
		setPriorEdge(oppEdge);
		priorLink = EdgeLink::single;
		oppEdge->setNextEdge(this);
		oppEdge->nextLink = EdgeLink::single;
		oppEdge->whichEnd = Opposite(found.whichEnd);
	} else {
		OP_ASSERT(!nextEdge);
		setNextEdge(oppEdge);
		nextLink = EdgeLink::single;
		oppEdge->setPriorEdge(this);
		oppEdge->priorLink = EdgeLink::single;
		oppEdge->whichEnd = found.whichEnd;
	}
	OP_ASSERT(whichPtT(match).pt == oppEdge->flipPtT(match).pt);
	// iterate starting with first edge to see if  opp pt forms loop
	// if so, detach remaining chain and close loop
	OpPoint endPt = oppEdge->whichPtT(match).pt;
	OpEdge* test = firstEdge;
	do {
		if (endPt == test->whichPtT(Opposite(match)).pt) {
			OpEdge* detach;
			if (EdgeMatch::start == match) {
				detach = test->nextEdge;
				if (detach)
					detach->clearPriorEdge();
				test->setNextEdge(oppEdge);
				oppEdge->setPriorEdge(test);
			} else {
				detach = test->priorEdge;
				if (detach)
					detach->clearNextEdge();
				test->setPriorEdge(oppEdge);
				oppEdge->setNextEdge(test);
			}
			return this;
		}
		test = EdgeMatch::start == match ? test->priorEdge : test->nextEdge;
	} while (test != oppEdge);

	return oppEdge->linkUp(match, firstEdge);
}

bool OpEdge::matchLink(std::vector<OpEdge*>& linkups, std::vector<OpEdge*>& unsectInX) {
	OP_ASSERT(lastEdge);
	OP_ASSERT(EdgeMatch::start == lastEdge->whichEnd || EdgeMatch::end == lastEdge->whichEnd);
	(void) setLinkBounds();
	// count intersections equaling end
	// each intersection has zero, one, or two active edges
	std::vector<FoundEdge> found;
	lastEdge->segment->activeAtT(lastEdge, EdgeMatch::end, found, AllowReversal::yes);
	OpEdge* closest = nullptr;
	EdgeMatch closestEnd = EdgeMatch::none;
	AllowReversal closestReverse = AllowReversal::no;
	if (!found.size()) {
		OpPoint matchPt = lastEdge->whichPtT(EdgeMatch::end).pt;
		float closestDistance = OpInfinity;
		for (size_t index = 0; index < linkups.size(); ++index) {
			OpEdge* edge = linkups[index];
			if (this == edge)
				continue;
			if (!edge->priorEdge) {
				float distance = (edge->whichPtT().pt - matchPt).length();
				if (closestDistance > distance) {
					found.clear();
					closestDistance = distance;
					closest = edge;
					closestEnd = edge->whichEnd;
					closestReverse = AllowReversal::no;
				} else if (closestDistance == distance)
					found.emplace_back(edge, edge->whichEnd, AllowReversal::no);	// keep ties
			}
			if (edge->lastEdge && !edge->lastEdge->nextEdge) {
				OP_ASSERT(!edge->lastEdge->nextEdge);
				float distance = (edge->lastEdge->whichPtT(EdgeMatch::end).pt - matchPt).length();
				if (closestDistance > distance) {
					found.clear();
					closestDistance = distance;
					closest = edge->lastEdge;
					closestEnd = Opposite(edge->lastEdge->whichEnd);
					closestReverse = AllowReversal::yes;
				} else if (closestDistance == distance)
					found.emplace_back(edge->lastEdge, Opposite(edge->lastEdge->whichEnd),
							AllowReversal::yes);	// keep ties
			}
		}
#if OP_DEBUG
		if (!closest) {
			focus(429);
			oo(10);
			showPoints();
			showHex();
			showIntersections();
			OpDebugOut("");
		}
#endif
		OP_ASSERT(closest);
		found.emplace_back(closest, closestEnd, closestReverse);
	}
	OP_ASSERT(found.size());
	OpRect bestBounds;
	closest = nullptr;
	closestEnd = EdgeMatch::none;
	closestReverse = AllowReversal::no;
	for (int trial = 0; !closest && trial < 2; ++trial) {
		for (const auto& foundOne : found) {
			OpEdge* oppEdge = foundOne.edge;
			// skip edges which flip the fill sum total, implying there is a third edge inbetween
			OP_ASSERT(EdgeMatch::start == foundOne.whichEnd || EdgeMatch::end == foundOne.whichEnd);
			               // e.g., one if normals point to different fill sums       
			if (!trial && (((lastEdge->sum.sum() + oppEdge->sum.sum()) & 1) 
			// e.g., one if last start-to-end connects to found start-to-end (end connects to start)
					== (lastEdge->whichEnd == foundOne.whichEnd)))
				continue;
			// choose smallest closed loop -- this is an arbitrary choice
			OpPointBounds testBounds = linkBounds;
			testBounds.add(oppEdge->setLinkBounds());
			if (!(bestBounds.area() < testBounds.area())) {	// 'not' logic since best = NaN at first
				bestBounds = testBounds;
				closest = oppEdge;
				closestEnd = foundOne.whichEnd;
				closestReverse = foundOne.reverse;
			}
		}
	}
	OP_ASSERT(closest); // !!! if found is not empty, but no edge has the right sum, choose one anyway?
	OP_ASSERT(closest->lastEdge);
	if (AllowReversal::yes == closestReverse)
		closest->setLinkDirection(closestEnd);
	OpEdge* clearClosest = closest;
	do {
		clearClosest->clearActive();
		clearClosest = clearClosest->nextEdge;
	} while (clearClosest && clearClosest->isActive());
	closest->setPriorEdge(lastEdge);
	closest->priorLink = EdgeLink::single;
	lastEdge->setNextEdge(closest);
	lastEdge->nextLink = EdgeLink::single;
	lastEdge = closest->lastEdge;
	closest->lastEdge = nullptr;
 	OpDebugPlayback(this, 411);
	showNormals();
	OP_ASSERT(lastEdge);
	if (lastEdge->isClosed(this) || lastEdge->segment->contour->contours
			->closeGap(lastEdge, this, unsectInX))
		return lastEdge->validLoop();
	if (!lastEdge->nextEdge)
		return matchLink(linkups, unsectInX);
	return lastEdge->matchLink(linkups, unsectInX);
}

NormalDirection OpEdge::normalDirection(Axis axis, float t) {
	const OpCurve& curve = setCurve();
	return curve.normalDirection(axis, t);
}

OpEdge* OpEdge::prepareForLinkup() {
    OpEdge* first = this;
	while (EdgeLink::multiple != first->priorLink) {
		OpEdge* prior = first->priorEdge;
		if (!prior)
			break;
		OP_ASSERT(this != prior);
		first = prior;
	}
	OpEdge* next = first;
	OpEdge* last;
	do {
		OP_ASSERT(next->isActive());
		next->clearActive();
		last = next;
		if (EdgeLink::multiple == next->nextLink)
			break;
		next = next->nextEdge;
	} while (next);
    if (!first->lastEdge)
		first->lastEdge = last;
    return first;
}

// in function to make setting breakpoints easier
// !!! if this is not inlined in release, do something more cleverer
void OpEdge::setActive() {
	active_impl = true;
}

const OpCurve& OpEdge::setCurve() {
	if (curveSet)
		return curve_impl;
	curveSet = true;
	curve_impl.pts[0] = start.pt;
	unsigned index = 0;
	unsigned ptCount = segment->c.pointCount();
	while (++index < ptCount - 1)
		curve_impl.pts[index] = ctrlPts[index - 1];
	if (1 == ptCount)
		--index;
	curve_impl.pts[index] = end.pt;
	OP_ASSERT(++index == ptCount);
	curve_impl.weight = weight;
	curve_impl.type = segment->c.type;
#if OP_DEBUG
	curve_impl.debugIntersect = OpDebugIntersect::edge;
#endif
	return curve_impl;
}

void OpEdge::setFromPoints(const std::array<OpPoint, 4>& pts) {
	start.pt = pts[0];
	unsigned index = 0;
	unsigned ptCount = segment->c.pointCount();
	while (++index < ptCount - 1)
		ctrlPts[index - 1] = pts[index];
	if (1 == ptCount)
		--index;
	end.pt = pts[index];
	OP_ASSERT(++index == ptCount);
}

// this compares against float epsilon instead of zero
// when comparing against a line, an edge close to zero can fall into denormalized numbers,
//   causing the calling subdivision to continue for way too long. Using epsilon as a stopgap
//   avoids this. The alternative would be to change the math to disallow denormalized numbers
bool OpEdge::setLinear() {
	if (lineSet)
		return isLine_impl;
	lineSet = true;
	const OpCurve& rotated = setVertical();
	for (int index = 1; index < rotated.pointCount() - 1; ++index) {
		if (fabsf(rotated.pts[index].x) > OpEpsilon)
			return (isLine_impl = false);
	}
	return (isLine_impl = true);
}

OpPointBounds OpEdge::setLinkBounds() {
	if (!lastEdge) {
		OpEdge* first = this;
		OP_ASSERT(priorEdge);
		while (first->priorEdge)
			first = first->priorEdge;
		OP_ASSERT(first->lastEdge);
		return first->setLinkBounds();
	}
	if (linkBounds.isSet())
		return linkBounds;
	linkBounds = ptBounds;
	const OpEdge* edge = this;
	while (edge != lastEdge) {
		edge = edge->nextEdge;
		linkBounds.add(edge->ptBounds);
	}
	return linkBounds;
}

// reverses link walk
// if the edge chain is one long, match is required to know if it must be reversed
void OpEdge::setLinkDirection(EdgeMatch match) {
	if (EdgeMatch::start == match && lastEdge)
		return;
	OpEdge* edge = this;
	while (edge->priorEdge) {
		std::swap(edge->priorEdge, edge->nextEdge);
		std::swap(edge->priorLink, edge->nextLink);
		edge->whichEnd = Opposite(edge->whichEnd);
		edge = edge->nextEdge;
	}
	std::swap(edge->priorEdge, edge->nextEdge);
	std::swap(edge->priorLink, edge->nextLink);
	edge->whichEnd = Opposite(edge->whichEnd);
	lastEdge = edge;
	if (this != edge) {
//		OP_ASSERT(!inLinkLoop(edge));		// !!! don't know what this OP_ASSERT is testing for
		edge->lastEdge = nullptr;
	}
}

// setter to make adding breakpoints easier
// !!! if release doesn't inline, restructure more cleverly...
void OpEdge::setNextEdge(OpEdge* edge) {
	nextEdge = edge;
}

void OpEdge::setPointBounds() {
	ptBounds.set(start.pt, end.pt);
#if OP_DEBUG
	OpPointBounds copy = ptBounds;
	for (int index = 0; index < segment->c.pointCount() - 2; ++index)
		ptBounds.add(ctrlPts[index]);
	OP_ASSERT(copy == ptBounds);
#endif
}

void OpEdge::setPoints(std::array<OpPoint, 4>& pts) const {
	pts[0] = start.pt;
	unsigned index = 0;
	unsigned ptCount = segment->c.pointCount();
	while (++index < ptCount - 1)
		pts[index] = ctrlPts[index - 1];
	if (1 == ptCount)
		--index;
	pts[index] = end.pt;
	OP_ASSERT(++index == ptCount);
}

// setter to make adding breakpoints easier
// !!! if release doesn't inline, restructure more cleverly...
void OpEdge::setPriorEdge(OpEdge* edge) {
	priorEdge = edge;
}

#if 0
// setter to make adding breakpoints easier
// !!! if release doesn't inline, restructure more cleverly...
void OpEdge::setPriorSum(OpEdge* edge) {
	priorSum_impl = edge;
}
#endif

const OpCurve& OpEdge::setVertical() {
	if (!verticalSet) {
		verticalSet = true;
		const OpCurve& curve = setCurve();
		LinePts edgeLine { start.pt, end.pt };
		vertical_impl = curve.toVertical(edgeLine);
	}
	return vertical_impl;
}

#if 0	// !!! happy to comment out, because it doesn't make sense...
// if edge normal points towards ray, set sumWinding to 0
void OpEdge::setWinding(OpVector ray) {
	OpVector normal = segment->c.normal(center.t);
	if (OpOperand::left == segment->contour->operand)
		sum.left = ray.dot(normal) >= 0;
	else
		sum.right = ray.dot(normal) >= 0;
#if OP_DEBUG
	sum.debugReason = ZeroReason::rayNormal;
#endif
}
#endif

// use already computed points stored in edge
void OpEdge::subDivide() {
	CurvePts pts = segment->c.subDivide(start, end);
	weight = pts.weight;
	setFromPoints(pts.pts);
	setPointBounds();
	if (OpType::line == segment->c.type || (!ptBounds.height() ^ !ptBounds.width())) {
		isLine_impl = true;
		lineSet = true;
	}
 	if (start.pt == end.pt) {
		OP_ASSERT(0);	// !!! check to see if this can still happen
		sumType = EdgeSum::point;
		winding.zero(ZeroReason::isPoint);
	}
	calcCenterT();
}

CalcFail OpEdge::subIfDL(Axis axis, float t, OpWinding* sumWinding) {
	NormalDirection NdotR = normalDirection(axis, t);
	if (NormalDirection::downLeft == NdotR)
		*sumWinding -= winding;
	else if (NormalDirection::upRight != NdotR)
		OP_DEBUG_FAIL(*this, CalcFail::fail);
	return CalcFail::none;
}

// note this should be only called on temporary edges (e.g., used to make coincident intersections)
// after this is called, edge does not agree with segment, but does match another edge
// this has to be called a second time (e.g., after trimming) to agree with segment
void OpEdge::reverse() {
	std::swap(start, end);
	if (OpType::cubic == segment->c.type)
		std::swap(ctrlPts[0], ctrlPts[1]);
	start.t = 1 - start.t;
	end.t = 1 - end.t;
}

// fuzz-generated test crbug_526025 generates an edge link that is invalid. Worth chasing down 
// someday, but today, just return failure since the fuzz test won't succeed
bool OpEdge::validLoop() const {
	std::vector<const OpEdge*> seen;
	const OpEdge* last = this;
	for (;;) {
		if (seen.end() != std::find(seen.begin(), seen.end(), last)) {
			return true;
		}
		seen.push_back(last);
		if (!last->nextEdge)
			return true;
		if (last != last->nextEdge->priorEdge)
			return false;
		last = last->nextEdge;
	}
}

#if 0
OpEdge* OpEdge::visibleAdjacent(EdgeMatch match) {
	return (const_cast<OpSegment*>(segment))->visibleAdjacent(this, ptT(match));
}
#endif

bool OpEdge::debugFail() const {
#if OP_DEBUG
    return segment->debugFail();
#endif
    return false;
}

bool OpEdge::debugSuccess() const {
#if OP_DEBUG
    return segment->debugSuccess();
#endif
    return true;
}

// !!! add zero reason here
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
#if OP_DEBUG
	debugReason = 0 == left_impl && 0 == right_impl ? ZeroReason::move :
			ZeroReason::uninitialized;
#endif
}

void OpWinding::setSum(OpWinding winding, const OpSegment* segment) {
	OP_ASSERT(WindingType::uninitialized == debugType);
	OP_ASSERT(WindingType::temp == winding.debugType);
	OP_DEBUG_CODE(debugType = WindingType::sum);
	const OpContours& contours = *segment->contour->contours;
	left_impl = winding.left() & contours.leftFillTypeMask();
	right_impl = winding.right() & contours.rightFillTypeMask();
}
