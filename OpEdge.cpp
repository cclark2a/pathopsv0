#include "OpEdge.h"
#include "OpContour.h"

OpEdge::OpEdge(const OpEdge* edge, const OpPtT& newPtT, NewEdge isLeftRight  
		OP_DEBUG_PARAMS(EdgeMaker maker, int line, std::string file, const OpIntersection* i1, 
		const OpIntersection* i2))
	: OpEdge() {
	segment = edge->segment;
	start = NewEdge::isLeft == isLeftRight ? edge->start : newPtT;
	end = NewEdge::isLeft == isLeftRight ? newPtT : edge->end;
#if OP_DEBUG
	debugStart = i1;
	debugEnd = i2;
	debugMaker = maker;
	debugMakerLine = line;
	debugMakerFile = file;
	debugParentID = edge->id;
#endif	
	complete();
}

// called after winding has been modified by other coincident edges
OpEdge::OpEdge(const OpEdge* edge, const OpPtT& s, const OpPtT& e  
		OP_DEBUG_PARAMS(EdgeMaker maker, int line, std::string file, const OpIntersection* i1,
		const OpIntersection* i2))
	: OpEdge() {
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
	id = segment->contour->contours->id++;
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
	if (!winding.visible() || EdgeSum::unsortable == sumType)
		return;
	if (many.isSet())
		std::swap(many, sum);
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
bool OpEdge::calcCenterT() {
	const OpRect& r = ptBounds;
	Axis axis = r.width() >= r.height() ? Axis::vertical : Axis::horizontal;
	float middle = (r.ltChoice(axis) + r.rbChoice(axis)) / 2;
	const OpCurve& curve = setCurve();
	float t = curve.center(axis, middle);
	if (OpMath::IsNaN(t)) {
		winding.zero(ZeroReason::centerNaN);
		fail = EdgeFail::center;
		return true;
	}
	center.t = OpMath::Interp(start.t, end.t, t);
	center.pt = curve.ptAtT(t);
	center.pt.pin(ptBounds);
	OP_ASSERT(OpMath::Between(ptBounds.left, center.pt.x, ptBounds.right));
	OP_ASSERT(OpMath::Between(ptBounds.top, center.pt.y, ptBounds.bottom));
	return start.t < center.t && center.t < end.t;
}

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

void OpEdge::clearActiveAndPals() {
	setActive(false);
    for (auto pal : pals) {
        pal->setActive(false); 
    }
}

void OpEdge::clearNextEdge() {
	setNextEdge(nullptr);
	nextLink = EdgeLink::unlinked;
}

void OpEdge::clearPriorEdge() {
	setPriorEdge(nullptr);
	priorLink = EdgeLink::unlinked;
}

void OpEdge::complete() {
	id = segment->contour->contours->id++;
	winding.setWind(segment->winding.left(), segment->winding.right());
	subDivide();	// uses already computed points stored in edge
}

// !!! either: implement 'stamp' that puts a unique # on the edge to track if it has been visited;
// or, re-walk the chain from this (where the find is now) to see if chain has been seen
bool OpEdge::containsLink(const OpEdge* edge) const {
	const OpEdge* chain = this;
	std::vector<const OpEdge*> seen;
	for (;;) {
		if (edge == chain)
			return true;
		seen.push_back(chain);		
		if (chain = chain->nextEdge; !chain)
			break;	
		if (auto seenIter = std::find(seen.begin(), seen.end(), chain); seen.end() != seenIter)
			break;
	}
	return false;
}

struct LoopCheck {
	LoopCheck(OpEdge* e) 
		: edge(e) {
		pt = e->ptT(e->whichEnd).pt;
	}

	bool operator<(const LoopCheck& rh) const {
		return pt.x < rh.pt.x || (pt.x == rh.pt.x && pt.y < rh.pt.y);
	}

	OpEdge* edge;
	OpPoint pt;
};


// iterate edges to see some pt forms a loop
// if so, detach remaining chain and close loop
// at this point, 'this' should not form a loop; (caller asserts if edge is a loop)
// check if any points in next links are in previous links
bool OpEdge::detachIfLoop() {
	std::vector<LoopCheck> edges;
	OpEdge* test = this;
	// walk forwards to end, keeping one point per edge
	while (test) {
		edges.emplace_back(test);
		test = test->nextEdge;
	}
	// walk backwards to start
	std::sort(edges.begin(), edges.end());
	auto detachLoop = [](OpEdge* test, OpEdge* oppEdge) {
		if (OpEdge* detach = test->nextEdge)
			detach->clearPriorEdge();
		test->setNextEdge(oppEdge);
		oppEdge->setPriorEdge(test);
		return true;
	};
	test = this;
	while ((test = test->priorEdge)) {
		LoopCheck testCheck(test);
		if (auto bound = std::lower_bound(edges.begin(), edges.end(), testCheck); 
				bound != edges.end() && bound->pt == testCheck.pt)
			return detachLoop(bound->edge, test);
		for (auto pal : test->pals) {
			LoopCheck palCheck(pal);
			if (auto bound = std::lower_bound(edges.begin(), edges.end(), pal);
					bound != edges.end() && !(LoopCheck(pal) < edges.front())) {
				OP_ASSERT(0);
				// If opp edge does not from a loop but pal does,
				// probably need to replace opp with pal.
				// Wait for this to happen for realsies before writing code to handle it.
				return detachLoop(bound->edge, test);
			}
		}
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

// note that caller clears active flag if loop is closed
bool OpEdge::isClosed(OpEdge* test) {
	if (isLoop(WhichLoop::prior, LeadingLoop::will) 
			|| isLoop(WhichLoop::next, LeadingLoop::will))
		return true;
	if (flipPtT(EdgeMatch::start).pt != test->whichPtT().pt)
		return false;
	OP_ASSERT(!nextEdge || nextEdge == test);
	setNextEdge(test);
	OP_ASSERT(!test->priorEdge || test->priorEdge == this);
	test->setPriorEdge(this);
	winding.zero(ZeroReason::looped);
	test->winding.zero(ZeroReason::looped);
	return true;
}

// keep this in sync with op edge : debug dump chain
// ignore axis changes when detecting sum loops (for now)
// !!! if the axis change is required to detect for sum loops, document why!
// !!! either add 'stamp' or rewalk links instead of find
const OpEdge* OpEdge::isLoop(WhichLoop which, LeadingLoop leading) const {
	if (!(WhichLoop::prior == which ? priorEdge : nextEdge))
		return nullptr;
	const OpEdge* chain = this;
	std::vector<const OpEdge*> seen;
	for (;;) {
		seen.push_back(chain);
		chain = WhichLoop::prior == which ? chain->priorEdge : chain->nextEdge;
		if (!chain)
			break;	
		if (seen.end() == std::find(seen.begin(), seen.end(), chain))
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

void OpEdge::linkToEdge(FoundEdge& found, EdgeMatch match) {
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
}

/* relationship between prev/this/next and whichEnd: (start, end)
   a and b represent points that match; ? represents the other nonmatching end
   prev: (?, a) which:start		this: (a, b) which:start		next: (b, ?) which: start
   prev: (a, ?) which:end		this: (a, b) which:start		next: (b, ?) which: start
   prev: (?, b) which:start		this: (a, b) which:end		    next: (a, ?) which: start
   prev: (b, ?) which:end		this: (a, b) which:end  		next: (a, ?) which: start
   prev: (?, a) which:start		this: (a, b) which:start		next: (?, b) which: end		etc...
*/
// caller clears active flag
// parameter match determines whether link is looked for prior to, or next to, 'this'
OpEdge* OpEdge::linkUp(EdgeMatch match, std::vector<OpEdge*>* unsectInX) {
	OP_ASSERT(!isLoop(WhichLoop::next, LeadingLoop::will));
	std::vector<FoundEdge> edges;
	if (unsectInX)
		matchUnsectable(match, unsectInX, edges);
	else {
		bool hasPal = segment->activeAtT(this, match, edges, AllowReversal::no);
		hasPal |= segment->activeNeighbor(this, match, edges);
		if (hasPal)
			skipPals(match, edges);
	}
	if (edges.size() > 1)
		(EdgeMatch::start == match ? priorLink : nextLink) = EdgeLink::multiple;
	if (1 != edges.size())
		return this;
	FoundEdge found = edges.front();
	OP_ASSERT(!found.edge->isLoop(WhichLoop::next, LeadingLoop::will));
	OP_ASSERT(!found.edge->isLoop(WhichLoop::prior, LeadingLoop::will));
	linkToEdge(found, match);
	OP_ASSERT(whichPtT(match).pt == found.edge->flipPtT(match).pt);
	if ((EdgeMatch::start == match ? this : found.edge)->detachIfLoop())
		return found.edge;
	return found.edge->linkUp(match, unsectInX);
}

bool OpEdge::matchLink(std::vector<OpEdge*>& linkups, std::vector<OpEdge*>& unsectInX) {
	OP_ASSERT(lastEdge);
	OP_ASSERT(EdgeMatch::start == lastEdge->whichEnd || EdgeMatch::end == lastEdge->whichEnd);
	// count intersections equaling end
	// each intersection has zero, one, or two active edges
	std::vector<FoundEdge> found;
	const OpSegment* last = lastEdge->segment;
	// should be able to ignore result because pals have already been marked inactive
	(void) last->activeAtT(lastEdge, EdgeMatch::end, found, AllowReversal::yes);
	(void) last->activeNeighbor(this, EdgeMatch::end, found);
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
		if (!closest) {	// look for a run of unsectables that close the gap
			OP_ASSERT(!priorEdge);
			OP_ASSERT(!lastEdge->nextEdge);
			if (OpEdge* result = linkUp(EdgeMatch::start, &unsectInX))
				return result;
		}
#if OP_DEBUG
		if (!closest) {
			focus(id);
			oo(10);
			showPoints();
			showHex();
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
			OpPointBounds testBounds = setLinkBounds();
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
	OP_ASSERT(AllowReversal::yes == closestReverse || closest->lastEdge);
	if (AllowReversal::yes == closestReverse)
		closest->setLinkDirection(closestEnd);
	OpEdge* clearClosest = closest;
	do {
		clearClosest->clearActiveAndPals();
		clearClosest = clearClosest->nextEdge;
	} while (clearClosest && clearClosest->isActive());
	closest->setPriorEdge(lastEdge);
	closest->priorLink = EdgeLink::single;
	lastEdge->setNextEdge(closest);
	lastEdge->nextLink = EdgeLink::single;
	lastEdge = closest->lastEdge;
	closest->lastEdge = nullptr;
	OP_ASSERT(lastEdge);
	if (lastEdge->isClosed(this) 
			|| last->contour->contours->closeGap(lastEdge, this, unsectInX))
		return lastEdge->validLoop();
	if (!lastEdge->nextEdge)
		return matchLink(linkups, unsectInX);
	return lastEdge->matchLink(linkups, unsectInX);
}

// keep only one unsectable from any set of pals
void OpEdge::matchUnsectable(EdgeMatch match, std::vector<OpEdge*>* unsectInX,
		std::vector<FoundEdge>& edges) {
	const OpPoint firstPt = whichPtT(match).pt;
	for (OpEdge* unsectable : *unsectInX) {
#if OP_DEBUG
		auto checkDupes = [&edges](OpEdge* test) {
			for (const auto& found : edges) {
				OP_ASSERT(found.edge != test);
				for (auto pal : found.edge->pals)
					OP_ASSERT(pal != test);
			}
		};
#endif
		auto checkEnds = [&edges, firstPt  OP_DEBUG_PARAMS(checkDupes)](OpEdge* unsectable) {
			if (unsectable->inOutput || unsectable->inOutQueue)
				return false;
			if (firstPt == unsectable->start.pt) {
				OP_DEBUG_CODE(checkDupes(unsectable));
				edges.emplace_back(unsectable, EdgeMatch::start, AllowReversal::yes);
				return true;
			}
			if (firstPt == unsectable->end.pt) {
				OP_DEBUG_CODE(checkDupes(unsectable));
				edges.emplace_back(unsectable, EdgeMatch::end, AllowReversal::yes);
				return true;
			}
			return false;
		};
		if (checkEnds(unsectable))
			continue;
		for (auto pal : unsectable->pals) {
			if (checkEnds(pal))
				break;
		}
	}
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
		next->clearActiveAndPals();
		next->inOutQueue = true;
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
void OpEdge::setActive(bool state) {
	active_impl = state;
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

void OpEdge::setFromPoints(const OpPoint pts[]) {
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
	const OpCurve& curve = setCurve();
	return (isLine_impl = curve.isLinear());
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
	if (this != edge)
		edge->lastEdge = nullptr;
}

// setter to make adding breakpoints easier
// !!! if release doesn't inline, restructure more cleverly...
void OpEdge::setNextEdge(OpEdge* edge) {
	nextEdge = edge;
}

void OpEdge::setPointBounds() {
	ptBounds.set(start.pt, end.pt);
	// this check can fail; control points can have some error so they lie just outside bounds
#if 0 // OP_DEBUG
	OpPointBounds copy = ptBounds;
	for (int index = 0; index < segment->c.pointCount() - 2; ++index)
		ptBounds.add(ctrlPts[index]);
	OP_ASSERT(copy == ptBounds);
#endif
}

// setter to make adding breakpoints easier
// !!! if release doesn't inline, restructure more cleverly...
void OpEdge::setPriorEdge(OpEdge* edge) {
	priorEdge = edge;
}

const OpCurve& OpEdge::setVertical() {
	if (!verticalSet) {
		verticalSet = true;
		const OpCurve& curve = setCurve();
		LinePts edgeLine { start.pt, end.pt };
		vertical_impl = curve.toVertical(edgeLine);
	}
	return vertical_impl;
}

// Remove duplicate pals that look at each other in found edges.
// Remove pals equal to 'this' and edges this links to.
void OpEdge::skipPals(EdgeMatch match, std::vector<FoundEdge>& edges) const {
	std::vector<FoundEdge> skipped;
	for (const auto& found : edges) {
		for (auto pal : found.edge->pals) {
			for (const auto& skip : skipped) {
				if (skip.edge == pal)
					goto skipIt;
			}
			const OpEdge* test = this;
			do {
				if (test == pal)
					goto skipIt;
			} while ((test = EdgeMatch::start == match ? test->priorEdge : test->nextEdge));
		}
		skipped.push_back(found);
skipIt:
		;
	}
	if (skipped.size() < edges.size())
		std::swap(skipped, edges);
}

// use already computed points stored in edge
void OpEdge::subDivide() {
	OpCurve pts = segment->c.subDivide(start, end);
	weight = pts.weight;
	setFromPoints(pts.pts);
	setPointBounds();
	if (OpType::line == segment->c.type || (!ptBounds.height() ^ !ptBounds.width())
			|| !calcCenterT()) {
		isLine_impl = true;
		lineSet = true;
		center.t = OpMath::Interp(start.t, end.t, .5);
		center.pt = ptBounds.center();
	}
 	if (start.pt == end.pt) {
		OP_ASSERT(0);	// !!! check to see if this can still happen
		winding.zero(ZeroReason::isPoint);
	}
}

CalcFail OpEdge::subIfDL(Axis axis, float t, OpWinding* sumWinding) {
	NormalDirection NdotR = normalDirection(axis, t);
	if (NormalDirection::downLeft == NdotR)
		*sumWinding -= winding;
	else if (NormalDirection::upRight != NdotR)
		OP_DEBUG_FAIL(*this, CalcFail::fail);
	return CalcFail::none;
}

// fuzz-generated test crbug_526025 generates an edge link that is invalid. Worth chasing down 
// someday, but today, just return failure since the fuzz test won't succeed
// !!! at best, this should become debugging-only code
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
		if (last != last->nextEdge->priorEdge) {
			OP_ASSERT(0);
			return false;
		}
		last = last->nextEdge;
	}
}

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
