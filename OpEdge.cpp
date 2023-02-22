#include "OpEdge.h"
#include "OpContour.h"

OpEdge::OpEdge(const OpEdge* e, OpPtT newPtT, NewEdge isLeftRight  OP_DEBUG_PARAMS(EdgeMaker maker))
	: OpEdge() {
	segment = e->segment;
	start = NewEdge::isLeft == isLeftRight ? e->start : newPtT;
	end = NewEdge::isLeft == isLeftRight ? newPtT : e->end;
#if OP_DEBUG
	debugMaker = maker;
	debugParentID = e->id;
#endif	
	complete();
}

void OpEdge::addMatchingEnds(const OpEdge& opp) const {
	if (opp.segment == segment && (opp.start == end || opp.end == start))
		return;
	OpSegment* _segment = const_cast<OpSegment*>(segment);
	OpSegment* oppSegment = const_cast<OpSegment*>(opp.segment);
	if (start.pt == opp.start.pt || start.pt == opp.end.pt)
		_segment->intersections.emplace_back(start, oppSegment, 0  
				OP_DEBUG_PARAMS(IntersectMaker::addMatchingEnds1));
#if OP_DEBUG
	if (90 == segment->contour->contours->id)
		OpDebugOut("");
#endif
	if (end.pt == opp.start.pt || end.pt == opp.end.pt)
		_segment->intersections.emplace_back(end, oppSegment, 0  
				OP_DEBUG_PARAMS(IntersectMaker::addMatchingEnds2));
	if (start.pt == opp.start.pt || end.pt == opp.start.pt)
		oppSegment->intersections.emplace_back(opp.start, _segment, 0  
				OP_DEBUG_PARAMS(IntersectMaker::addMatchingEnds3));
	if (start.pt == opp.end.pt || end.pt == opp.end.pt)
		oppSegment->intersections.emplace_back(opp.end, _segment, 0  
				OP_DEBUG_PARAMS(IntersectMaker::addMatchingEnds4));
}

/* table of winding states that the op types use to keep an edge
	left op (first path)	right op (second path)		keep if:
			0					0					---
dump
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
	OpContours* contours = segment->contour->contours;
	WindState leftState = contours->windState(winding.left, sum.left, OpOperand::left);
	WindState rightState = contours->windState(winding.right, sum.right, OpOperand::right);
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
			windZero = sum.right || 0 == sum.left ? WindZero::normal : WindZero::opp;
		break;
	case OpOperator::Intersect:
		keep = bothFlipped ? leftState == rightState :
				WindState::zero != leftState && WindState::zero != rightState;
		if (keep)
			windZero = !sum.left || !sum.right ? WindZero::normal : WindZero::opp;
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
			windZero = (0 == sum.left) == (0 == sum.right) ? WindZero::normal : WindZero::opp;
		break;
	case OpOperator::ReverseSubtract:
		keep = bothFlipped ? leftState == rightState : 
				WindState::zero == leftState || WindState::one == rightState;
		if (keep)
			windZero = sum.left || 0 == sum.right ? WindZero::normal : WindZero::opp;
		break;
	default:
		assert(0);
	}
	if (!keep)
		winding.zero(ZeroReason::applyOp);
}

// for center t (and center t only), use edge geometry since center is on edge even if there is error
// and using segment instead might return more than one intersection
void OpEdge::calcCenterT() {
	const OpRect& r = pointBounds;
	if (setLinear()) {
		center.t = OpMath::Interp(start.t, end.t, .5);
		center.pt = r.center();
	} else {
		OpDebugIntersectSave save(OpDebugIntersect::edge);
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
		center.pt.pin(pointBounds);
	}
	assert(OpMath::Between(pointBounds.left, center.pt.x, pointBounds.right));
	assert(OpMath::Between(pointBounds.top, center.pt.y, pointBounds.bottom));
}

void OpEdge::calcWinding(Axis axis) {
	int prevLeft = 0;
	int prevRight = 0;
	OpVector ray = Axis::horizontal == axis ? OpVector{ 1, 0 } : OpVector{ 0, 1 };
	if (priorSum && (EdgeFail::none == priorSum->fail || EdgeFail::priorDistance == priorSum->fail)) {
		assert(priorSum->sum.left != OpMax);
		assert(priorSum->sum.right != OpMax);
		// have winding of previous edge
		prevLeft = priorSum->sum.left;
		prevRight = priorSum->sum.right;
		const OpCurve& curve = priorSum->setCurve();
		assert(priorT);	// should have been initialized to some value off end of curve
		float tNdotR = curve.normal(priorT).normalize().dot(-ray);
		if (tNdotR > 0) {
			prevLeft -= priorSum->winding.left;
			prevRight -= priorSum->winding.right;
		}
	}
	// look at direction of edge relative to ray and figure winding/oppWinding contribution
	float NdotR = segment->c.normal(center.t).normalize().dot(ray);
	if (NdotR > 0) {
		prevLeft += winding.left;
		prevRight += winding.right;
	}
	sum.left = prevLeft & segment->contour->contours->leftFillTypeMask();
	sum.right = prevRight & segment->contour->contours->rightFillTypeMask();
}

// function so that setting breakpoints is easier
// !!! if this is not inlined in release, do something more cleverer...
void OpEdge::clearActive() {
	if (0) OpDebugBreak(this, 271, false);
	active = false;
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
	winding.left = segment->winding.left;
	winding.right = segment->winding.right;
	subDivide();	// uses already computed points stored in edge
	id = segment->contour->contours->id++;
}

bool OpEdge::containsLink(const OpEdge* edge) const {
	const OpEdge* test = this;
	do {
		if (edge == test)
			return true;
		test = test->nextEdge;
	} while (test && test != this);
	return false;
}

float OpEdge::findPtT(OpPoint opp) const {
	OpVector lineSize = end.pt - start.pt;
	if (fabsf(lineSize.dy) > fabsf(lineSize.dx))
		return (opp.y - start.pt.y) / lineSize.dy;
	return (opp.x - start.pt.x) / lineSize.dx;
}

// for each edge: recurse until priorSum is null or sum winding has value 
// or -- sort the edges first ? the sort doesn't seem easy or obvious -- may need to think about it
// if horizontal axis, look at rect top/bottom
void OpEdge::findWinding(Axis axis  OP_DEBUG_PARAMS(int* debugWindingLimiter)) {
	assert(++(*debugWindingLimiter) < 100);
	assert(sum.unset());	// second pass or uninitialized
	assert(!isLoop(EdgeLoop::sum));
	assert(!priorSum || !priorSum->isLoop(EdgeLoop::sum));
	if (priorSum && priorSum->sum.unset())
		priorSum->findWinding(axis  OP_DEBUG_PARAMS(debugWindingLimiter));
	calcWinding(axis);
}

// note that caller clears active flag if loop is closed
bool OpEdge::isClosed(OpEdge* test) {
	if (isLoop(EdgeLoop::link))
		return true;
	if (flipPtT(EdgeMatch::start).pt == test->whichPtT().pt) {
		assert(!nextEdge || nextEdge == test);
		setNextEdge(test);
		assert(!test->priorEdge || test->priorEdge == this);
		test->setPriorEdge(this);
		winding.zero(ZeroReason::looped);
		test->winding.zero(ZeroReason::looped);
		return true;
	}
	return false;
}

const OpEdge* OpEdge::isLoop(EdgeLoop edgeLoop) const {
	const OpEdge* chain = this;
	std::vector<const OpEdge*> seen;
	do {
		if (seen.end() != std::find(seen.begin(), seen.end(), chain)) {
			return chain;
		}
		seen.push_back(chain);
		chain = EdgeLoop::link == edgeLoop ? chain->nextEdge : chain->nextSum;
	} while (chain);
	chain = EdgeLoop::link == edgeLoop ? priorEdge : priorSum;
	while (chain) {
		if (seen.end() != std::find(seen.begin(), seen.end(), chain)) {
			return chain;
		}
		seen.push_back(chain);
		chain = EdgeLoop::link == edgeLoop ? chain->priorEdge : chain->priorSum;
	}
	return nullptr;
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
//  	OpDebugBreak(this, 267, EdgeMatch::start == match);
	segment->activeAtT(this, match, edges, AllowReversal::no);
	if (edges.size() > 1)
		(EdgeMatch::start == match ? priorLink : nextLink) = EdgeLink::multiple;
	if (1 != edges.size())
		return this;
	FoundEdge found = edges.front();
	OpEdge* oppEdge = found.edge;
	assert(!oppEdge->hasLinkTo(match));
	assert(oppEdge != this);
	if (EdgeMatch::start == match) {
		assert(!priorEdge);
		setPriorEdge(oppEdge);
		priorLink = EdgeLink::single;
		oppEdge->setNextEdge(this);
		oppEdge->nextLink = EdgeLink::single;
		oppEdge->whichEnd = Opposite(found.whichEnd);
	} else {
		assert(!nextEdge);
		setNextEdge(oppEdge);
		nextLink = EdgeLink::single;
		oppEdge->setPriorEdge(this);
		oppEdge->priorLink = EdgeLink::single;
		oppEdge->whichEnd = found.whichEnd;
	}
	assert(whichPtT(match).pt == oppEdge->flipPtT(match).pt);
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

void OpEdge::markFailNext(std::vector <OpEdge*>& edges, Axis axis) {
	// the ray hit multiple, indistinguishable edges
	OpEdge* next = this;
	float direction = start.pt.choice(axis) - end.pt.choice(axis);
	do {
		if (this != next) {
			float nextDirection = next->start.pt.choice(axis) - next->end.pt.choice(axis);
			bool reversed = direction * nextDirection < 0;
			winding.move(next->winding, segment->contour->contours, reversed);
			for (auto edge : edges) {
				if (edge->nextSum == next)
					edge->nextSum = edge == this ? nullptr : this;
				if (edge->priorSum == next)
					edge->priorSum = edge == this ? nullptr : this;
			}
		}
		next->fail = EdgeFail::priorDistance;
		OpEdge* unlink = next;
		next = next->nextSum;
		unlink->nextSum = nullptr;
	} while (next && EdgeFail::none == next->fail);
}

void OpEdge::markFailPrior(std::vector <OpEdge*>& edges, Axis axis) {
	// the ray hit multiple, indistinguishable edges
	OpEdge* prior = this;
	float direction = start.pt.choice(axis) - end.pt.choice(axis);
	do {
		if (this != prior) {
			float priorDirection = prior->start.pt.choice(axis) - prior->end.pt.choice(axis);
			bool reversed = direction * priorDirection < 0;
			winding.move(prior->winding, segment->contour->contours, reversed);
			for (auto edge : edges) {
				if (edge->nextSum == prior)
					edge->nextSum = edge == this ? nullptr : this;
				if (edge->priorSum == prior)
					edge->priorSum = edge == this ? nullptr : this;
			}
		}
		prior->fail = EdgeFail::nextDistance;
		OpEdge* unlink = prior;
		prior = prior->priorSum;
		unlink->priorSum = nullptr;
	} while (prior && EdgeFail::none == prior->fail);
}

bool OpEdge::matchLink(std::vector<OpEdge*>& linkups) {
	assert(lastEdge);
	assert(EdgeMatch::start == lastEdge->whichEnd || EdgeMatch::end == lastEdge->whichEnd);
	(void) setLinkBounds();
	// count intersections equaling end
	// each intersection has zero, one, or two active edges
	std::vector<FoundEdge> found;
#if 01 && OP_DEBUG
	if (103 == id && 108 == lastEdge->id) {
		OpDebugOut("");
	}
#endif
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
				assert(!edge->lastEdge->nextEdge);
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
		assert(closest);
		found.emplace_back(closest, closestEnd, closestReverse);
	}
	assert(found.size());
	OpRect bestBounds;
	closest = nullptr;
	closestEnd = EdgeMatch::none;
	closestReverse = AllowReversal::no;
	for (int trial = 0; !closest && trial < 2; ++trial) {
		for (const auto& foundOne : found) {
			OpEdge* oppEdge = foundOne.edge;
			// skip edges which flip the fill sum total, implying there is a third edge inbetween
			assert(EdgeMatch::start == foundOne.whichEnd || EdgeMatch::end == foundOne.whichEnd);
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
#if 01 && OP_DEBUG
	if (103 == id && 100 == closest->id) {
		OpDebugOut("");
	}
#endif
	assert(closest); // !!! if found is not empty, but no edge has the right sum, choose one anyway?
	if (AllowReversal::yes == closestReverse)
		closest->setLinkDirection(closestEnd);
	OpEdge* clearClosest = closest;
	do {
		clearClosest->clearActive();
		clearClosest = clearClosest->nextEdge;
	} while (clearClosest && clearClosest->active);
	closest->setPriorEdge(lastEdge);
	closest->priorLink = EdgeLink::single;
	lastEdge->setNextEdge(closest);
	lastEdge->nextLink = EdgeLink::single;
#if 0 && OP_DEBUG
	if (139 == lastEdge->id || 139 == closest->lastEdge->id)
		OpDebugOut("");
#endif
	lastEdge = closest->lastEdge;
	closest->lastEdge = nullptr;
	if (lastEdge->isClosed(this) || lastEdge->segment->contour->contours->closeGap(lastEdge, this))
		return lastEdge->validLoop(EdgeLoop::link);
	if (!lastEdge->nextEdge)
		return matchLink(linkups);
	return lastEdge->matchLink(linkups);
}

// check to see if next sum forms a loop
// A loop is detected if an edge is seen twice. The seen vector saves visits.
// The seen next breadcrumb prevents subsequent calls from finding an old loop.
bool OpEdge::nextSumLoops() {
	std::vector<const OpEdge*> seen;
	assert(!seenNext);
	seenNext = true;
	seen.push_back(this);
	OpEdge* next = this;
	while ((next = next->nextSum)) {
		if (next->seenNext)
			return seen.end() != std::find(seen.begin(), seen.end(), next);
		seen.push_back(next);
		next->seenNext = true;
	}
	return false;
}

OpEdge* OpEdge::prepareForLinkup() {
    OpEdge* first = this;
	while (EdgeLink::multiple != first->priorLink) {
		OpEdge* prior = first->priorEdge;
		if (!prior)
			break;
		assert(this != prior);
		first = prior;
	}
	OpEdge* next = first;
	OpEdge* last;
	do {
		assert(next->active);
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

// check to see if prior sum forms a loop
// A loop is detected if an edge is seen twice. The seen vector saves visits.
// The seen prior breadcrumb prevents subsequent calls from finding an old loop.
bool OpEdge::priorSumLoops() {
	std::vector<const OpEdge*> seen;
	assert(!seenPrior);
	seenPrior = true;
	seen.push_back(this);
	OpEdge* prior = this;
	while ((prior = prior->priorSum)) {
		if (prior->seenPrior)
			return seen.end() != std::find(seen.begin(), seen.end(), prior);
		seen.push_back(prior);
		prior->seenPrior = true;
	}
	return false;
}

// in function to make setting breakpoints easier
// !!! if this is not inlined in release, do something more cleverer
void OpEdge::setActive() {
	active = true;
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
	assert(++index == ptCount);
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
	assert(++index == ptCount);
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
		assert(priorEdge);
		while (first->priorEdge)
			first = first->priorEdge;
		assert(first->lastEdge);
		return first->setLinkBounds();
	}
	if (linkBounds.isSet())
		return linkBounds;
	linkBounds = pointBounds;
	const OpEdge* edge = this;
	while (edge != lastEdge) {
		edge = edge->nextEdge;
		linkBounds.add(edge->pointBounds);
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
	if (01) OpDebugBreak(this, 107, 119 == edge->id);
	nextEdge = edge;
}

void OpEdge::setPointBounds() {
	pointBounds.set(start.pt, end.pt);
#if OP_DEBUG
	OpPointBounds copy = pointBounds;
	for (int index = 0; index < segment->c.pointCount() - 2; ++index)
		pointBounds.add(ctrlPts[index]);
	assert(copy == pointBounds);
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
	assert(++index == ptCount);
}

// setter to make adding breakpoints easier
// !!! if release doesn't inline, restructure more cleverly...
void OpEdge::setPriorEdge(OpEdge* edge) {
	if (01) OpDebugBreak(this, 119, 107 == edge->id);
	priorEdge = edge;
}

const OpCurve& OpEdge::setVertical() {
	if (verticalSet)
		return vertical_impl;
	verticalSet = true;
	const OpCurve& curve = setCurve();
	std::array<OpPoint, 2> edgeLine = { start.pt, end.pt };
	curve.toVertical(edgeLine, vertical_impl);
	return vertical_impl;
}

// if edge normal points towards ray, set sumWinding to 0
void OpEdge::setWinding(OpVector ray) {
	OpVector normal = segment->c.normal(center.t);
	if (OpOperand::left == segment->contour->operand)
		sum.left = ray.dot(normal) >= 0;
	else
		sum.right = ray.dot(normal) >= 0;
#if OP_DEBUG
	sum.reason = ZeroReason::rayNormal;
#endif
}

// use already computed points stored in edge
void OpEdge::subDivide() {
	std::array<OpPoint, 4> pts;
	segment->c.subDivide(start, end, pts, &weight);
	setFromPoints(pts);
	setPointBounds();
	if (lineType == segment->c.type || (!pointBounds.height() ^ !pointBounds.width())) {
		isLine_impl = true;
		lineSet = true;
	}
	isPoint = start.pt == end.pt;
	if (isPoint)
		winding.zero(ZeroReason::isPoint);
	calcCenterT();
}

// note this should be only called on temporary edges (e.g., used to make coincident intersections)
// after this is called, edge does not agree with segment, but does match another edge
// this has to be called a second time (e.g., after trimming) to agree with segment
void OpEdge::reverse() {
	std::swap(start, end);
	if (cubicType == segment->c.type)
		std::swap(ctrlPts[0], ctrlPts[1]);
	start.t = 1 - start.t;
	end.t = 1 - end.t;
}

// fuzz-generated test crbug_526025 generates an edge link that is invalid. Worth chasing down 
// someday, but today, just return failure since the fuzz test won't succeed
bool OpEdge::validLoop(EdgeLoop edgeLoop) const {
	std::vector<const OpEdge*> seen;
	const OpEdge* last = this;
	for (;;) {
		if (seen.end() != std::find(seen.begin(), seen.end(), last)) {
			return true;
		}
		seen.push_back(last);
		const OpEdge* test = EdgeLoop::link == edgeLoop ? last->nextEdge : last->nextSum;
		if (!test)
			return true;
		if (last != (EdgeLoop::link == edgeLoop ? test->priorEdge : test->priorSum))
			return false;
		last = test;
	}
}

void OpWinding::move(OpWinding& opp, const OpContours* contours, bool backwards) {
	if (OpFillType::winding == contours->left)
		left += backwards ? -opp.left : opp.left;
	else
		left ^= opp.left;
	if (OpFillType::winding == contours->right)
		right += backwards ? -opp.right : opp.right;
	else
		right ^= opp.right;
	opp.left = 0;
	opp.right = 0;
}
