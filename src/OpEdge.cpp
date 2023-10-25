// (c) 2023, Cary Clark cclark2@gmail.com
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
	debugSetMaker = { file, line };
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
	debugSetMaker = { file, line };
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
		return CalcFail::fail; // e.g., may underflow if edge is too small
	return CalcFail::none;
}

void OpEdge::addPal(EdgeDistance& dist) {
	if (pals.end() == std::find_if(pals.begin(), pals.end(), 
			[dist](auto pal) { return dist.edge == pal.edge; }))
		pals.push_back(dist);
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

OpEdge* OpEdge::advanceToEnd(EdgeMatch match) {
	OpEdge* result = this;
	while (OpEdge* edge = (EdgeMatch::start == match ? result->priorEdge : result->nextEdge)) {
		result = edge;
	}
	return result;
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
	if (disabled || unsortable)
		return;
	OpWinding su = sum;
	OpWinding wi = winding;
	OpContours* contours = segment->contour->contours;
	WindState left = contours->windState(wi.left(), su.left(), OpOperand::left);
	WindState right = contours->windState(wi.right(), su.right(), OpOperand::right);
	if (left != WindState::flipOff && left != WindState::flipOn
			&& right != WindState::flipOff && right != WindState::flipOn) {
		setDisabled(OP_DEBUG_CODE(ZeroReason::noFlip));
		return;
	}
	bool bothFlipped = (left == WindState::flipOff || left == WindState::flipOn)
			&& (right == WindState::flipOff || right == WindState::flipOn);
	bool keep = false;
	switch (contours->opOperator) {
	case OpOperator::Subtract:
		keep = bothFlipped ? left != right : WindState::one == left || WindState::zero == right;
		if (keep)
			windZero = su.right() || !su.left() ? WindZero::normal : WindZero::opp;
		break;
	case OpOperator::Intersect:
		keep = bothFlipped ? left == right : WindState::zero != left && WindState::zero != right;
		if (keep)
			windZero = !su.left() || !su.right() ? WindZero::normal : WindZero::opp;
		break;
	case OpOperator::Union:
		keep = bothFlipped ? left == right : WindState::one != left && WindState::one != right;
		if (keep)
			windZero = !su.left() && !su.right() ? WindZero::normal : WindZero::opp;
		break;
	case OpOperator::ExclusiveOr:
		keep = !bothFlipped;
		if (keep)
			windZero = !su.left() == !su.right() ? WindZero::normal : WindZero::opp;
		break;
	case OpOperator::ReverseSubtract:
		keep = bothFlipped ? left != right : WindState::zero == left || WindState::one == right;
		if (keep)
			windZero = su.left() || !su.right() ? WindZero::normal : WindZero::opp;
		break;
	default:
		OP_ASSERT(0);
	}
	if (!keep)
		setDisabled(OP_DEBUG_CODE(ZeroReason::applyOp));
}

// old thinking:
// for center t (and center t only), use edge geometry since center is on edge even if there is error
// and using segment instead might return more than one intersection
// new thinking:
// segments are now broken monotonically when they are built, so they should not return more than
// one intersection anymore often than edges. 
bool OpEdge::calcCenterT() {
	const OpRect& r = ptBounds;
	Axis axis = r.width() >= r.height() ? Axis::vertical : Axis::horizontal;
	float middle = (r.ltChoice(axis) + r.rbChoice(axis)) / 2;
#if USE_SEGMENT_CENTER
	const OpCurve& curve = segment->c;
#else
	const OpCurve& curve = setCurve();
#endif
	float t = curve.center(axis, middle);
	if (OpMath::IsNaN(t)) {
		setDisabled(OP_DEBUG_CODE(ZeroReason::centerNaN));
		rayFail = EdgeFail::center;
		return true;
	}
#if USE_SEGMENT_CENTER
	center.t = t;
#else
	center.t = OpMath::Interp(start.t, end.t, t);
#endif
	center.pt = curve.ptAtT(t);
//#if !USE_SEGMENT_CENTER
	center.pt.pin(ptBounds);  // required by pentrek6
//#endif
	OP_ASSERT(OpMath::Between(ptBounds.left, center.pt.x, ptBounds.right));
	OP_ASSERT(OpMath::Between(ptBounds.top, center.pt.y, ptBounds.bottom));
	return start.t < center.t && center.t < end.t;
}

void OpEdge::clearActiveAndPals(ZeroReason reason) {
	setActive(false);
    for (auto& pal : pals) {
		if (ZeroReason::none != reason) {
			pal.edge->setActive(false);
			pal.edge->setDisabled(OP_DEBUG_CODE(reason));
		}
    }
	lastEdge = nullptr;
}

void OpEdge::clearNextEdge() {
	setNextEdge(nullptr);
}

void OpEdge::clearPriorEdge() {
	setPriorEdge(nullptr);
}

void OpEdge::complete() {
	winding.setWind(segment->winding.left(), segment->winding.right());
	subDivide();	// uses already computed points stored in edge
}

size_t OpEdge::countUnsortable() const {
	OP_ASSERT(!priorEdge);
	OP_ASSERT(lastEdge);
	OP_ASSERT(!debugIsLoop());
	const OpEdge* test = this;
	size_t count = 0;
	do {
		if (test->unsortable)
			count++;
	} while ((test = test->nextEdge));
	return count;
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

OpIntersection* OpEdge::findSect(EdgeMatch match) {
	OpPoint pt = whichPtT(match).pt;
	auto& i = segment->sects.i;
    auto found = std::find_if(i.begin(), i.end(), [pt](auto sect) { return sect->ptT.pt == pt; });
	OP_ASSERT(found != i.end());
	return *found;
}

float OpEdge::findT(Axis axis, float oppXY) const {
	float result;
    OP_DEBUG_CODE(FoundPtT foundPtT =) segment->findPtT(axis, start.t, end.t, oppXY, &result);
	OP_ASSERT(FoundPtT::single == foundPtT);
	OP_ASSERT(OpNaN != result);
	return result;
}

bool OpEdge::hasLinkTo(OpEdge* match) const {
	if (this == match)
		return true;
	OP_ASSERT(!priorEdge);
	OP_ASSERT(lastEdge);
	OP_ASSERT(!debugIsLoop());
	const OpEdge* test = this;
	while ((test = test->nextEdge)) {
		if (test == match)
			return true;
	}
	return false;
}

void OpEdge::linkToEdge(FoundEdge& found, EdgeMatch match) {
	OpEdge* oppEdge = found.edge;
	OP_ASSERT(!oppEdge->hasLinkTo(match));
	OP_ASSERT(oppEdge != this);
	const OpPoint edgePt = whichPtT(match).pt;
	if (EdgeMatch::start == match) {
		OP_ASSERT(!priorEdge);
		setPriorEdge(oppEdge);
		oppEdge->setNextEdge(this);
	} else {
		OP_ASSERT(!nextEdge);
		setNextEdge(oppEdge);
		oppEdge->setPriorEdge(this);
	}
	if (edgePt == oppEdge->start.pt)
		oppEdge->whichEnd = Opposite(match);
	else {
		OP_ASSERT(edgePt == oppEdge->end.pt);
		oppEdge->whichEnd = match;
	}
}

// Find pals for unsectables created during curve/curve intersection. There should be at most
// two matching unsectable ids in the distances array. Mark between edges as well.
void OpEdge::markPals() {
	EdgeDistance* pal = nullptr;
	bool foundUnsectable = false;
	for (auto& dist : ray.distances) {
		if (dist.edge->unsectableID == unsectableID) {
			if (this != dist.edge)
				pal = &dist;
			if (foundUnsectable) {	// found both ends
				addPal(*pal);
				return;
			}
			foundUnsectable = true;
			continue;
		}
		if (!foundUnsectable)
			continue;
		if (!dist.edge->unsectableID && !dist.edge->pals.size())
			dist.edge->between = true;
	}
	// !!! not sure how to assert an error if information was inconsistent (e.g., unpaired)
}

// keep only one unsectable from any set of pals
void OpEdge::matchUnsectable(EdgeMatch match, const std::vector<OpEdge*>& unsectInX,
		std::vector<FoundEdge>& edges, AllowPals allowPals, AllowClose allowClose) {
	const OpPoint firstPt = whichPtT(match).pt;
	for (int index = 0; index < (int) unsectInX.size(); ++index) {
		OpEdge* unsectable = unsectInX[index];
		if (this == unsectable)
			continue;
		auto isDupe = [&edges](OpEdge* test) {
			for (const auto& found : edges) {
				if (found.edge == test)
					return true;
				for (auto& pal : found.edge->pals)
					if (pal.edge == test)
						return true;
			}
			return false;
		};
		auto checkEnds = [this, &edges, firstPt, isDupe, allowPals, allowClose]
				(OpEdge* unsectable) {
			if (unsectable->inOutput || unsectable->inLinkups || unsectable->disabled)
				return false;
			if (this == unsectable)
				return false;
#if 0
			bool startMatch = (firstPt == unsectable->whichPtT().pt || AllowClose::yes == allowClose)
					&& !unsectable->priorEdge;
			bool endMatch = (firstPt == unsectable->whichPtT(EdgeMatch::end).pt
					|| AllowClose::yes == allowClose) && !unsectable->nextEdge;
#else
            bool startMatch = firstPt == unsectable->start.pt
                    && (EdgeMatch::start == unsectable->whichEnd ? !unsectable->priorEdge :
                    !unsectable->nextEdge);
            bool endMatch = firstPt == unsectable->end.pt
                    && (EdgeMatch::end == unsectable->whichEnd ? !unsectable->priorEdge :
                    !unsectable->nextEdge);
#endif
			if (!startMatch && !endMatch)
				return false;
			if (AllowClose::no == allowClose && isDupe(unsectable))
				return false;
#if 0
			if (unsectable->pals.size() && AllowPals::yes == allowPals) {
#else
			if (unsectable->pals.size() && AllowPals::no == allowPals) {
#endif
			const OpEdge* link = this;
				OP_ASSERT(!link->nextEdge);
				OP_ASSERT(!link->debugIsLoop());
				do {
					if (unsectable->isPal(link))
						return false;
					link = link->priorEdge;
				} while (link);
			}
			OP_ASSERT(!unsectable->debugIsLoop());
			if (AllowClose::yes == allowClose) {
				OP_ASSERT(1 == edges.size());
				edges.back().check(nullptr, unsectable, 
						startMatch ? EdgeMatch::start : EdgeMatch::end, firstPt);
			} else
#if 0
				edges.emplace_back(unsectable, startMatch ? unsectable->whichEnd 
						: Opposite(unsectable->whichEnd));
#else
				edges.emplace_back(unsectable, startMatch ? EdgeMatch::start : EdgeMatch::end);
#endif
			return true;
		};
		if (checkEnds(unsectable))
			continue;
		if (AllowPals::no == allowPals)
			continue;
		for (auto& pal : unsectable->pals) {
			if (checkEnds(pal.edge))
				break;
		}
	}
}

OpEdge* OpEdge::nextOut() {
	clearActiveAndPals(ZeroReason::addedPalToOutput);
	inLinkups = false;
    inOutput = true;
    return nextEdge;
}

NormalDirection OpEdge::normalDirection(Axis axis, float t) {
	const OpCurve& curve = setCurve();
	return curve.normalDirection(axis, t);
}

OpType OpEdge::type() {
	return isLinear() ? OpType::line : segment->c.type; 
}

// in function to make setting breakpoints easier
// !!! if this is not inlined in release, do something more cleverer
void OpEdge::setActive(bool state) {
	active_impl = state;
}

void OpEdge::setBetween() {
	setUnsortable();
	between = true;
}

const OpCurve& OpEdge::setCurve() {
	if (!curveSet) {
		curveSet = true;
		curve_impl.set(start.pt, ctrlPts, end.pt, segment->c.pointCount(), segment->c.type, weight);
	}
	return curve_impl;
}

// should be inlined. Out of line for ease of setting debugging breakpoints
void OpEdge::setDisabled(OP_DEBUG_CODE(ZeroReason reason)) {
	disabled = true; 
	OP_DEBUG_CODE(debugZero = reason); 
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

OpEdge* OpEdge::setLastEdge(OpEdge* old) {
	if (old)
		old->lastEdge = nullptr;
	lastEdge = nullptr;
	OpEdge* linkStart = advanceToEnd(EdgeMatch::start);
	OpEdge* linkEnd = advanceToEnd(EdgeMatch::end);
	linkStart->lastEdge = linkEnd;
	return linkEnd;
}

// this sets up the edge linked list to be suitable for joining another linked list
// the edits are nondestructive 
bool OpEdge::setLastLink(EdgeMatch match) {
	if (!priorEdge && !nextEdge) {
		if (EdgeMatch::none == whichEnd) {
			OP_ASSERT(!lastEdge);
			lastEdge = this;
		} else
			OP_ASSERT(lastEdge);
		whichEnd = match;
		return false;
	} 
	if (!lastEdge)
		return setLinkDirection(EdgeMatch::end);
	if (lastEdge == this) {
		if (EdgeMatch::end == match && EdgeMatch::start == whichEnd)
			whichEnd = EdgeMatch::end;
		else if (EdgeMatch::start == match && EdgeMatch::end == whichEnd)
			whichEnd = EdgeMatch::start;
	}
	return false;
}

// this compares against float epsilon instead of zero
// when comparing against a line, an edge close to zero can fall into denormalized numbers,
//   causing the calling subdivision to continue for way too long. Using epsilon as a stopgap
//   avoids this. The alternative would be to change the math to disallow denormalized numbers
bool OpEdge::isLinear() {
	if (lineSet)
		return isLine_impl;
	lineSet = true;
	const OpCurve& curve = setCurve();
	return (isLine_impl = curve.isLinear());
}

OpPointBounds OpEdge::setLinkBounds() {
	OP_ASSERT(lastEdge); // fix caller to pass first edge of links
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
bool OpEdge::setLinkDirection(EdgeMatch match) {
	if (EdgeMatch::start == match && lastEdge)	// !!! this requires that lastEdge only be at start
		return false;
	OpEdge* edge = this;
	while (edge->priorEdge) {
		std::swap(edge->priorEdge, edge->nextEdge);
		edge->whichEnd = Opposite(edge->whichEnd);
		edge = edge->nextEdge;
	}
	std::swap(edge->priorEdge, edge->nextEdge);
	edge->whichEnd = Opposite(edge->whichEnd);
	edge->lastEdge = nullptr;
	lastEdge = edge;
	return true;
}

void OpEdge::setNextEdge(OpEdge* edge) {
	if (nextEdge)
		nextEdge->priorEdge = nullptr;
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

void OpEdge::setPriorEdge(OpEdge* edge) {
	if (priorEdge)
		priorEdge->nextEdge = nullptr;
	priorEdge = edge;
}

void OpEdge::setUnsortable() {
	unsortable = true;
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
void OpEdge::skipPals(EdgeMatch match, std::vector<FoundEdge>& edges) {
	std::vector<FoundEdge> sectables;
	std::vector<FoundEdge> unsectables;
	bool duplicates = false;
	for (const auto& found : edges) {
		if (found.edge->pals.size()) {
			bool refsLinkups = false;
			for (auto pal : found.edge->pals) {
				if ((refsLinkups = pal.edge->inLinkups))
					break;
				OpEdge* test = this;
				OP_ASSERT(!test->debugIsLoop());
				do {
					if ((refsLinkups = (test == pal.edge)))
						break;
				} while ((test = test->priorEdge));
				test = this;
				while ((test = test->nextEdge)) {
					if ((refsLinkups = (test == pal.edge)))
						break;
				}
			}
			if (refsLinkups)
				duplicates = true;
			else
				unsectables.push_back(found);
		} else
			sectables.push_back(found);
	}
	OpEdge* first = advanceToEnd(EdgeMatch::start);
	first->setLastEdge(nullptr);
	for (unsigned oIndex = 1; oIndex < unsectables.size(); ++oIndex) {
		FoundEdge& outer = unsectables[oIndex - 1];	// note: cannot underflow
		auto& oPals = outer.edge->pals;
		OpRect outerBounds = first->setLinkBounds().add(outer.edge->ptBounds);
		for (unsigned iIndex = oIndex; iIndex < unsectables.size(); ++iIndex) {
			FoundEdge& inner = unsectables[iIndex];
			bool outerTouchesInner = oPals.end() != std::find_if(oPals.begin(), oPals.end(), 
					[&inner](auto oPal) { return oPal.edge == inner.edge; });
			auto& iPals = inner.edge->pals;
			bool innerTouchesOuter = iPals.end() != std::find_if(iPals.begin(), iPals.end(), 
					[&outer](auto iPal) { return iPal.edge == outer.edge; });
			if (!outerTouchesInner && !innerTouchesOuter)
				continue;
			duplicates = true;
			OpRect innerBounds = first->setLinkBounds().add(inner.edge->ptBounds);
			if (outerBounds.perimeter() <= innerBounds.perimeter())
				continue;
			std::swap(inner, outer);
			std::swap(innerBounds, outerBounds);
		}
		sectables.push_back(outer);
	}
	if (!duplicates)
		return;
	std::swap(sectables, edges);
}

// use already computed points stored in edge
void OpEdge::subDivide() {
	id = segment->nextID();
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
//		OP_ASSERT(0);	// triggered by fuzz763_9
		setDisabled(OP_DEBUG_CODE(ZeroReason::isPoint));
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

#if OP_DEBUG
bool OpEdge::debugFail() const {
    return segment->debugFail();
}
#endif

// keep this in sync with op edge : debug dump chain
// ignore axis changes when detecting sum loops (for now)
// !!! if the axis change is required to detect for sum loops, document why!
// !!! either add 'stamp' or rewalk links instead of find
#if OP_DEBUG_DUMP || OP_DEBUG
const OpEdge* OpEdge::debugIsLoop(WhichLoop which, LeadingLoop leading) const {
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
#endif

bool OpEdgeStorage::contains(OpIntersection* start, OpIntersection* end) const {
	for (int index = 0; index < used; index += sizeof(OpEdge)) {
		const OpEdge* test = (const OpEdge*) &storage[index];
		if (test->segment == start->segment && test->start == start->ptT
				&& test->end == end->ptT)
			return true;
	}
	if (!next)
		return false;
	return next->contains(start, end);
}

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

void OpWinding::setSum(OpWinding winding, const OpSegment* segment) {
	OP_ASSERT(WindingType::uninitialized == debugType);
	OP_ASSERT(WindingType::temp == winding.debugType);
	OP_DEBUG_CODE(debugType = WindingType::sum);
	const OpContours& contours = *segment->contour->contours;
	left_impl = winding.left() & contours.leftFillTypeMask();
	right_impl = winding.right() & contours.rightFillTypeMask();
}
