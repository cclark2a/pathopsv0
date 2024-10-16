// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpEdge.h"
#include "OpContour.h"
#include "OpCurveCurve.h"

#if OP_DEBUG
#include "OpJoiner.h"
#include "PathOps.h"
#endif

// don't add if points are close
// prefer end if types are different
// return true if close enough points are ctrl + mid
bool OpHulls::add(const OpPtT& ptT, OpVector threshold, SectType sectType, const OpEdge* opp) {			
	auto found = std::find_if(h.begin(), h.end(), [ptT, threshold](const HullSect& hull) {
		return ptT.isNearly(hull.sect, threshold);
	});
	if (h.end() == found) {
		h.emplace_back(ptT, sectType, opp);
		return false;
	}
	if (SectType::endHull == sectType) {
		found->sect = ptT;
		found->type = sectType;
		return false;
	}
	OP_ASSERT(SectType::midHull == sectType || SectType::controlHull == sectType);
	return SectType::endHull != found->type && found->type != sectType;
}

bool OpHulls::closeEnough(int index, const OpEdge& edge, const OpEdge& oEdge, OpPtT* oPtT,
		OpPtT* hull1Sect) {
	if (!hull1Sect->pt.isNearly(oPtT->pt, edge.segment->threshold())) {
		const OpCurve& eCurve = edge.segment->c;
		OpVector eTangent = eCurve.tangent(hull1Sect->t);
		const OpCurve& oCurve = oEdge.segment->c;
		OpVector oTangent = oCurve.tangent(oPtT->t);
		LinePts eLine { hull1Sect->pt, hull1Sect->pt + eTangent };
		LinePts oLinePts = {{ oPtT->pt, oPtT->pt + oTangent }};
		OpRoots oRoots = eLine.tangentIntersect(oLinePts);
		if (2 == oRoots.count)
			return false;  // if tangents are parallel and not coincident: no intersection
		OP_ASSERT(1 == oRoots.count);
		if (0 > oRoots.roots[0] || oRoots.roots[0] > 1) {
			OpPtT::MeetInTheMiddle(*oPtT, *hull1Sect);
		} else {
			OpPoint sectPt = eLine.ptAtT(oRoots.roots[0]);
			Axis eLarger = edge.ptBounds.largerAxis();
			OpPtT ePtT = edge.findT(eLarger, sectPt.choice(eLarger));
			float newOPtT = oEdge.segment->findValidT(0, 1, sectPt);
			if (OpMath::IsNaN(newOPtT))
				return false;
			*oPtT = OpPtT(sectPt, newOPtT);
			*hull1Sect = OpPtT(sectPt, ePtT.t);
		}
	}
	return true;
}

bool OpHulls::sectCandidates(int index, const OpEdge& edge) {
	const HullSect& hullStart = h[index - 1];
	const HullSect& hullEnd = h[index];
	OpPtT hull1Sect = hullStart.sect;
	const OpPtT& hull2Sect = hullEnd.sect;
	if (!hull1Sect.isNearly(hull2Sect, edge.segment->threshold()))
		return false;
	if (SectType::controlHull == h[index - 1].type
			&& SectType::controlHull == h[index].type)
		return false;
	if (SectType::endHull == hullStart.type || SectType::endHull == hullEnd.type) {
		// check to see if hull pt is close to original edge
		Axis eLarger = edge.ptBounds.largerAxis();
		float eXy1 = hull1Sect.pt.choice(eLarger);
		float eXy2 = hull2Sect.pt.choice(eLarger);
		float eXyAvg = OpMath::Average(eXy1, eXy2);
		OpPtT ePtT = edge.findT(eLarger, eXyAvg);
		if (!ePtT.pt.isFinite()) {
			ePtT.pt.choice(eLarger) = eXyAvg;
			ePtT.pt.choice(!eLarger) = edge.segment->c.ptAtT(ePtT.t).choice(!eLarger);
		}
		if (!hull1Sect.isNearly(ePtT, edge.segment->threshold()))
			return false;
	}
	return true;
}

void OpHulls::nudgeDeleted(const OpEdge& edge, const OpCurveCurve& cc, CurveRef which) {
	for (;;) {
		sort(false);
		for (size_t index = 0; index + 1 < h.size(); ) {
			// while hull sect is in a deleted bounds, bump its t and recompute
			if ((SectType::midHull == h[index].type || SectType::controlHull == h[index].type)
					&& cc.checkSplit(edge.startT, h[index + 1].sect.t, which, h[index].sect))
				goto tryAgain;
			++index;
			if ((SectType::midHull == h[index].type || SectType::controlHull == h[index].type)
					&& cc.checkSplit(h[index - 1].sect.t, edge.endT, which, h[index].sect))
				goto tryAgain;
			OP_ASSERT(h[index - 1].sect.t < h[index].sect.t);
		}
		break;
tryAgain:
		;
	}
}

void OpHulls::sort(bool useSmall) {
	std::sort(h.begin(), h.end(), [useSmall](const HullSect& s1, const HullSect& s2) {
		return useSmall ? s1.sect.t > s2.sect.t : s1.sect.t < s2.sect.t;
	});
}

EdgePal::EdgePal(OpEdge* e, float c, float tIn, bool r)
	: edge(e)
	, cept(c)
	, edgeInsideT(tIn)
	, reversed(r) {
	OP_DEBUG_CODE(debugUID = 0);
}

EdgePal::EdgePal(OpEdge* e, bool r  OP_DEBUG_PARAMS(int uID))
	: edge(e)
	, cept(OpNaN)
	, edgeInsideT(OpNaN)
	, reversed(r) {
	OP_DEBUG_CODE(debugUID = uID);
}

// called when creating edge for curve curve intersection building
OpEdge::OpEdge(OpSegment* s  OP_LINE_FILE_DEF())
	: OpEdge() {
	OP_LINE_FILE_SET(debugSetMaker);
	OP_DEBUG_CODE(debugParentID = s->id);
	segment = s;
//	startSect = -1;
//	endSect = -1;
	startT = 0;
	endT = 1;
	complete(s->c.firstPt(), s->c.lastPt());
}

// called when creating edge from intersection pairs
OpEdge::OpEdge(OpIntersection* sectStart, OpIntersection* sectEnd  OP_LINE_FILE_DEF())
	: OpEdge() {
	OP_LINE_FILE_SET(debugSetMaker);
	OP_DEBUG_CODE(debugParentID = sectStart->id);
	segment = sectStart->segment;
	startT = sectStart->ptT.t;
	endT = sectEnd->ptT.t;
	complete(sectStart->ptT.pt, sectEnd->ptT.pt);
}

// called when creating filler; edge that closes small gaps
OpEdge::OpEdge(OpContours* contours, const OpPtT& start, const OpPtT& end  OP_LINE_FILE_DEF())
	: OpEdge() {
	OP_LINE_FILE_SET(debugSetMaker);
	OP_DEBUG_CODE(debugParentID = 0);
	OP_DEBUG_IMAGE_CODE(if (!debugCustom) debugColor = mediumPurple);
	segment = nullptr;  // assume these can't be used -- edge does not exist in segment
//	startSect = -1;
//	endSect = -1;
	startT = start.t;
	endT = end.t;
	id = contours->nextID();
	PathOpsV0Lib::CurveData lineData { start.pt, end.pt };
	PathOpsV0Lib::Curve lineCurve { &lineData, sizeof(lineData), (PathOpsV0Lib::CurveType) 0 };
	PathOpsV0Lib::Curve userCurve = contours->contextCallBacks.makeLineFuncPtr(lineCurve);
	curve = OpCurve(contours, userCurve);
	curve.isLineSet = true;
	curve.isLineResult = true;
	setPointBounds();
	center.t = OpMath::Interp(startT, endT, .5);
	center.pt = ptBounds.center();
    setDisabled(OP_LINE_FILE_NPARAMS());
    setUnsortable(Unsortable::filler);
}

// called from curve curve when splitting edges
OpEdge::OpEdge(const OpEdge* edge, const OpPtT& newPtT, NewEdge isLeftRight  OP_LINE_FILE_DEF())
	: OpEdge() {
	OP_LINE_FILE_SET(debugSetMaker);
	OP_DEBUG_CODE(debugParentID = edge->id);
	segment = edge->segment;
//	startSect = -1;
//	endSect = -1;
	if (NewEdge::isLeft == isLeftRight) {
		startT = edge->startT;
		endT = newPtT.t;
		complete(edge->startPt(), newPtT.pt);
	} else {
		startT = newPtT.t;
		endT = edge->endT;
		complete(newPtT.pt, edge->endPt());
	}
}

// called by curve curve's snip range
OpEdge::OpEdge(const OpEdge* edge, const OpPtT& s, const OpPtT& e  OP_LINE_FILE_DEF())
	: OpEdge() {
	OP_LINE_FILE_SET(debugSetMaker);
	OP_DEBUG_CODE(debugParentID = edge->id);
	segment = edge->segment;
//	startSect = -1;
//	endSect = -1;
	startT = s.t;
	endT = e.t;
	complete(s.pt, e.pt);
}

OpEdge::OpEdge(const OpEdge* edge, float t1, float t2  OP_LINE_FILE_DEF())
	: OpEdge() {
	OP_LINE_FILE_SET(debugSetMaker);
	OP_DEBUG_CODE(debugParentID = edge->id);
	segment = edge->segment;
	startT = t1;
	endT = t2;
	complete(segment->c.ptAtT(t1), segment->c.ptAtT(t2));
}

CalcFail OpEdge::addIfUR(Axis axis, float edgeInsideT, OpWinding* sumWinding) {
	NormalDirection NdotR = normalDirection(axis, edgeInsideT);
	if (NormalDirection::upRight == NdotR)
		sumWinding->add(winding);
	else if (NormalDirection::downLeft != NdotR)
		return CalcFail::fail; // e.g., may underflow if edge is too small
	return CalcFail::none;
}

void OpEdge::addPal(const EdgePal& dist) {
	if (pals.end() == std::find_if(pals.begin(), pals.end(), 
			[dist](auto pal) { return dist.edge == pal.edge; }))
		pals.push_back(dist);
}

// given an intersecting ray and edge t, add or subtract edge winding to sum winding
// but don't change edge's sum, since an unsectable edge does not allow that accumulation
CalcFail OpEdge::addSub(Axis axis, float edgeInsideT, OpWinding* sumWinding) {
	NormalDirection NdotR = normalDirection(axis, edgeInsideT);
	if (NormalDirection::upRight == NdotR)
		sumWinding->add(winding);
	else if (NormalDirection::downLeft == NdotR)
		sumWinding->subtract(winding);
	else
		OP_DEBUG_FAIL(*this, CalcFail::fail);
	return CalcFail::none;
}

OpEdge* OpEdge::advanceToEnd(EdgeMatch match) {
	OP_ASSERT(!debugIsLoop(match));
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
	if (EdgeFail::center == rayFail)
		setDisabled(OP_LINE_FILE_NPARAMS());
	if (disabled || Unsortable::none != isUnsortable)
		return;
	OpContour* contour = segment->contour;
	PathOpsV0Lib::WindKeep keep = contour->callBacks.windingKeepFuncPtr(winding.w, sum.w);
	switch (keep) {
		case PathOpsV0Lib::WindKeep::Discard:
			setDisabled(OP_LINE_FILE_NPARAMS());
			return;
		case PathOpsV0Lib::WindKeep::End:
			windZero = WindZero::zero;
			return;
		case PathOpsV0Lib::WindKeep::Start:
			windZero = WindZero::nonZero;
			return;
	}
	OP_ASSERT(0);
}

// old thinking:
// for center t (and center t only), use edge geometry since center is on edge even if there is error
// and using segment instead might return more than one intersection
// new thinking:
// segments are now broken monotonically when they are built, so they should not return more than
// one intersection anymore often than edges. 
void OpEdge::calcCenterT() {
	const OpRect& r = ptBounds;
	Axis axis = r.largerAxis();
	float middle = OpMath::Average(r.ltChoice(axis), r.rbChoice(axis));
	const OpCurve& segCurve = segment->c;
	float t = segCurve.center(axis, middle);
	if (OpMath::IsNaN(t)) {
// while this should be disabled eventually, it must be visible to influence winding calc
// other edges' rays that hit this should also be disabled and marked ray fail
//		setDisabled(OP_LINE_FILE_NARGS());
		OP_LINE_FILE_SET_IMMED(debugSetDisabled);
		OP_DEBUG_CODE(center = { OpPoint(SetToNaN::dummy), OpNaN } );
		centerless = true;  // !!! is this redundant with ray fail?
		rayFail = EdgeFail::center;
		return;
	}
	if (startT >= t || t >= endT)
		t = OpMath::Average(startT, endT);
	center.t = t;
	center.pt = segCurve.ptAtT(t);
	center.pt.pin(ptBounds);  // required by pentrek6
	OP_ASSERT(OpMath::Between(ptBounds.left, center.pt.x, ptBounds.right));
	OP_ASSERT(OpMath::Between(ptBounds.top, center.pt.y, ptBounds.bottom));
}

void OpEdge::clearActiveAndPals(OP_LINE_FILE_NP_DEF()) {
	setActive(false);
    for (auto& pal : pals) {
		if (!pal.edge->isUnsectable())
			continue;  // !!! hack ?
		pal.edge->setActive(false);
		pal.edge->setDisabled(OP_LINE_FILE_NP_CALLER());
    }
	clearLastEdge();
}

void OpEdge::clearLastEdge() {
	lastEdge = nullptr;
}

void OpEdge::clearNextEdge() {
	setNextEdge(nullptr);
}

void OpEdge::clearPriorEdge() {
	setPriorEdge(nullptr);
}

void OpEdge::complete(OpPoint startPoint, OpPoint endPoint) {
	subDivide(startPoint, endPoint);	// uses already computed points stored in edge
	winding.setWind(segment->winding);
}

OpContours* OpEdge::contours() const {
	return segment->contour->contours;
}

size_t OpEdge::countUnsortable() const {
	OP_ASSERT(!priorEdge);
	OP_ASSERT(lastEdge);
	OP_ASSERT(!debugIsLoop());
	const OpEdge* test = this;
	size_t count = 0;
	do {
		if (Unsortable::none != test->isUnsortable)
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

OpPtT OpEdge::findT(Axis axis, float oppXY) const {
	OpPtT found;
	float startXY = startPt().choice(axis);
	float endXY = endPt().choice(axis);
	if (oppXY == startXY)
		found = start();
	else if (oppXY == endXY)
		found = end();
	else {
		found.pt = OpPoint(SetToNaN::dummy);
		found.t = segment->findAxisT(axis, startT, endT, oppXY);
		if (OpMath::IsNaN(found.t))
			found = (oppXY < startXY) == (startXY < endXY) ? start() : end();
	}
	return found;
}

// !!! (out of date comment) this compares against float epsilon instead of zero
// when comparing against a line, an edge close to zero can fall into denormalized numbers,
//   causing the calling subdivision to continue for way too long. Using epsilon as a stopgap
//   avoids this. The alternative would be to change the math to disallow denormalized numbers
bool OpEdge::isLine() {
	return curve.isLine();
}

bool OpEdge::isUnsectablePair(OpEdge* opp) {
	return pals.end() != std::find_if(pals.begin(), pals.end(), [opp](const EdgePal& edgePal) {
			return edgePal.edge == opp; });
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
	if (edgePt == oppEdge->startPt())
		oppEdge->setWhich(!match);
	else {
		OP_ASSERT(edgePt == oppEdge->endPt());
		oppEdge->setWhich(match);
	}
}

bool OpEdge::linksTo(OpEdge* match) const {
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

// Find pals for unsectables created during curve/curve intersection. There should be at most
// two matching unsectable ids in the distances array. Mark between edges as well.
void OpEdge::markPals() {
	OP_ASSERT(isUnsectable());
	// edge is between one or more unsectableID ranges in intersections
	for (EdgePal& pal : pals) {
		for (auto& dist : ray.distances) {
			if (pal.edge == dist.edge)
				addPal(dist);
		}
	}
}

MatchReverse OpEdge::matchEnds(const LinePts& linePts) const {
	return curve.matchEnds(linePts);
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
            bool startMatch = firstPt == unsectable->startPt()
                    && (EdgeMatch::start == unsectable->which() ? !unsectable->priorEdge :
                    !unsectable->nextEdge);
            bool endMatch = firstPt == unsectable->endPt()
                    && (EdgeMatch::end == unsectable->which() ? !unsectable->priorEdge :
                    !unsectable->nextEdge);
			if (!startMatch && !endMatch)
				return false;
			if (AllowClose::no == allowClose && isDupe(unsectable))
				return false;
			if (unsectable->isUnsectable() && AllowPals::no == allowPals) {
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
				edges.emplace_back(unsectable, startMatch ? EdgeMatch::start : EdgeMatch::end);
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
	clearActiveAndPals(OP_LINE_FILE_NPARAMS());
	inLinkups = false;
    inOutput = true;
	OP_DEBUG_IMAGE_CODE(if (!debugCustom) debugColor = orange);
    return nextEdge;
}

// !!! note that t value is 0 to 1 within edge (not normalized to segment t)
NormalDirection OpEdge::normalDirection(Axis axis, float edgeInsideT) {
	return curve.normalDirection(axis, edgeInsideT);
}

// if there is another path already output, and it is first found in this ray,
// check to see if the tangent directions are opposite. If they aren't, reverse
// this edge's links before sending it to the host graphics engine
void OpEdge::output(bool closed) {
    const OpEdge* firstEdge = closed ? this : nullptr;
    OpEdge* edge = this;
	bool reverse = false;
	bool abort = false;
	// returns true if reverse/no reverse criteria found
	auto test = [&reverse, &abort](const EdgePal* outer, const EdgePal* inner) {
		if (!outer->edge->inOutput && !outer->edge->inLinkups)
			return false;
		// reverse iff normal direction of inner and outer match and outer normal points to nonzero
		OpEdge* oEdge = outer->edge;
		Axis axis = oEdge->ray.axis;
		NormalDirection oNormal = oEdge->normalDirection(axis, outer->edgeInsideT);
		if ((oEdge->windZero == WindZero::zero) == (NormalDirection::upRight == oNormal))
			return true;  // don't reverse if outer normal in direction of inner points to zero
		OpEdge* iEdge = inner->edge;
	//	OP_ASSERT(!iEdge->inOutput);  // triggered by cubic1810520
		if (iEdge->inOutput && !iEdge->isUnsectable()) {  // defer dealing with this until we find an easier test case
			OpDebugOut("!!! edge already output\n");
			abort = true;
			return true;
		}
		if (axis != iEdge->ray.axis)
			return false;
		NormalDirection iNormal = iEdge->normalDirection(axis, inner->edgeInsideT);
		if (EdgeMatch::end == iEdge->which())
			iNormal = !iNormal;
		if (NormalDirection::downLeft != iNormal 
				&& NormalDirection::upRight != iNormal)
			return false;
		if (EdgeMatch::end == oEdge->which())
			oNormal = !oNormal;
		if (NormalDirection::downLeft != oNormal 
				&& NormalDirection::upRight != oNormal)
			return false;
		reverse = iNormal == oNormal;
		return true;
	};
	do {
	//	OP_ASSERT(!edge->inOutput);	// !!! cubic714074 triggers with very small edge, used twice
		unsigned index;
		const EdgePal* inner;
		for (index = 0; index < edge->ray.distances.size(); ++index) {
			inner = &edge->ray.distances[index];
			if (inner->edge == edge)
				break;
		}
		OP_ASSERT(!index || index < edge->ray.distances.size());
		if (index == 0)  // if nothing to its left, don't reverse
			break;
		const EdgePal* outer = &edge->ray.distances[index - 1];
		if (test(outer, inner))
			break;
		edge = edge->nextEdge;
    } while (firstEdge != edge);
	if (abort)
		return;
	if (reverse) {
		if (priorEdge) {
			OP_ASSERT(debugIsLoop());
			lastEdge = priorEdge;
			lastEdge->nextEdge = nullptr;
			priorEdge = nullptr;
		}
		edge = lastEdge;
		edge->setLinkDirection(EdgeMatch::none);
		firstEdge = nullptr;
	} else
		edge = this;
	edge->outputLinkedList(firstEdge, true);
}

void OpEdge::outputLinkedList(const OpEdge* firstEdge, bool first)
{
//	PathOpsV0Lib::PathOutput nativePath = contours()->callerOutput;
	OP_DEBUG_CODE(debugOutPath = curve.contours->debugOutputID);
	OpEdge* next = nextOut();
	OpCurve copy = curve;
	if (EdgeMatch::end == which())
		copy.reverse();
	copy.output(first, firstEdge == next);
	if (firstEdge == next) {
		OP_DEBUG_CODE(debugOutPath = curve.contours->nextID());
		return;
	}
	OP_ASSERT(next);
	next->outputLinkedList(firstEdge, false);
}

PathOpsV0Lib::CurveType OpEdge::type() {
	return segment->c.c.type; 
}

// in function to make setting breakpoints easier
// !!! if this is not inlined in release, do something more cleverer
void OpEdge::setActive(bool state) {
	active_impl = state;
}

bool OpEdge::isClose() {
	if (closeSet)
		return isClose_impl;
	closeSet = true;
	if (ccStart || ccEnd) {
		OP_ASSERT(!isClose_impl);
		return false;
	}
	return isClose_impl = start().isNearly(end(), contours()->threshold());
}

OpPtT OpEdge::ptTCloseTo(OpPtT oPtPair, const OpPtT& ptT) const {
	OpVector unitTan = segment->c.tangent(ptT.t);
	OpVector tan = unitTan.setLength(sqrtf(oPtPair.t));
	if (1 == ptT.t)
		tan = -tan;
	OpPoint testPt = ptT.pt + tan;
	if (!ptBounds.contains(testPt))
		return center;
	// use unit tan to pass correct axis to find pt t
	Axis axis = fabsf(unitTan.dx) > fabsf(unitTan.dy) ? Axis::vertical : Axis::horizontal; 
	float resultT = segment->findAxisT(axis, startT, endT, testPt.choice(axis));
	if (!OpMath::IsNaN(resultT))
		return OpPtT(segment->c.ptAtT(resultT), resultT);
	return center;
}
	
// should be inlined. Out of line for ease of setting debugging breakpoints
void OpEdge::setDisabled(OP_LINE_FILE_NP_DEF()) {
	disabled = true; 
	OP_LINE_FILE_SET(debugSetDisabled); 
	OP_DEBUG_IMAGE_CODE(if (!debugCustom) debugColor = red);
}

OpEdge* OpEdge::setLastEdge() {
	clearLastEdge();
	OpEdge* linkStart = advanceToEnd(EdgeMatch::start);
	OpEdge* linkEnd = advanceToEnd(EdgeMatch::end);
	linkStart->lastEdge = linkEnd;
	return linkEnd;
}

// this sets up the edge linked list to be suitable for joining another linked list
// the edits are nondestructive 
bool OpEdge::setLastLink(EdgeMatch match) {
	if (!priorEdge && !nextEdge) {
		lastEdge = this;
		setWhich(match);
		return false;
	} 
	if (!lastEdge)
		return setLinkDirection(EdgeMatch::end);
	if (lastEdge == this) {
		if (EdgeMatch::end == match && EdgeMatch::start == which())
			setWhich(EdgeMatch::end);
		else if (EdgeMatch::start == match && EdgeMatch::end == which())
			setWhich(EdgeMatch::start);
	}
	return false;
}

OpPointBounds OpEdge::setLinkBounds() {
	OP_ASSERT(lastEdge);  // fix caller to pass first edge of links
	if (!linkBounds.isSet()) {
		OP_ASSERT(ptBounds.isFinite());
		linkBounds = ptBounds;
		const OpEdge* edge = this;
		while (edge != lastEdge) {
			edge = edge->nextEdge;
			OP_ASSERT(edge->ptBounds.isFinite());
			linkBounds.add(edge->ptBounds);
		}
	}
	OP_ASSERT(linkBounds.isFinite());
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
		edge->setWhich(!edge->which());
		edge = edge->nextEdge;
	}
	std::swap(edge->priorEdge, edge->nextEdge);
	edge->setWhich(!edge->which());
	edge->clearLastEdge();
	lastEdge = edge;
	return true;
}

void OpEdge::setNextEdge(OpEdge* edge) {
	if (nextEdge)
		nextEdge->priorEdge = nullptr;
	nextEdge = edge;
}

void OpEdge::setPointBounds() {
	ptBounds.set(startPt(), endPt());
}

void OpEdge::setPriorEdge(OpEdge* edge) {
	if (priorEdge)
		priorEdge->nextEdge = nullptr;
	priorEdge = edge;
}

void OpEdge::setUnsortable(Unsortable unsortable) {  // setter exists so breakpoints can be set
	isUnsortable = unsortable;
}

const OpCurve& OpEdge::setVertical(const LinePts& pts, MatchEnds match) {
	if (!upright_impl.pts[0].isFinite() ||  // !!! needed by CMake build; don't know why ...
		upright_impl.pts[0] != pts.pts[0] || upright_impl.pts[1] != pts.pts[1]) {
		upright_impl = pts;
		vertical_impl = curve.toVertical(pts, match);
	}
	return vertical_impl;
}

void OpEdge::setWhich(EdgeMatch m) {
	whichEnd_impl = m;
}

// use already computed points stored in edge
void OpEdge::subDivide(OpPoint startPoint, OpPoint endPoint) {
	id = segment->nextID();
	curve = segment->c.subDivide(OpPtT(startPoint, startT), OpPtT(endPoint, endT));
	setPointBounds();
	calcCenterT();
	if (curve.isLine()) {
		center.t = OpMath::Interp(startT, endT, .5);
		center.pt = ptBounds.center();
	}
 	if (startPoint == endPoint) {
//		OP_ASSERT(0);	// triggered by fuzz763_9
		setDisabled(OP_LINE_FILE_NPARAMS());
	}
}

CalcFail OpEdge::subIfDL(Axis axis, float edgeInsideT, OpWinding* sumWinding) {
	NormalDirection NdotR = normalDirection(axis, edgeInsideT);
	if (NormalDirection::downLeft == NdotR)
		sumWinding->subtract(winding);
	else if (NormalDirection::upRight != NdotR)
		OP_DEBUG_FAIL(*this, CalcFail::fail);
	return CalcFail::none;
}

void OpEdge::setSum(const PathOpsV0Lib::Winding& w  OP_LINE_FILE_DEF()) {
	OP_ASSERT(!sum.contour);
	sum.contour = segment->contour;
	sum.w.data = contours()->allocateWinding(w.size);
	memcpy(sum.w.data, w.data, w.size);
	sum.w.size = w.size;
#if OP_DEBUG_MAKER
	debugSetSum = { fileName, lineNo };
#endif
}

// this is too complicated because
// edges in linked list have last edge set
//   and have link bounds set
// unlinking an edge in a linked list requires re-jiggering last edge, link bounds, and list membership
// since this doesn't have a OpJoiner, it is in the wrong context to do all this
// which begs the question, why unlink something in the linked list?
void OpEdge::unlink() {
	if (!inOutput) {
		OpEdge* linkStart = advanceToEnd(EdgeMatch::start);
		if (linkStart->inLinkups)
			return;
	}
	priorEdge = nullptr;
	nextEdge = nullptr;
	clearLastEdge();
	setWhich(EdgeMatch::start);  // !!! should this set to none?
}

bool OpEdge::unsectableSeen(EdgeMatch match) const {
	for (const EdgePal& pal : pals) {
		if (pal.reversed == (EdgeMatch::end == match) ? pal.edge->startSeen : pal.edge->endSeen)
			return true;
	}
	return false;
}

#if OP_DEBUG
bool OpEdge::debugFail() const {
    return segment->debugFail();
}
#endif

bool OpEdgeStorage::contains(OpIntersection* start, OpIntersection* end) const {
	for (size_t index = 0; index < used; index++) {
		const OpEdge* test = &storage[index];
		if (test->segment == start->segment && test->start() == start->ptT
				&& test->end() == end->ptT)
			return true;
	}
	if (!next)
		return false;
	return next->contains(start, end);
}

bool OpEdgeStorage::contains(OpPoint start, OpPoint end) const {
	for (size_t index = 0; index < used; index++) {
		const OpEdge* test = &storage[index];
		if (test->start().pt == start && test->end().pt == end)
			return true;
	}
    if (!next)
        return false;
    return next->contains(start, end);
}
