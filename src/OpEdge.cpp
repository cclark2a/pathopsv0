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
bool OpHulls::add(const OpPtT& ptT, SectType sectType, const OpEdge* opp) {			
	auto found = std::find_if(h.begin(), h.end(), [ptT](const HullSect& hull) {
		return ptT.soClose(hull.sect);
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
	if (!hull1Sect->pt.isNearly(oPtT->pt)) {
		const OpCurve& eCurve = edge.segment->c;
		OpVector eTangent = eCurve.tangent(hull1Sect->t);
		const OpCurve& oCurve = oEdge.segment->c;
		OpVector oTangent = oCurve.tangent(oPtT->t);
		OpLine eLine(hull1Sect->pt, hull1Sect->pt + eTangent);
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
	if (!hull1Sect.isNearly(hull2Sect))
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
		if (!hull1Sect.isNearly(ePtT))
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
					&& cc.checkSplit(edge.start.t, h[index + 1].sect.t, which, h[index].sect))
				goto tryAgain;
			++index;
			if ((SectType::midHull == h[index].type || SectType::controlHull == h[index].type)
					&& cc.checkSplit(h[index - 1].sect.t, edge.end.t, which, h[index].sect))
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

EdgeDistance::EdgeDistance(OpEdge* e, float c, float tIn, bool r)
	: edge(e)
	, cept(c)
	, edgeInsideT(tIn)
	, reversed(r) {
}

OpEdge::OpEdge(const OpEdge* edge, const OpPtT& newPtT, NewEdge isLeftRight  
		OP_DEBUG_PARAMS(EdgeMaker maker, int line, std::string file))
	: OpEdge() {
	segment = edge->segment;
	start = NewEdge::isLeft == isLeftRight ? edge->start : newPtT;
	end = NewEdge::isLeft == isLeftRight ? newPtT : edge->end;
#if 0
	if (edge->curvySet && edge->curvy <= OP_CURVACIOUS_LIMIT) {
		curvySet = true;
		curvy = edge->curvy * .5;  // !!! bogus, but shouldn't matter
	}
#endif
#if OP_DEBUG
	debugMaker = maker;
	debugSetMaker = { file, line };
	debugParentID = edge->id;
#endif	
	complete();
}

// called after winding has been modified by other coincident edges
OpEdge::OpEdge(const OpEdge* edge, const OpPtT& s, const OpPtT& e  
		OP_DEBUG_PARAMS(EdgeMaker maker, int line, std::string file))
	: OpEdge() {
	segment = edge->segment;
	start = s;
	end = e;
#if OP_DEBUG
	debugMaker = maker;
	debugSetMaker = { file, line };
	debugParentID = edge->id;
#endif	
	complete();
}

OpEdge::OpEdge(const OpEdge* edge, float t1, float t2
		OP_DEBUG_PARAMS(EdgeMaker maker, int line, std::string file))
	: OpEdge() {
	segment = edge->segment;
	start = { segment->c.ptAtT(t1), t1 };
	end = { segment->c.ptAtT(t2), t2 };
#if OP_DEBUG
	debugMaker = maker;
	debugSetMaker = { file, line };
	debugParentID = edge->id;
#endif	
	complete();
}

CalcFail OpEdge::addIfUR(Axis axis, float edgeInsideT, OpWinding* sumWinding) {
	NormalDirection NdotR = normalDirection(axis, edgeInsideT);
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
CalcFail OpEdge::addSub(Axis axis, float edgeInsideT, OpWinding* sumWinding) {
	NormalDirection NdotR = normalDirection(axis, edgeInsideT);
	if (NormalDirection::upRight == NdotR)
		*sumWinding += winding;
	else if (NormalDirection::downLeft == NdotR)
		*sumWinding -= winding;
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
		disabled = true;
	if (disabled || unsortable)
		return;
	OpWinding su = sum;
	OpWinding wi = winding;
	OpContours* contours = this->contours();
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
			windZero = su.right() || !su.left() ? WindZero::zero : WindZero::nonZero;
		break;
	case OpOperator::Intersect:
		keep = bothFlipped ? left == right : WindState::zero != left && WindState::zero != right;
		if (keep)
			windZero = !su.left() || !su.right() ? WindZero::zero : WindZero::nonZero;
		break;
	case OpOperator::Union:
		keep = bothFlipped ? left == right : WindState::one != left && WindState::one != right;
		if (keep)
			windZero = !su.left() && !su.right() ? WindZero::zero : WindZero::nonZero;
		break;
	case OpOperator::ExclusiveOr:
		keep = !bothFlipped;
		if (keep)
			windZero = !su.left() == !su.right() ? WindZero::zero : WindZero::nonZero;
		break;
	case OpOperator::ReverseSubtract:
		keep = bothFlipped ? left != right : WindState::zero == left || WindState::one == right;
		if (keep)
			windZero = su.left() || !su.right() ? WindZero::zero : WindZero::nonZero;
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
void OpEdge::calcCenterT() {
	const OpRect& r = ptBounds;
	Axis axis = r.largerAxis();
	float middle = OpMath::Average(r.ltChoice(axis), r.rbChoice(axis));
	const OpCurve& segCurve = segment->c;
	float t = segCurve.center(axis, middle);
	if (OpMath::IsNaN(t)) {
// while this should be disabled eventually, it must be visible to influence winding calc
// other edges' rays that hit this should also be disabled and marked ray fail
//		setDisabled(OP_DEBUG_CODE(ZeroReason::centerNaN));
		OP_DEBUG_CODE(debugZero = ZeroReason::centerNaN);
		OP_DEBUG_CODE(center = { OpPoint(SetToNaN::dummy), OpNaN } );
		centerless = true;  // !!! is this redundant with ray fail?
		rayFail = EdgeFail::center;
		return;
	}
	if (start.t >= t || t >= end.t)
		t = OpMath::Average(start.t, end.t);
	center.t = t;
	center.pt = segCurve.ptAtT(t);
	center.pt.pin(ptBounds);  // required by pentrek6
	OP_ASSERT(OpMath::Between(ptBounds.left, center.pt.x, ptBounds.right));
	OP_ASSERT(OpMath::Between(ptBounds.top, center.pt.y, ptBounds.bottom));
}

void OpEdge::clearActiveAndPals(ZeroReason reason) {
	setActive(false);
    for (auto& pal : pals) {
		if (ZeroReason::none != reason) {
			if (!pal.edge->pals.size())
				continue;  // !!! hack ?
			pal.edge->setActive(false);
			pal.edge->setDisabled(OP_DEBUG_CODE(reason));
		}
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

void OpEdge::complete() {
	winding.setWind(segment->winding.left(), segment->winding.right());
	subDivide();	// uses already computed points stored in edge
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

#if 0
// Guess if curve is nearly a line by comparing distance between ends and mid t pt to end line.
// Note that the rotated line is not square, so distances are sloppy as well.
float OpEdge::curviness() {
	if (curvySet)
		return curvy;
	curvySet = true;
	const OpCurve& rotated = setVertical();
	float length = fabsf(rotated.lastPt().y);
	OpPoint rotatedCenter = rotated.pts[rotated.pointCount()];
	curvy = OpMath::FloatDivide(fabsf(rotatedCenter.x), length);
	return curvy;
}
#endif

OpIntersection* OpEdge::findSect(EdgeMatch match) {
	OpPoint pt = whichPtT(match).pt;
	auto& i = segment->sects.i;
    auto found = std::find_if(i.begin(), i.end(), [pt](auto sect) { return sect->ptT.pt == pt; });
	OP_ASSERT(found != i.end());
	return *found;
}

OpPtT OpEdge::findT(Axis axis, float oppXY) const {
	OpPtT found;
	float startXY = start.pt.choice(axis);
	float endXY = end.pt.choice(axis);
	if (oppXY == startXY)
		found = start;
	else if (oppXY == endXY)
		found = end;
	else {
		found.pt = OpPoint(SetToNaN::dummy);
		found.t = segment->findAxisT(axis, start.t, end.t, oppXY);
		if (OpMath::IsNaN(found.t))
			found = (oppXY < startXY) == (startXY < endXY) ? start : end;
	}
	return found;
}

// this compares against float epsilon instead of zero
// when comparing against a line, an edge close to zero can fall into denormalized numbers,
//   causing the calling subdivision to continue for way too long. Using epsilon as a stopgap
//   avoids this. The alternative would be to change the math to disallow denormalized numbers
bool OpEdge::isLinear() {
	if (lineSet)
		return isLine_impl;
	lineSet = true;
	return (isLine_impl = curve.isLinear());
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
		oppEdge->setWhich(!match);
	else {
		OP_ASSERT(edgePt == oppEdge->end.pt);
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
            bool startMatch = firstPt == unsectable->start.pt
                    && (EdgeMatch::start == unsectable->which() ? !unsectable->priorEdge :
                    !unsectable->nextEdge);
            bool endMatch = firstPt == unsectable->end.pt
                    && (EdgeMatch::end == unsectable->which() ? !unsectable->priorEdge :
                    !unsectable->nextEdge);
			if (!startMatch && !endMatch)
				return false;
			if (AllowClose::no == allowClose && isDupe(unsectable))
				return false;
			if (unsectable->pals.size() && AllowPals::no == allowPals) {
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
	clearActiveAndPals(ZeroReason::addedPalToOutput);
	inLinkups = false;
    inOutput = true;
    return nextEdge;
}

// !!! note that t value is 0 to 1 within edge (not normalized to segment t)
NormalDirection OpEdge::normalDirection(Axis axis, float edgeInsideT) {
	return curve.normalDirection(axis, edgeInsideT);
}

// if there is another path already output, and it is first found in this ray,
// check to see if the tangent directions are opposite. If they aren't, reverse
// this edge's links before sending it to the host graphics engine
void OpEdge::output(OpOutPath& path, bool closed) {
    const OpEdge* firstEdge = closed ? this : nullptr;
    OpEdge* edge = this;
	bool reverse = false;
	// returns true if reverse/no reverse criteria found
	auto test = [&reverse](const EdgeDistance* outer, const EdgeDistance* inner) {
		if (!outer->edge->inOutput && !outer->edge->inLinkups)
			return false;
		// reverse iff normal direction of inner and outer match and outer normal points to nonzero
		OpEdge* oEdge = outer->edge;
		Axis axis = oEdge->ray.axis;
		NormalDirection oNormal = oEdge->normalDirection(axis, outer->edgeInsideT);
		if ((oEdge->windZero == WindZero::zero) == (NormalDirection::upRight == oNormal))
			return true;  // don't reverse if outer normal in direction of inner points to zero
		OpEdge* iEdge = inner->edge;
		OP_ASSERT(!iEdge->inOutput);
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
		OP_ASSERT(!edge->inOutput);
		unsigned index;
		const EdgeDistance* inner;
		for (index = 0; index < edge->ray.distances.size(); ++index) {
			inner = &edge->ray.distances[index];
			if (inner->edge == edge)
				break;
		}
		OP_ASSERT(!index || index < edge->ray.distances.size());
		if (index == 0)  // if nothing to its left, don't reverse
			break;
		const EdgeDistance* outer = &edge->ray.distances[index - 1];
		if (test(outer, inner))
			break;
		edge = edge->nextEdge;
    } while (firstEdge != edge);
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
	bool first = true;
    do {
		OP_DEBUG_CODE(edge->debugOutPath = path.debugID);
		OpEdge* next = edge->nextOut();
		OpCurve copy = edge->curve;
		if (EdgeMatch::end == edge->which())
			copy.reverse();
		if (!copy.output(path, first, firstEdge == next))
			break;
		first = false;
        edge = next;
    } while (firstEdge != edge);
	OP_DEBUG_CODE(path.debugNextID(this));
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

const OpRect& OpEdge::closeBounds() {
	// close bounds holds point bounds outset by 'close' fudge factor
	if (linkBounds.isSet())
		return linkBounds;
	linkBounds = ptBounds;
	return linkBounds.outsetClose();
}

bool OpEdge::isClose() {
	if (closeSet)
		return isClose_impl;
	closeSet = true;
	if (ccStart || ccEnd) {
		OP_ASSERT(!isClose_impl);
		return false;
	}
	return isClose_impl = start.soClose(end);
}

void OpEdge::setCurveCenter() {
	curve.pts[curve.pointCount()] = center.pt;
	curve.centerPt = true;
}

// should be inlined. Out of line for ease of setting debugging breakpoints
void OpEdge::setDisabled(OP_DEBUG_CODE(ZeroReason reason)) {
	disabled = true; 
	OP_DEBUG_CODE(debugZero = reason); 
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
	ptBounds.set(start.pt, end.pt);
	// this check can fail; control points can have some error so they lie just outside bounds
#if 0 // OP_DEBUG
	OpPointBounds copy = ptBounds;
	for (int index = 1; index < curve_impl.pointCount() - 1; ++index)
		ptBounds.add(curve_impl.pts[index]);
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

const OpCurve& OpEdge::setVertical(const LinePts& pts) {
	if (upright_impl.pts[0] != pts.pts[0] || upright_impl.pts[1] != pts.pts[1]) {
		upright_impl = pts;
		vertical_impl = curve.toVertical(pts);
	}
	return vertical_impl;
}

void OpEdge::setWhich(EdgeMatch m) {
	whichEnd_impl = m;
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
	first->setLastEdge();
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
			if (innerBounds.perimeter() < outerBounds.perimeter())
				continue;
			if (innerBounds.perimeter() == outerBounds.perimeter()
					&& inner.edge->ptBounds.perimeter() <= outer.edge->ptBounds.perimeter())
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
	auto ctrlPtsEqualEnds = [this]() {
		for (int index = 1; index < segment->c.pointCount() - 1; ++index) {
			if (curve.pts[index] != start.pt && curve.pts[index] != end.pt)
				return false;
		}
		return true;
	};
	id = segment->nextID();
	curve = segment->c.subDivide(start, end);
	setPointBounds();
	calcCenterT();
	if (OpType::line == segment->c.type || (!ptBounds.height() ^ !ptBounds.width())
			|| ctrlPtsEqualEnds()) {
		isLine_impl = true;
		lineSet = true;
		exactLine = true;
		center.t = OpMath::Interp(start.t, end.t, .5);
		center.pt = ptBounds.center();
	}
 	if (start.pt == end.pt) {
//		OP_ASSERT(0);	// triggered by fuzz763_9
		setDisabled(OP_DEBUG_CODE(ZeroReason::isPoint));
	}
}

CalcFail OpEdge::subIfDL(Axis axis, float edgeInsideT, OpWinding* sumWinding) {
	NormalDirection NdotR = normalDirection(axis, edgeInsideT);
	if (NormalDirection::downLeft == NdotR)
		*sumWinding -= winding;
	else if (NormalDirection::upRight != NdotR)
		OP_DEBUG_FAIL(*this, CalcFail::fail);
	return CalcFail::none;
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
	visited = false;
}

#if OP_DEBUG
bool OpEdge::debugFail() const {
    return segment->debugFail();
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

void OpWinding::setSum(OpWinding winding, const OpContours* contours) {
	OP_ASSERT(WindingType::uninitialized == debugType);
	OP_ASSERT(WindingType::temp == winding.debugType);
	OP_DEBUG_CODE(debugType = WindingType::sum);
	left_impl = winding.left() & contours->leftFillTypeMask();
	right_impl = winding.right() & contours->rightFillTypeMask();
}
