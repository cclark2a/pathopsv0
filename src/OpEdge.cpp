// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpEdge.h"
#include "OpCurveCurve.h"

#if OP_DEBUG
#include "OpJoiner.h"
#include "PathOps.h"
#endif

// don't add if points are close
// prefer end if types are different
// return true if close enough points are ctrl + mid
bool OpHulls::add(const OpPtT& ptT, OpVector threshold, const EdgeDist& dist, 
			SectType sectType, const OpEdge* opp) {	
	bool foundNear = false;
	bool nearEnd = false;
	for (size_t index = h.size(); index-- != 0; ) {
		const HullSect& hSect = h[index];
		bool near = ptT.isNearly(hSect.sect, threshold);
		if (!near)
			continue;
		foundNear = true;
		if (SectType::endHull == sectType)
			h.erase(h.begin() + index);
		else
			nearEnd |= SectType::endHull != hSect.type && sectType != hSect.type;
	}
	if (!foundNear || SectType::endHull == sectType) {
		h.emplace_back(opp, ptT, sectType);
		if (SectType::endHull == sectType && dist.isSet())
			h.back().oppDist = dist;
		return false;
	}
	return nearEnd;
}

#if 0
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
			float newOPtT = oEdge.segment->c.findValidT(0, 1, sectPt);
			if (OpMath::IsNaN(newOPtT))
				return false;
			*oPtT = OpPtT(sectPt, newOPtT);
			*hull1Sect = OpPtT(sectPt, ePtT.t);
		}
	}
	return true;
}
#endif

#if OP_DEBUG
bool OpHulls::debugSectCandidates(int index, const OpEdge& edge) const {
	const HullSect& hullStart = h[index - 1];
	const HullSect& hullEnd = h[index];
	const OpPtT& hull1Sect = hullStart.sect;
	const OpPtT& hull2Sect = hullEnd.sect;
	if (!hull1Sect.isNearly(hull2Sect, edge.segment->threshold()))
		return false;
#if 0
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
#endif
	return true;	// code coverage says this is never reached...
}
#endif

// checks to catch infinite loops if caller data isn't resolvable
bool OpHulls::nudgeDeleted(const OpEdge& edge, OpCurveCurve& cc, CurveRef which) {
	int safetyCount = 16;
	PathOpsV0Lib::CurveCount nudgeFunc = edge.context()->contextCallbacks.hullNudgeFuncPtr;
	if (nudgeFunc)
		safetyCount = (*nudgeFunc)(edge.curve.c);
	do {
		sort(false);
		for (size_t index = 0; index + 1 < h.size(); ) {
			HullSect& hull = h[index];
			HullSect& next = h[++index];
			OpPtT& sectToNudge = hull.sect;
			OpPtT& nextToNudge = next.sect;
			// while hull sect is in a deleted bounds, bump its t and recompute
			if ((SectType::midHull == hull.type || SectType::controlHull == hull.type)
					&& cc.checkSplit(edge.startT, nextToNudge.t, which, sectToNudge))
				goto tryAgain;
			if ((SectType::midHull == next.type || SectType::controlHull == next.type)
					&& cc.checkSplit(sectToNudge.t, edge.endT, which, nextToNudge))
				goto tryAgain;
			OP_ASSERT(sectToNudge.t <= nextToNudge.t);
		}
		return true;
tryAgain:
		;
	} while (--safetyCount);
	OP_ASSERT(0);  // !!! add test that triggers this, if one exists -- treat as error?
	return false;
}

void OpHulls::sort(bool useSmall) {
	std::sort(h.begin(), h.end(), [useSmall](const HullSect& s1, const HullSect& s2) {
		return useSmall ? s1.sect.t > s2.sect.t : s1.sect.t < s2.sect.t;
	});
}

Distance::Distance(OpEdge* e, float c, float tIn)
	: edge(e)
	, cept(c)
	, edgeInsideT(tIn)
	, reversed(false) {
	OP_DEBUG_CODE(debugID = e->context()->nextID());
}

#if 0
EdgePal::EdgePal(OpEdge* e, float c, float tIn)
	: edge(e)
	, cept(c)
	, edgeInsideT(tIn)
	, unsectID(0) {
}
#endif

EdgePal::EdgePal(OpEdge* e, int uID, bool r)
	: edge(e)
	, unsectID(uID)
	, reversed(r) {
}

#if OP_DEBUG_DUMP
Distance::Distance()
	: edge(nullptr)
	, rayOrder((RayOrder) -1)
	, cept(OpDebugNaN)
	, edgeInsideT(OpDebugNaN)	
	, dependent(-1)
	, reversed(-1)
	, over(-1)
	, debugID(-1) {
}

EdgePal::EdgePal()
	: edge(nullptr)
	, unsectID(0)
	, reversed(-1) {
}
#endif

OpPoint EdgePal::matchPt(EdgeMatch m) const {
	return edge->ptT(reversed ? !m : m).pt;
}

EdgeOutput::EdgeOutput(OpContext* context, OpEdge* edge, bool isLoop) {
    PathOpsV0Lib::ContextCount maxLoops = context->contextCallbacks.maxLoopsFuncPtr;
    int safetyCounter = maxLoops ? (*maxLoops)((ContextPtr) context) : 0;
	while (edge->output(isLoop) && --safetyCounter >= 0)
        OP_ASSERT(safetyCounter >= 0);
}

// called when creating edge for curve curve intersection building
OpEdge::OpEdge(OpSegment* s  OP_LINE_FILE_ARGS())
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

// called when creating edges for curve curve intersection when segments partially intersect
OpEdge::OpEdge(OpSegment* s, const OpPtT& start, const OpPtT& end  OP_LINE_FILE_ARGS())
	: OpEdge() {
	OP_LINE_FILE_SET(debugSetMaker);
	OP_DEBUG_CODE(debugParentID = s->id);
	segment = s;
	complete(start, end);
}

// called when creating edge from intersection pairs
OpEdge::OpEdge(OpIntersection* sectStart, OpIntersection* sectEnd  OP_LINE_FILE_ARGS())
	: OpEdge() {
	OP_LINE_FILE_SET(debugSetMaker);
	OP_DEBUG_CODE(debugParentID = sectStart->id);
	segment = sectStart->segment;
	complete(sectStart->ptT, sectEnd->ptT);
	curve.start = sectStart->ptT.pt;
	curve.end = sectEnd->ptT.pt;
#if 0  // !!! replaced by is small ?
	auto altEnd = [this](OpPoint edgeEnd, OpPoint altEnd) {
		if (edgeEnd == altEnd)
			return false;
		PathOpsV0Lib::CurveConst altEndFuncPtr = 
				context()->callback(curve.c.type).maxAlternateEndFuncPtr;
		float maxAltEnd = altEndFuncPtr ? (*altEndFuncPtr)(lastEdge->curve.c) : 4.0f;
		float vLen = (edgeEnd - altEnd).length();
		float edgeLen = (curve.lastPt() - curve.firstPt()).length();
		float ratio = edgeLen / vLen;
		return ratio < maxAltEnd;
	};
	alternateEnd = altEnd(curve.firstPt(), curve.c.data->start) 
			|| altEnd(curve.lastPt(), curve.c.data->end);
	if (alternateEnd)
		setUnsortable(Unsortable::alternateEnd);
#endif
}

// called when creating filler; edge that closes small gaps
// points may not be on segment, so don't guess at t values
OpEdge::OpEdge(OpContext* context, const OpPoint start, const OpPoint end  OP_LINE_FILE_ARGS())
	: OpEdge() {
	OP_LINE_FILE_SET(debugSetMaker);
	OP_DEBUG_CODE(debugParentID = 0);
//	startSect = -1;
//	endSect = -1;
//	OP_ASSERT(start.t < end.t);
//	startT = start.t;
//	endT = end.t;
	id = context->nextID();
	PathOpsV0Lib::CurveData lineData { start, end };
	PathOpsV0Lib::Curve lineCurve { (ContextPtr) context, &lineData, sizeof(lineData), 
            PathOpsV0Lib::degenerateLine };
	curve = OpCurve(lineCurve, Rotated::no);
	curve.isLineSet = true;
	curve.isLineResult = true;
//	setPointBounds();
//	center.t = OpMath::Interp(startT, endT, .5);
	center.t = OpNaN;
	center.pt = bounds().center();
	setDisabled(OP_LINE_FILE_NPARGS());
	setUnsortable(Unsortable::filler);
}

// called from curve curve when splitting edges
OpEdge::OpEdge(const OpEdge* edge, const OpPtT& newPtT, NewEdge isLeftRight  OP_LINE_FILE_ARGS())
	: OpEdge() {
	OP_LINE_FILE_SET(debugSetMaker);
	OP_DEBUG_CODE(debugParentID = edge->id);
	segment = edge->segment;
	if (NewEdge::isLeft == isLeftRight) {
		startDist = edge->startDist;
		complete(OpPtT { edge->curve.firstPt(), edge->startT }, newPtT);
	} else {
        OP_ASSERT(NewEdge::isRight == isLeftRight);
		endDist = edge->endDist;
		complete(newPtT, OpPtT { edge->curve.lastPt(), edge->endT });
	}
}

// called by curve curve's snip range
OpEdge::OpEdge(const OpEdge* edge, const OpPtT& s, const OpPtT& e  OP_LINE_FILE_ARGS())
	: OpEdge() {
	OP_LINE_FILE_SET(debugSetMaker);
	OP_DEBUG_CODE(debugParentID = edge->id);
	segment = edge->segment;
	if (edge->startT == s.t)
		startDist = edge->startDist;
	if (edge->endT == e.t)
		endDist = edge->endDist;
	complete(s, e);
}

#if 0  // !!! disallowed : it tosses point that has been adjusted and recomputes from t
// called by curve curve when constructing edge from hull intersections
OpEdge::OpEdge(OpSegment* seg, float t1, float t2  OP_LINE_FILE_ARGS())
	: OpEdge() {
	OP_LINE_FILE_SET(debugSetMaker);
	OP_DEBUG_CODE(debugParentID = seg->id);
	segment = seg;
	startT = t1;
	endT = t2;
	startDist = OpNaN;	// !!! probably should always initialize this in default constructor
	endDist = OpNaN;
	complete(segment->c.ptAtT(t1), segment->c.ptAtT(t2));
}
#endif

#if OP_DEBUG_VALIDATE
OpEdge::~OpEdge() {
//	OP_ASSERT(!segment || !segment->contour->context->debugJoiner);
	id = -1;
}
#endif 

CalcFail OpEdge::addIfUR(Axis axis, float edgeInsideT, OpWinding* sumWinding) const {
	NormalDirection NdotR = normalDirection(axis, edgeInsideT);
	if (NormalDirection::upRight == NdotR)
		sumWinding->add(winding);
	else if (NormalDirection::downLeft != NdotR)
		return CalcFail::fail; // e.g., may underflow if edge is too small
	return CalcFail::none;
}

void OpEdge::addPal(OpEdge* edge, int uID, bool reversed) {
	auto palIter = std::find_if(pals.begin(), pals.end(), 
			[edge](const auto& pal) { return edge == pal.edge; });
	if (pals.end() == palIter) {
		pals.emplace_back(edge, uID, reversed);
		palIter = pals.end() - 1;
		segment->hasPals = true;
		segment->contour->hasPals = true;
		unsummable = true;
	}
	auto& oPals = edge->pals;
	auto distIter = std::find_if(oPals.begin(), oPals.end(), 
			[this](auto oPal) { return oPal.edge == this; });
	if (!uID && oPals.end() != distIter)
		uID = distIter->unsectID;
	if (!uID)
		uID = context()->nextID();
	palIter->unsectID = uID;
	if (oPals.end() != distIter)
		distIter->unsectID = uID;
}

// given an intersecting ray and edge t, add or subtract edge winding to sum winding
// but don't change edge's sum, since an unsectable edge does not allow that accumulation
CalcFail OpEdge::addSub(Axis axis, float edgeInsideT, OpWinding* sumWinding) const {
//	OpWinding adjust(WindingUninitialized::dummy);
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

WindingCondition OpEdge::apply() {
	if (centerless)
		setDisabled(OP_LINE_FILE_NPARGS());
	if (!isSortable() && !isKept())  // e.g., if pals cumulatively don't cross winding zero, disable
		setDisabled(OP_LINE_FILE_NPARGS());
	if (disabled || !isSortable() || !sum.isSet())
		return 0;
    OpContext* ctxt = context();
	PathOpsV0Lib::WindKeep keep = winding.keep(sum);
#if 0  // enable to see how windings are passed to caller
	DebugLevel l = DebugLevel::normal;
	DebugBase b = DebugBase::dec;
	std::string keepNames[] = { "Discard", "End", "Start" };
	OpDebugOut("edge:" + STR(id) + " contour:" + STR(segment->contour->id) 
			+ " winding:" + winding.debugDump(l, b) + " sum:" + sum.debugDump(l, b) 
			+ " keep:" + keepNames[(int) keep] 
			+ (startPt().y < endPt().y ? " down" : " up") + "\n");
#endif
    bool affectsWinding = winding.isWound();
	switch (keep) {
		case PathOpsV0Lib::WindKeep::Discard:
			setDisabled(OP_LINE_FILE_NPARGS());
			break;
		case PathOpsV0Lib::WindKeep::End:
            if (affectsWinding)
			    windZero = WindZero::zero;
			break;
		case PathOpsV0Lib::WindKeep::Start:
			if (affectsWinding)
                windZero = WindZero::nonZero;
			break;
        default:
        	OP_ASSERT(0);
	}
    // for contains-like, allow short circuiting if any-kept or any-discarded is expected and met
    if (PathOpsV0Lib::WindingShort windingShort = ctxt->windingCallbacks.windingShortFuncPtr)
        if (WindingCondition condition = (WindingCondition) (*windingShort)(curve.c.context, keep))
            return condition;
    ctxt->allDiscarded &= PathOpsV0Lib::WindKeep::Discard == keep;
    ctxt->allKept &= PathOpsV0Lib::WindKeep::Discard != keep;
    return 0;
}

// old thinking:
// for center t (and center t only), use edge geometry since center is on edge even if there is error
// and using segment instead might return more than one intersection
// new thinking:
// segments are now broken monotonically when they are built, so they should not return more than
// one intersection anymore often than edges. 
void OpEdge::calcCenterT() {
	const OpCurve& segCurve = segment->c;
	OpRect r = bounds();
	Axis axis = r.largerAxis();
	float middle = OpMath::Average(r.ltChoice(axis), r.rbChoice(axis));
	float t = segCurve.center(axis, middle);
	if (OpMath::IsNaN(t)) {
// while this should be disabled eventually, it must be visible to influence winding calc
// other edges' rays that hit this should also be disabled and marked ray fail
//		setDisabled(OP_LINE_FILE_NARGS());
		OP_LINE_FILE_SET_IMMED(debugSetDisabled);
		centerless = true;
		return;
	}
	if (startT >= t || t >= endT)
		t = OpMath::Average(startT, endT);
	if (startT >= t || t >= endT) {
		OP_LINE_FILE_SET_IMMED(debugSetDisabled);
		OP_DEBUG_CODE(center = { OpPoint(SetToNaN::dummy), OpNaN } );
		centerless = true;
		return;
	}
	center.t = t;
	center.pt = segCurve.ptAtT(t);
	center.pt.pin(r);  // required by pentrek6
	OP_ASSERT(OpMath::Between(r.left, center.pt.x, r.right));
	OP_ASSERT(OpMath::Between(r.top, center.pt.y, r.bottom));
}

void OpEdge::clearActiveAndPals(OP_LINE_FILE_NP_ARGS()) {
	setActive(false);
	for (auto& pal : pals) {
		if (!pal.edge->hasPals())
			continue;  // !!! hack ?
		pal.edge->setActive(false);
		pal.edge->setDisabled(OP_LINE_FILE_NP_CARGS());
	}
	clearLast(/* InOutput::no */);
}

void OpEdge::clearLast(/* InOutput inOut */) {
	if (!lastEdge)
		return;
	lastEdge->segment->contour->removeLast(this /*, inOut*/);
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

std::vector<OpPoint> OpEdge::collectMatch(EdgeMatch m, float* t) const {
	std::vector<OpPoint> pts;
	OpPtT firstPtT = whichSect(m);
	if (t)
		*t = firstPtT.t;
	OpPoint pt = firstPtT.pt;
	pts.push_back(pt);
	if (segment)
		segment->sects.collectMatchingPts(pt, pts);
	// edge which may not be set if edge is disabled, but gathered here from joiner disabled pass (testCubics56146)
	bool matchStart = EdgeMatch::none == which() ? EdgeMatch::start == m 
			: EdgeMatch::start == which(m);
	if (matchStart && pts.end() == std::find(pts.begin(), pts.end(), curve.c.data->start))
		pts.push_back(curve.c.data->start);
	if (!matchStart && pts.end() == std::find(pts.begin(), pts.end(), curve.c.data->end))
		pts.push_back(curve.c.data->end);
	return pts;
}

bool OpEdge::compareMatch(EdgeMatch m, OpEdge* opp, EdgeMatch oppM) const {
	std::vector<OpPoint> pts = collectMatch(m);
	std::vector<OpPoint> oppPts = opp->collectMatch(oppM);
	for (OpPoint pt : pts) {
		for (OpPoint oppPt : oppPts) {
			if (pt == oppPt)
				return true;
		}
	}
	return false;
}

void OpEdge::complete(OpPtT startPtT, OpPtT endPtT) {
	startT = startPtT.t;
	endT = endPtT.t;
	complete(startPtT.pt, endPtT.pt);  // compute exact points
}

void OpEdge::complete(OpPoint startPoint, OpPoint endPoint) {
	OP_DEBUG_VALIDATE_CODE(OP_ASSERT(!segment->contour->context->debugJoiner));
	OP_ASSERT(startT < endT);
	subDivide(startPoint, endPoint);	// uses already computed points stored in edge
	winding.setWind(segment->winding);
    PathOpsV0Lib::CurveConst smallFuncPtr = context()->callback(curve.c.type).smallTFuncPtr;
    float smallT = (smallFuncPtr ? (*smallFuncPtr)(curve.c) : 32.f) * OpEpsilon;
    smallTRange = endT - startT <= smallT;
}

OpContext* OpEdge::context() const {
	return segment->contour->context; 
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

// !!! should this be computed once and stored in edge's sum winding?
// if isn't sortable, check if edge plus pals can't cross zero winding regardless of how pals are ordered
// e.g. edge 482 is next to 435 but at winding === +3 so cumulative result is +1 regardless of order...
bool OpEdge::isKept() const {
	OP_ASSERT(!isSortable());
	if (pals.empty())
		return true;
	int edgeIndex = (int) ray.distances.size();
	while (ray.distances[--edgeIndex].edge != this) {
		OP_ASSERT(edgeIndex > 0);
	}
	int lastPal = edgeIndex;
	while (lastPal < (int) ray.distances.size() - 1) {
		OpEdge* nextEdge = ray.distances[lastPal + 1].edge;
		if (pals.end() == std::find_if(pals.begin(), pals.end(), [nextEdge](const EdgePal& edgePal) {
				return nextEdge == edgePal.edge; } ))
			break;
		++lastPal;
	}
	int firstPal = edgeIndex;
	while (firstPal > 0) {
		OpEdge* priorEdge = ray.distances[firstPal - 1].edge;
		if (pals.end() == std::find_if(pals.begin(), pals.end(), [priorEdge](const EdgePal& edgePal) {
				return priorEdge == edgePal.edge; } ))
			break;
		--firstPal;
	}
	int summedIndex = firstPal;
	OpEdge* computedSum = nullptr;
	while (summedIndex > 0) {
		computedSum = ray.distances[--summedIndex].edge;
		if (computedSum->isSummable() && computedSum->sum.isSet())
			break; 
		computedSum = nullptr;
	}
	OpWinding priorWinding(this, WindingSum::dummy);
	if (computedSum) {
		const Distance& sumDistance = ray.distances[summedIndex];
//		OP_ASSERT(!sumEdge->isUnsectable());
		if (computedSum->sum.isSet())
			priorWinding.w = computedSum->sum.copyData();
		else
			priorWinding.zero();
		computedSum->subIfDL(ray.axis, sumDistance.edgeInsideT, &priorWinding);
	}
	while (++summedIndex < firstPal) {
		const Distance& priorDist = ray.distances[summedIndex];
		OpEdge* priorEdge = priorDist.edge;
		priorEdge->addSub(ray.axis, priorDist.edgeInsideT, &priorWinding);
	}
	OpWinding sumWinding(this, WindingSum::dummy);
	sumWinding.w = priorWinding.copyData();
	bool discardAll = true;
	for (int palIndex = firstPal; palIndex <= lastPal; ++palIndex) {
		const Distance& dist = ray.distances[palIndex];
		dist.edge->addSub(ray.axis, dist.edgeInsideT, &sumWinding);
		discardAll &= PathOpsV0Lib::WindKeep::Discard == priorWinding.keep(sumWinding);
	}
	return !discardAll;
}

void OpEdge::linkToEdge(FoundEdge& found, EdgeMatch match) {
	OpEdge* oppEdge = found.edge;
	advanceToEnd(EdgeMatch::start)->clearLast();
	oppEdge->advanceToEnd(EdgeMatch::start)->clearLast();
//	OP_ASSERT(!oppEdge->hasLinkTo(match));  // !!! doesn't make sense -- opp match is unknown
	OP_ASSERT(oppEdge != this);
	const OpPoint edgePt = whichSect(match).pt;
	if (EdgeMatch::start == match) {
		OP_ASSERT(!priorEdge);
		setPriorEdge(oppEdge);
		OP_ASSERT(!oppEdge->nextEdge);
		oppEdge->setNextEdge(this);
	} else {
		OP_ASSERT(!nextEdge);
		setNextEdge(oppEdge);
		OP_ASSERT(!oppEdge->priorEdge);
		oppEdge->setPriorEdge(this);
	}
	if (edgePt == oppEdge->startPt())
		oppEdge->setWhich(!match);
	else if (edgePt == oppEdge->endPt())
		oppEdge->setWhich(match);
	else {
		// !!! for now, brute force check all matching possibilities
		bool startFoundMatch = compareMatch(match, found.edge, EdgeMatch::start);
		OP_DEBUG_CODE(bool endFoundMatch = compareMatch(match, found.edge, EdgeMatch::end));
		OP_ASSERT(startFoundMatch != endFoundMatch);
		oppEdge->setWhich(startFoundMatch ? !match : match);
	}
    std::vector<LoopCheck> edges;
	EdgesLoop edgesLoop = OpContour::IsLoop(edges, oppEdge, match);
	if (EdgesLoop::no == edgesLoop)
		updateLastEdge();
	OP_ASSERT(EdgesLoop::tail != edgesLoop);  // !!! if triggered, more code to write
}

float OpEdge::margin() const {
	PathOpsV0Lib::ContextCallbacks& cb = context()->contextCallbacks;
	float maxUnsectT = cb.maxUnsectableTFuncPtr ? cb.maxUnsectableTFuncPtr(curve.c) : 4.0f;
	return context()->threshold.choice(!ray.axis) * maxUnsectT;
}

// Find pals for unsectables created during curve/curve intersection. There should be at most
// two matching unsectable ids in the distances array. Mark between edges as well.
void OpEdge::markPals() {
	OP_ASSERT(hasPals());
	// edge is between one or more unsectableID ranges in intersections
	for (EdgePal& pal : pals) {
		for (auto& dist : ray.distances) {
			if (pal.edge == dist.edge)
				addPal(dist.edge, 0, dist.reversed);
		}
	}
}

// if there is another path already output, and it is first found in this ray,
// check to see if the tangent directions are opposite. If they aren't, reverse
// this edge's links before sending it to the host graphics engine
bool OpEdge::output(bool closed) {
	const OpEdge* firstEdge = closed ? this : nullptr;
	OpEdge* edge = this;
	bool reverse = false;
	bool abort = false;
	// returns true if reverse/no reverse criteria found
	// if all loop edges are unsectable, there may be no valid reverse criteria (testQuads5343280)
	auto test = [&reverse, &abort, this](const Distance* outer, const Distance* inner) {
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
		if (iEdge->inOutput && !iEdge->hasPals() && Unsortable::none == iEdge->unsortable) {  // defer dealing with this until we find an easier test case
			OpDebugOut(context()->debugData.testname + " !!! edge already output\n");
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
    if (!context()->windingCallbacks.windingWoundFuncPtr) {
	    do {
	    //	OP_ASSERT(!edge->inOutput);	// !!! cubic714074 triggers with very small edge, used twice
		    unsigned index;
		    const Distance* inner = nullptr;
		    for (index = 0; index < edge->ray.distances.size(); ++index) {
			    inner = &edge->ray.distances[index];
			    if (inner->edge == edge)
				    break;
		    }
		    OP_ASSERT(!index || index < edge->ray.distances.size());
		    if (index == 0)  // if nothing to its left, don't reverse
			    break;
		    const Distance* outer = &edge->ray.distances[index - 1];
		    if (test(outer, inner))
			    break;
		    edge = edge->nextEdge;
	    } while (edge && firstEdge != edge);  // may be closed and edge==null if no valid reverse
    }
	if (abort)
		return false;
	if (reverse) {
		if (priorEdge) {
			OP_ASSERT(debugIsLoop());
			setLast(this, priorEdge, InOutput::yes);
			lastEdge->nextEdge = nullptr;
			priorEdge = nullptr;
		}
		edge = lastEdge;
		edge->setLinkDirection(EdgeMatch::none, nullptr, InOutput::yes);
	} else
		edge = this;
	return edge->outputLinkedList();
}

// !!! this doesn't account for frames that may or may not form loops
// if frame, only add filler from last to first if gap can be accounted for by op curve start, end
bool OpEdge::outputLinkedList() {
	OpEdge* firstEdge = this;
	OpEdge* edge = this;
	bool result = true;
	bool closeLoop = context()->allowError(PathOpsV0Lib::ContextError::missing, &curve.c);
	do {
		OpEdge* next = edge->nextEdge;
		edge->outputLink(firstEdge, closeLoop);
		result &= !edge->inOutput;
		edge = next;
	} while (edge && edge != firstEdge);
	return result;  // if true, all edges are kept by caller; none in linked list are in output
}

// track if caller returns 'keep' 
// if so, mark edges accordingly and reuse to build next linked list
// !!! tracking must be used for cut or some other new frame fill, but unsure if it still works...
void OpEdge::outputLink(OpEdge* firstEdge, bool closeLoop) {
//	OpBreak(this, 41435);
	OpCurve copy(curve.c, Rotated::no);
	if (EdgeMatch::end == which())
		copy.reverse();
	bool last = !nextEdge || firstEdge == nextEdge;
	OpEdge* nextPtEdge = nextEdge ? nextEdge : firstEdge;
	OpPoint nextPt = nextPtEdge->whichCurvePt();
	bool addFiller = nextPt != copy.lastPt();
	PathOpsV0Lib::WindKeep keep = copy.output(winding.w, this == firstEdge, last && !addFiller 
            OP_DEBUG_RASTER_PARAMS(this));
    if (PathOpsV0Lib::WindKeep::Discard == keep) {
	    inOutput = true;
	    clearActiveAndPals(OP_LINE_FILE_NPARGS());
	    if (linkHead) {
		    segment->contour->removeLink(this);
	    }
	    inLinkups = false;
	    clearNextEdge();	    
    }
	if (addFiller && (!last || !closeLoop)) {
		OpEdge* filler = context()->addFiller(copy.lastPt(), nextPt, segment);
    	filler->curve.output(winding.w, false, last  OP_DEBUG_RASTER_PARAMS(filler));
	}
}

// in function to make setting breakpoints easier
// !!! if this is not inlined in release, do something more cleverer
void OpEdge::setActive(bool state) {
	active_impl = state;
}

// should be inlined. Out of line for ease of setting debugging breakpoints
void OpEdge::setDisabled(OP_LINE_FILE_NP_ARGS()) {
	disabled = true; 
	OP_LINE_FILE_SET(debugSetDisabled); 
}

void OpEdge::setLast(OpEdge* first, OpEdge* last, InOutput inOut) {
	OpContour* oldContour = lastEdge ? lastEdge->segment->contour : nullptr;
	OP_ASSERT(InOutput::yes == inOut || !first->lastEdge || first->lastEdge == last);
	OP_ASSERT(InOutput::yes == inOut || first == last || !last->lastEdge);
//	OP_ASSERT(!last->nextEdge);
	OpContour* newContour = last->segment->contour;
	if (first->lastEdge && first != this) {
		newContour->removeLast(first /*, inOut */);
        first->setLastEdge(nullptr);
    }
	bool updateLast = !oldContour || oldContour != newContour;
	if (updateLast && oldContour)
		oldContour->removeLast(this /*, inOut */);
	setLastEdge(last);
	if (updateLast && InOutput::no == inOut)
		newContour->addLast(this);
	setLinkBounds();
}

void OpEdge::setLastEdge(OpEdge* last) {  // !!! to allow setting breakpoints at runtime
	lastEdge = last;
}

// this sets up the edge linked list to be suitable for joining another linked list
// the edits are nondestructive 
bool OpEdge::setLastLink(EdgeMatch match) {
	if (!priorEdge && !nextEdge) {
		setLastEdge(this);
		setWhich(match);
		return false;
	} 
	if (!lastEdge)
		return setLinkDirection(EdgeMatch::end, nullptr, InOutput::no);
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
		OP_ASSERT(bounds().isFinite());
		linkBounds = bounds();
		const OpEdge* edge = this;
		while (edge != lastEdge) {
			edge = edge->nextEdge;
			OP_ASSERT(edge->bounds().isFinite());
			linkBounds.add(edge->bounds());
		}
	}
	OP_ASSERT(linkBounds.isFinite());
	return linkBounds;
}

// reverses link walk
// if the edge chain is one long, match is required to know if it must be reversed
bool OpEdge::setLinkDirection(EdgeMatch match, std::vector<OpEdge*>* linkErasures, InOutput inOut) {
	if (EdgeMatch::start == match && lastEdge)	// !!! this requires that lastEdge only be at start
		return false;
	OpEdge* edge = this;
	while (edge->priorEdge) {
		std::swap(edge->priorEdge, edge->nextEdge);
		if (EdgeMatch::none == edge->which())
			edge->setWhich(EdgeMatch::start);
		edge->setWhich(!edge->which());
		edge = edge->nextEdge;
	}
	std::swap(edge->priorEdge, edge->nextEdge);
	edge->setWhich(!edge->which());
	edge->clearLast(/* inOut */);
	setLastEdge(edge);
	if (edge->linkHead && linkErasures) {
#if OP_DEBUG_VALIDATE
		edge->debugScheduledForErasure = true;
#endif
		linkErasures->push_back(edge);
	}
	if (InOutput::no == inOut)
		edge->segment->contour->addLast(this);
	return true;
}

void OpEdge::setNextEdge(OpEdge* edge) {
	OP_ASSERT(this != edge);
	if (nextEdge)
		nextEdge->priorEdge = nullptr;
	nextEdge = edge;
}

void OpEdge::setPriorEdge(OpEdge* edge) {
	OP_ASSERT(this != edge);
	if (priorEdge)
		priorEdge->nextEdge = nullptr;
	priorEdge = edge;
	OP_DEBUG_VALIDATE_CODE(if (edge) debugPriorID = edge->id);
}

void OpEdge::setUnsortable(Unsortable u) {  // setter exists so breakpoints can be set
	unsortable = u;
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
	curve = segment->c.subDivide(startT, endT);
//	curve.adjust(startPoint, endPoint);  // move ends and adjust controls to aligned points
//	setPointBounds();
	calcCenterT();
#if 0  // if curve is near-linear cubic, t value for edge is wrong
	if (curve.isLine()) {
		center.t = OpMath::Interp(startT, endT, .5);
		center.pt = ptBounds.center();
	}
#endif
 	if (startPoint == endPoint || curve.firstPt() == curve.lastPt()) {
//		OP_ASSERT(0);	// triggered by fuzz763_9 ; triggered by loop63275
		setDisabled(OP_LINE_FILE_NPARGS());
	}
}

CalcFail OpEdge::subIfDL(Axis axis, float edgeInsideT, OpWinding* sumWinding) const {
	NormalDirection NdotR = normalDirection(axis, edgeInsideT);
	if (NormalDirection::downLeft == NdotR)
		sumWinding->subtract(winding);
	else if (NormalDirection::upRight != NdotR)
		OP_DEBUG_FAIL(*this, CalcFail::fail);
	return CalcFail::none;
}

void OpEdge::setSum(const OpWinding& w  OP_LINE_FILE_ARGS()) {
	OP_ASSERT(!unsummable);
	OP_ASSERT(WindingType::uninitialized == sum.type);
	sum.w = w.copyData();
	OP_ASSERT(sum.w.size);
	sum.type = WindingType::copy;
	OP_DEBUG_CODE(sum.debugType = DebugWindingType::sum);
    OP_LINE_FILE_SET(debugSetSum);
}

OpEdge* OpEdge::updateLastEdge() {
	OpEdge* linkStart = advanceToEnd(EdgeMatch::start);
	OpEdge* linkEnd = advanceToEnd(EdgeMatch::end);
	if (linkStart->lastEdge != linkEnd)
		linkStart->setLast(linkStart, linkEnd, InOutput::no);
	return linkEnd;
}

OpPtT OpEdge::whichSect(EdgeMatch match) const {
    return match == (EdgeMatch::none == which() ? EdgeMatch::start : which())
            ? startPtT() : endPtT();
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
#if 0 && OP_DEBUG // triggered by testQuads11267619, so it is necessary 11/11/24
	if (priorEdge || nextEdge || lastEdge || EdgeMatch::start != whichEnd_impl)
		OP_ASSERT(0);
#endif
	priorEdge = nullptr;
	nextEdge = nullptr;
	clearLast(/* InOutput::no */);
	setWhich(EdgeMatch::start);  // !!! should this set to none?
}

#if 0
bool OpEdge::unsectableSeen(EdgeMatch match) const {
	for (const EdgePal& pal : pals) {
		if (pal.reversed == (EdgeMatch::end == match) ? pal.edge->startSeen : pal.edge->endSeen)
			return true;
	}
	return false;
}
#endif

#if 0
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
#endif

bool OpEdgeStorage::containsPts(OpPoint start, OpPoint end) const {
	for (int index = 0; index < used; index++) {
		const OpEdge* test = &storage[index];
		if (test->startPt() == start && test->endPt() == end)
			return true;
	}
	if (!next)
		return false;
	return next->containsPts(start, end);
}

bool OpEdgeStorage::contains(int ccUnsectableID) const {
	for (int index = 0; index < used; index++) {
		const OpEdge* test = &storage[index];
		if (test->ccUnsectID == ccUnsectableID)
			return true;
	}
	if (!next)
		return false;
	return next->contains(ccUnsectableID);
}

#if OP_DEBUG_DUMP
void OpEdgeStorage::debugRelease() {
	for (int index = 0; index < used; ++index) {
		OpEdge& edge = storage[index];
		edge.debugReleased = true;
	}
	if (!next)
		return;
	return next->debugRelease();
}
#endif

void OpEdgeStorage::reuse() {
	OP_ASSERT(0);  // !!! not plumbed in, yet
#if OP_DEBUG_VALIDATE
	for (int index = 0; index < used; ++index)
		storage[index].~OpEdge();
#endif
	used = 0;
	next = nullptr;
}
