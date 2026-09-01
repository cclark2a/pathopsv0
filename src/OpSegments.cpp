// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpCurveCurve.h"
#include "OpSegment.h"
#include "OpSegments.h"
#include "OpWinder.h"

// used to sort segments in contour; use original data supplied by caller
static bool compareXBox(const OpSegment* s1, const OpSegment* s2) {
	OpRect r1 = s1->c.callerBounds();
	OpRect r2 = s2->c.callerBounds();
	if (r1.left < r2.left)
		return true;
	if (r1.left > r2.left)
		return false;
	if (r1.left == r2.left && r1.right < r2.right)
		return true;
	if (r1.left == r2.left && r1.right > r2.right)
		return false;
	return s1->id < s2->id;  // tie breaker
}

OpSegments::OpSegments(OpContext& c) 
	: context(c)
	, found(FoundIntersections::yes) {
	OP_DEBUG_CODE(debugFailSegID = 0);
	OP_DEBUG_CODE(debugFailOppID = 0);
}

// !!! maybe caller should be able to choose between sort in X / sort in Y / don't sort?
void OpSegments::initInX() {
	for (auto contour : context.contours) {
		for (auto& segment : contour->segments) {
			contour->sorted.push_back(&segment);
		}
		std::sort(contour->sorted.begin(), contour->sorted.end(), compareXBox);
	}
	OP_DEBUG_DUMP_CODE(context.dumpFile(__func__));
}

// may need to adjust values in opp if end is nearly equal to seg
std::vector<OpIntersection*> OpSegments::addEndMatches(OpSegment* seg, OpSegment* opp) {
    std::vector<OpIntersection*> result;
	auto add = [&result](OpSegment* seg, OpSegment* opp, const OpPtT& segPtT, 
			const OpPtT& oppPtT  OP_LINE_FILE_ARGS()) {
		if (seg->sects.contains(segPtT, opp) || opp->sects.contains(oppPtT, seg))
			return;
		OpIntersection* sect = seg->addSegEnd(segPtT, opp  OP_LINE_FILE_CARGS());
		OpIntersection* oSect = opp->addSegEnd(oppPtT, seg  OP_LINE_FILE_CARGS());
		sect->pair(oSect);
        result.push_back(sect);
	};
	auto checkEnds = [add, seg, opp](const OpPtT& oppPtT  OP_LINE_FILE_ARGS()) {
		OpPtT segPtT = seg->matchEnd(oppPtT.pt);
		if (!OpMath::IsNaN(segPtT.t))
			add(seg, opp, segPtT, oppPtT  OP_LINE_FILE_CARGS());
		return segPtT.t;
	};
	// check seg and opp ends against each other
	float startSegT = checkEnds(OpPtT(opp->c.start, 0)  OP_LINE_FILE_PARGS());
	float endSegT = checkEnds(OpPtT(opp->c.end, 1)  OP_LINE_FILE_PARGS());	
	auto checkOpp = [add, seg, opp, this](const OpPtT& segPtT  OP_LINE_FILE_ARGS()) {
		PathOpsV0Lib::CurveConst closeFun = context.callbacks[opp->c.c.type].closeEndFuncPtr;
		OpVector threshold = context.threshold * (closeFun ? (*closeFun)(opp->c.c) : 2.f);
		float oppT = opp->c.closest(segPtT.pt, threshold);
		if (!OpMath::IsNaN(oppT)) {
			OpPtT oppPtT = opp->c.ptTAtT(oppT);
			add(seg, opp, segPtT, oppPtT  OP_LINE_FILE_CARGS());
		}
		return oppT;
	};
	float startOppT = OpNaN;
	float endOppT = OpNaN;
	// check if seg end touches opp curve
	if (0 != startSegT && 0 != endSegT)  
		startOppT = checkOpp(OpPtT(seg->c.start, 0)  OP_LINE_FILE_PARGS());  
	if (1 != startSegT && 1 != endSegT) 
		endOppT = checkOpp(OpPtT(seg->c.end, 1)  OP_LINE_FILE_PARGS());
	auto checkSeg = [add, seg, opp, this](const OpPtT& oppPtT  OP_LINE_FILE_ARGS()) {
		PathOpsV0Lib::CurveConst closeFun = context.callbacks[seg->c.c.type].closeEndFuncPtr;
		OpVector threshold = context.threshold * (closeFun ? (*closeFun)(seg->c.c) : 2.f);
		float segT = seg->c.closest(oppPtT.pt, threshold);
		if (OpMath::IsNaN(segT)) 
            return;
		OpPtT segPtT = seg->c.ptTAtT(segT);
		add(seg, opp, segPtT, oppPtT  OP_LINE_FILE_CARGS());
	};
	// check if opp end touches seg curve
	if (OpMath::IsNaN(startSegT) && 0 != startOppT && 0 != endOppT)
		checkSeg(OpPtT(opp->c.start, 0)  OP_LINE_FILE_PARGS());
	if (OpMath::IsNaN(endSegT) && 1 != startOppT && 1 != endOppT)
		checkSeg(OpPtT(opp->c.end, 1)  OP_LINE_FILE_PARGS());
    return result;
}

// somewhat different from winder's edge based version, probably for no reason
FoundIntersections OpSegments::addLineCurveIntersection(OpSegment* opp, OpSegment* seg,
		std::vector<OpIntersection*>& matchingSects) {
	OP_ASSERT(opp != seg);
	OP_ASSERT(seg->c.debugIsLine());
	OpCurveCurve cc(seg, opp, matchingSects, ForCurveLineSect::dummy);
	if (cc.overflowFail)
			return FoundIntersections::fail;
	OpRoots oppRoots = seg->c.lineIntersection(opp->c);
	OP_DEBUG_CODE(MatchReverse matchRev = opp->matchEnds(seg));
	if (2 == oppRoots.count() && opp->c.isLine()) {
	#if DEFER_COIN_CHECK
		deferredCoinEnds.push_back({seg, opp});
	#else
		OpWinder::CoincidentCheck(seg, opp);
	#endif
		return FoundIntersections::yes;
	}
	OP_ASSERT(oppRoots.fail != RootFail::rawIntersectFailed);
	OP_ASSERT(!opp->c.isLine() || MatchEnds::both != matchRev.match);
	size_t segSects = seg->sects.i.size();
	size_t oppSects = opp->sects.i.size();
	for (float oppT : oppRoots.roots) {
		OpPtT oppPtT = opp->c.ptTAtT(oppT);  // point at curve's t
		float edgeT = seg->findLineT(oppPtT.pt);
		if (!(0 <= edgeT) || !(edgeT <= 1))
			continue;
		OpPtT edgePtT = seg->c.ptTAtT(edgeT);
			// don't add sects here if coincident or unsectable will be added below --
			// i guess record this and defer until after coin/unsect has been checked
		if (cc.limits.alreadyIn(edgePtT, oppPtT))
			continue;
    #if CHECK_SNIP
		bool skipIt = false;
        // !!! because crossover is inside snip, line / line intersection is missed
        //     later in winding, ray goes between limit and snip
        //     keep computed snip in intersection so that ray can skip it?
		for (const SnipPtTs& snip : cc.limits.snips) {
			if ((snip.segCut.lo.t <= edgePtT.t && edgePtT.t <= snip.segCut.hi.t) 
					|| (snip.oppCut.lo.t <= oppPtT.t && oppPtT.t <= snip.oppCut.hi.t)) {
                int snipID = seg->contour->context->nextID();
                snip.sect->setSnip(snip.segCut, snipID);
                snip.sect->opp->setSnip(snip.oppCut, snipID);
				skipIt = true;
				break;
			}
		}
		if (skipIt)
			continue;
    #endif
		if (seg->sects.contains(edgePtT, opp))
			continue;
		if (opp->sects.contains(oppPtT, seg))
			continue;
		OpIntersection* sect = seg->addSegBase(edgePtT  OP_LINE_FILE_PARAMS(opp));
		OpIntersection* oSect = opp->addSegBase(oppPtT  OP_LINE_FILE_PARAMS(seg));
		sect->pair(oSect);
	}
	// if pair share two intersections, and mid t is close, mark intersections as unsectable
	OpIntersection* sectS = nullptr;
	OpIntersection* sectE = nullptr;
	size_t index = 0;
	while (index < seg->sects.i.size()) {
		OpIntersection* test = seg->sects.i[index];
		if (test->opp->segment == opp) {
			if (sectS) {
				sectE = test;
				break;
			}
			sectS = test;
		}
		++index;
	}
	if (!sectE)
		return FoundIntersections::yes;
	float midT = OpMath::Average(sectS->ptT.t, sectE->ptT.t);
	// distance from seg point at midT normal to opp segment
	OpPtT midPtT = seg->c.ptTAtT(midT);
	OpPtT oppPtT = seg->distance(midPtT, opp);
	float dist = (midPtT.pt - oppPtT.pt).length();
	auto endFromT = [](OpIntersection* one, OpIntersection* two, MatchEnds match) -> MatchEnds {
		return (one->ptT.t < two->ptT.t) == (MatchEnds::start == match) 
				? MatchEnds::start : MatchEnds::end;
	};
	float threshLen = seg->threshold().length();
	if (dist <= threshLen) {
		auto removeBetweeners = [](OpSegment* seg, size_t segSects, 
				OpIntersection* start, OpIntersection* end) {
			if (start->ptT.t > end->ptT.t)
				std::swap(start, end);
			size_t index = seg->sects.i.size();
			const OpSegment* opp = start->opp->segment;
			OP_ASSERT(opp == end->opp->segment);
			while (index > segSects) {
				OpIntersection* test = seg->sects.i[--index];
				if (test->opp->segment == opp 
						&& start->ptT.t < test->ptT.t && test->ptT.t < end->ptT.t)
					seg->sects.i.erase(seg->sects.i.begin() + index);
			}
		};
		removeBetweeners(seg, segSects, sectS, sectE);
		removeBetweeners(opp, oppSects, sectS->opp, sectE->opp);
	#if !DEFER_COIN_CHECK
		std::array<CoinEnd, 4> ends {{{ seg, opp, sectS->ptT, OpVector() }, 
			{ seg, opp, sectE->ptT, OpVector() },
			{ opp, seg, sectS->opp->ptT, OpVector() }, 
			{ opp, seg, sectE->opp->ptT, OpVector() }}};
		OpWinder::CoincidentCheck(ends, nullptr, nullptr);
	#else
		deferredCoinSects.push_back( { sectS, sectE } );
	#endif
		return FoundIntersections::yes;
	} 
	PathOpsV0Lib::ContextCallbacks& cb = seg->contour->context->contextCallbacks;
	float unsectDist = cb.maxUnsectDistFuncPtr ? cb.maxUnsectDistFuncPtr(seg->c.c) : 8.0f;
	if (dist < seg->threshold().length() * unsectDist) {
		int usectID = seg->nextID();
		seg->addUnsectable(sectS->ptT, usectID, endFromT(sectS, sectE, MatchEnds::start), opp
				OP_LINE_FILE_PARGS());
		seg->addUnsectable(sectE->ptT, usectID, endFromT(sectS, sectE, MatchEnds::end), opp
				OP_LINE_FILE_PARGS());
		OpIntersection* oStart = sectS->opp;
		OpIntersection* oEnd = sectE->opp;
		bool flipped = oStart->ptT.t > oEnd->ptT.t;
		if (flipped)
			usectID = -usectID;
		opp->addUnsectable(oStart->ptT, usectID, endFromT(oStart, oEnd, MatchEnds::start), seg
				OP_LINE_FILE_PARGS());
		opp->addUnsectable(oEnd->ptT, usectID, endFromT(oStart, oEnd, MatchEnds::end), seg
				OP_LINE_FILE_PARGS());
	}
	return FoundIntersections::yes;
}

void OpSegments::addEndMatches(OpContour* contour, OpContour* oContour) {
	bool same = contour == oContour;
	for (size_t iDex = 0; iDex < contour->sorted.size(); ++iDex) {
		OpSegment* seg = contour->sorted[iDex];
		if (seg->disabled)
			continue;
		for (size_t oDex = same ? iDex + 1 : 0; oDex < oContour->sorted.size(); ++oDex) {
			OpSegment* opp = oContour->sorted[oDex];
			if (seg->c.aliasBounds().right < opp->c.aliasBounds().left)
				break;
            (void) addEndMatches(seg, opp);  // ignore return result
		}
	}
}

void OpSegments::checkCoins() {
	for (const DeferredCoinSect& deferred : deferredCoinSects) {
		OpSegment* seg = deferred.segStart->segment;
		OpSegment* opp = deferred.segStart->opp->segment;
        if (seg->disabled || opp->disabled)
            continue;
		OpIntersection* sectS = deferred.segStart;
		OpIntersection* sectE = deferred.segEnd;
		std::array<CoinEnd, 4> ends {{{ seg, opp, sectS->ptT, OpVector() }, 
			{ seg, opp, sectE->ptT, OpVector() },
			{ opp, seg, sectS->opp->ptT, OpVector() }, 
			{ opp, seg, sectE->opp->ptT, OpVector() }}};
		OpWinder::CoincidentCheck(ends, nullptr, nullptr);
	}
	for (const DeferredCoinEnd& defEnd : deferredCoinEnds) {
        if (defEnd.seg->disabled || defEnd.opp->disabled)
            continue;
		OpWinder::CoincidentCheck(defEnd.seg, defEnd.opp);
	}
	OP_DEBUG_DUMP_CODE(context.dumpFile(__func__));
}

void OpSegments::findCoincidences() {
	for (OpContour* oContour: context.contours) {
		if (oContour->disabled)
			continue;
		for (OpContour* member : oContour->members()) {
			if (oContour->id > member->id)
				continue;
			findCoincidence(oContour, member);
		}
	}
}

void OpSegments::findCoincidence(OpContour* contour, OpContour* oContour) {
	bool same = contour == oContour;
	for (size_t iDex = 0; iDex < contour->segments.size(); ++iDex) {
		OpSegment* seg = &contour->segments[iDex];
		if (seg->disabled)
			continue;
		for (size_t oDex = same ? iDex + 1 : 0; oDex < oContour->segments.size(); ++oDex) {
			OpSegment* opp = &oContour->segments[oDex];
			if (opp->disabled)
				continue;
			if (seg->c.aliasBounds() == opp->c.aliasBounds() && !findCoincidence(seg, opp))
				break;
		}
	}
}

// take care of totally coincident segments
bool OpSegments::findCoincidence(OpSegment* seg, OpSegment* opp) {
	MatchReverse mr = seg->matchEnds(opp);
	PathOpsV0Lib::Curve& curve = seg->c.c;
	if (MatchEnds::both != mr.match || curve.type != opp->c.c.type)
		return true;
	// if control points and weight match, treat as coincident: transfer winding
	PathOpsV0Lib::CurvesEqual funcPtr = context.callback(curve.type).curvesEqualFuncPtr;
	if (funcPtr && !(*funcPtr)(curve, opp->c.c))
		return true;
	return seg->moveWinding(opp, mr.reversed);
}

#if !DEFER_COIN_CHECK
IntersectResult OpSegments::lineCoincidence(OpSegment* seg, OpSegment* opp) {
	OP_ASSERT(seg->c.debugIsLine());
	OP_ASSERT(!seg->disabled);
	// special case pairs that exactly match start and end
	MatchReverse ends = seg->matchEnds(opp);
	if (MatchEnds::both == ends.match) {
		seg->moveWinding(opp, ends.reversed);
		return IntersectResult::yes;
	}
	OpVector tangent = seg->c.tangent(0);
	if (!tangent.dx || !tangent.dy) {
		OP_ASSERT(tangent.dx || tangent.dy);
		OP_ASSERT(opp->c.debugIsLine());
		OP_ASSERT(!opp->disabled);
		OpVector oTangent = opp->c.tangent(0);
		if (oTangent.dx && oTangent.dy)
			return IntersectResult::no;
		OP_ASSERT(oTangent.dx || oTangent.dy);
		if (!tangent.dot(oTangent))  // if at right angles, skip
			return IntersectResult::no;
		if (!seg->c.callerBounds().intersects(opp->c.callerBounds()))  // alias bounds intersect, caller bounds do not
			return IntersectResult::no;
		return OpWinder::CoincidentCheck(seg, opp);
	}
	LinePts oppLine = opp->c.linePts();
	OpCurve vertSeg = seg->c.toVertical(oppLine, ends.match);
	if (!vertSeg.isFinite()) {
		seg->contour->context->setError(PathOpsV0Lib::ContextError::toVertical  
				OP_DEBUG_PARAMS(seg->id));
		return IntersectResult::fail;
	}
	if (!vertSeg.isVertical())
		return IntersectResult::no;
	LinePts segLine = seg->c.linePts();
	OpCurve vertOpp = opp->c.toVertical(segLine, ends.flipped());
	if (!vertOpp.isFinite()) {
		seg->contour->context->setError(PathOpsV0Lib::ContextError::toVertical  
				OP_DEBUG_PARAMS(opp->id));
		return IntersectResult::fail;
	}
	if (!vertOpp.isVertical())
		return IntersectResult::no;
	LinePts edgePts;
	if (fabsf(vertSeg.firstPt().x - vertOpp.firstPt().x) > OpEpsilon)
		return IntersectResult::no;
	return OpWinder::CoincidentCheck(seg, opp);
}
#endif

// note: ends have already been matched for consecutive segments
FoundIntersections OpSegments::findIntersections() {
    PathOpsV0Lib::WindingIntersect windingSect = context.windingCallbacks.windingIntersectFuncPtr;
	for (OpContour* oContour: context.contours) {
		if (oContour->disabled)
			continue;
		for (OpContour* member : oContour->members()) {
			if (member->id > oContour->id)
				break;
            if (windingSect && !(*windingSect)(oContour->winding(), member->winding())) {
                addEndMatches(oContour, member);  // if both are frame, check endpoints only
                continue;
            }
			findIntersection(oContour, member);
		}
	}
	OP_DEBUG_DUMP_CODE(context.dumpFile(__func__));
	return found;
}

void OpSegments::findIntersection(OpContour* contour, OpContour* oContour) {
	bool same = contour == oContour;
	for (size_t iDex = 0; iDex < contour->sorted.size(); ++iDex) {
		OpSegment* seg = contour->sorted[iDex];
		if (seg->disabled)
			continue;
		for (size_t oDex = same ? iDex + 1 : 0; oDex < oContour->sorted.size(); ++oDex) {
			OpSegment* opp = oContour->sorted[oDex];
			if (opp->disabled)
				continue;
			if (!seg->c.closeBounds().intersects(opp->c.closeBounds()))
				continue;
            bool result = findIntersection(seg, opp);
			if (!result)
				break;
		}
	}
}

bool OpSegments::findIntersection(OpSegment* seg, OpSegment* opp) {
	// comparisons below need to be 'nearly' since adjusting opp may make sort incorrect
	// or, exact compare may miss nearly equal seg/opp pairs
	// set both to lines if they are linear before using them in t calculations
	(void) seg->c.isLine();
	(void) opp->c.isLine();
	std::vector<OpIntersection*> matchingSects = addEndMatches(seg, opp);
	// if the bounds only share a corner, there's nothing more to do
	// !!! this could also work by taking the tangents and looking for a negative cross product
//	if (seg->c.isSmall || opp->c.isSmall)  // small (disabled) just adds end matches
//		return true;
	OpRect segBounds = seg->c.aliasBounds();
	OpRect oppBounds = opp->c.aliasBounds();
	bool sharesHorizontal = segBounds.right == oppBounds.left || segBounds.left == oppBounds.right;
	bool sharesVertical = segBounds.bottom == oppBounds.top || segBounds.top == oppBounds.bottom;
	if (sharesHorizontal && sharesVertical)
		return true;  // !seg->disabled && !opp->disabled;
	// for line-curve intersection we can directly intersect
	if (seg->c.isLine()) {
#if !DEFER_COIN_CHECK
		if (opp->c.isLine()) {
			if (seg->disabled)
				return false;
			if (opp->disabled)
				return false;
			IntersectResult lineCoin = lineCoincidence(seg, opp);
			if (seg->disabled)
				return false;
			if (opp->disabled)
				return false;
			if (IntersectResult::fail == lineCoin) {
				OP_DEBUG_CODE(debugFailSegID = seg->id);
				OP_DEBUG_CODE(debugFailOppID = opp->id);
				found = FoundIntersections::fail;
				return false;
			}
			if (IntersectResult::coincident == lineCoin)
				return true;
		}
#endif
		found = addLineCurveIntersection(opp, seg, matchingSects);
		return true;
	} else if (opp->c.isLine()) {
		SwapEndMatches(matchingSects);
		addLineCurveIntersection(seg, opp, matchingSects);
		return true;
	}
	// look for curve curve intersections (skip coincidence already found)
	OpCurveCurve cc(seg, opp, matchingSects);
	if (cc.boundedEdgeFailed || cc.overflowFail) {
		found = FoundIntersections::fail;
		OP_DEBUG_CODE(cc.context->debugCurveCurve = nullptr);
		return false;
	}
	if (!cc.overlap) {
		OP_DEBUG_CODE(cc.context->debugCurveCurve = nullptr);
		return true;
	}
	SectFound ccResult = cc.divideAndConquer();
#if OP_DEBUG_DUMP
	OP_ASSERT(cc.debugBreak(CcBreak::atEnd));
#endif
	// search runs for small opp distances; turn found into limits
	SectFound limitsResult = cc.runsToLimits();
	if (SectFound::add == limitsResult)
		ccResult = limitsResult;
	if (SectFound::add == ccResult || cc.limits.size())
		cc.findUnsectable();
//    OP_ASSERT(cc.limits.size() < 4);
	cc.context->release(cc.context->ccStorage);
	cc.context->ccStorage = nullptr;
	OP_DEBUG_CODE(cc.context->debugCurveCurve = nullptr);
	return true;
}

void OpSegments::SwapEndMatches(std::vector<OpIntersection*>& matches) {
	for (OpIntersection*& match : matches) {
		match = match->opp;
	}
}