// (c) 2023, Cary Clark cclark2@gmail.com
#include "OpCurveCurve.h"
#include "OpDebugRecord.h"
#include "OpSegment.h"
#include "OpSegments.h"
#include "OpWinder.h"

static bool compareXBox(const OpSegment* s1, const OpSegment* s2) {
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
}

// may need to adjust values in opp if end is nearly equal to seg
std::vector<OpIntersection*> OpSegments::AddEndMatches(OpSegment* seg, OpSegment* opp) {
	OP_DEBUG_CONTEXT();
    std::vector<OpIntersection*> result;
	auto add = [&result](OpSegment* seg, OpSegment* opp, OpPoint pt, float segT, float oppT   
			OP_LINE_FILE_ARGS()) {
		if (seg->sects.contains(OpPtT { pt, segT }, opp) 
				|| opp->sects.contains(OpPtT { pt, oppT }, seg))
			return;
		OpIntersection* sect = seg->addSegSect(OpPtT { pt, segT }, opp  OP_LINE_FILE_CARGS());
		OpIntersection* oSect = opp->addSegSect(OpPtT { pt, oppT }, seg  OP_LINE_FILE_CARGS());
		sect->pair(oSect);
        result.push_back(sect);
	};
	auto checkEnds = [add, seg, opp](OpPoint oppPt, float oppT  OP_LINE_FILE_ARGS()) {
		OpPtT segPtT = seg->alignToEnd(oppPt);
		if (!OpMath::IsNaN(segPtT.t) && (0 == oppT || 1 == oppT)) {
			oppPt = seg->mergePoints(segPtT, opp, { oppPt, oppT });
			add(seg, opp, oppPt, segPtT.t, oppT  OP_LINE_FILE_CARGS());
		}
		OP_ASSERT(opp->c.firstPt() != opp->c.lastPt() || opp->willDisable || opp->disabled);
		return segPtT.t;
	};
	float startSegT = checkEnds(opp->c.firstPt(), 0  OP_LINE_FILE_PARGS());
	float endSegT = checkEnds(opp->c.lastPt(), 1  OP_LINE_FILE_PARGS());
	auto checkOpp = [add, seg, opp](OpPoint segPt, float segT  OP_LINE_FILE_ARGS()) {
		OpPtT oppPtT = opp->matchEnd(segPt);
		if (!OpMath::IsNaN(oppPtT.t)) {
			if (0 == oppPtT.t || 1 == oppPtT.t)
				segPt = seg->mergePoints({ segPt, segT }, opp, oppPtT);
			add(seg, opp, segPt, segT, oppPtT.t  OP_LINE_FILE_CARGS());
		}
		return oppPtT.t;
	};
	float startOppT = OpNaN;
	float endOppT = OpNaN;
	if (0 != startSegT && 0 != endSegT) 
		startOppT = checkOpp(seg->c.firstPt(), 0  OP_LINE_FILE_PARGS());  // see if start pt is on opp curve
	if (1 != startSegT && 1 != endSegT) 
		endOppT = checkOpp(seg->c.lastPt(), 1  OP_LINE_FILE_PARGS());
	auto checkSeg = [add, seg, opp](OpPoint oppPt, float oppT  OP_LINE_FILE_ARGS()) {
		OpPtT segPtT = seg->matchEnd(oppPt);
		if (OpMath::IsNaN(segPtT.t)) 
            return;
		if (0 == segPtT.t || 1 == segPtT.t)
			oppPt = opp->mergePoints({ oppPt, oppT }, seg, segPtT);
		add(seg, opp, oppPt, segPtT.t, oppT  OP_LINE_FILE_CARGS());
	};
//    OpBreak2(seg, opp, 5, 11);
	if (OpMath::IsNaN(startSegT) && 0 != startOppT && 0 != endOppT)
		checkSeg(opp->c.firstPt(), 0  OP_LINE_FILE_PARGS());
	if (OpMath::IsNaN(endSegT) && 1 != startOppT && 1 != endOppT)
		checkSeg(opp->c.lastPt(), 1  OP_LINE_FILE_PARGS());
    return result;
}

// somewhat different from winder's edge based version, probably for no reason
void OpSegments::AddLineCurveIntersection(OpSegment* opp, OpSegment* seg) {
	OP_DEBUG_CONTEXT();
	OP_ASSERT(opp != seg);
	OP_ASSERT(seg->c.debugIsLine());
	OpRoots oppRoots = seg->c.lineIntersection(opp->c);
	OP_DEBUG_CODE(MatchReverse matchRev = opp->matchEnds(seg));
	// if line and curve share end point, pass hint that root finder can call
	// reduced form that assumes one root is zero or one.
#if 0  // code coverage did not detect any of these cases
	if (septs.fail == RootFail::rawIntersectFailed) {
		// binary search on opp t-range to find where vert crosses zero
		OpCurve rotated = opp->c.toVertical(edgePts, matchRev.match);
		septs.roots[0] = rotated.tZeroX(0, 1);
		septs.count = 1;
	}
	if (opp->c.isLine() && MatchEnds::both == matchRev.match) {
		seg->moveWinding(opp, matchRev.reversed);
		return;
	}
#else
	if (2 == oppRoots.count() && opp->c.isLine()) {
		OpWinder::CoincidentCheck(seg, opp);
		return;
	}
	OP_ASSERT(oppRoots.fail != RootFail::rawIntersectFailed);
	OP_ASSERT(!opp->c.isLine() || MatchEnds::both != matchRev.match);
#endif
	std::vector<OpPtT> oppPtTs;
	std::vector<OpPtT> edgePtTs;
	size_t segSects = seg->sects.i.size();
	size_t oppSects = opp->sects.i.size();
	for (float oppT : oppRoots.roots) {
#if 0
//		if (OpMath::NearlyEndT(oppT))	// if curve hits middle of line, do not ignore (loop48977)
//			continue;
		// if computed point is nearly end, ignore
		OpPoint oppPt = opp->c.ptAtT(oppT);  // !!! redundant if ray intersect is rewritten to return pt
//		again, if curve hits middle of line, do not ignore (loop48977)
//		if (oppPt.isNearly(oppT < .5 ? opp->c.firstPt() : opp->c.lastPt(), seg->threshold()))
//			continue;
		if (oppPt.isNearly(edgePts.pts[0], seg->threshold()))
			continue;
		if (oppPt.isNearly(edgePts.pts[1], seg->threshold()))
			continue;
		float edgeT = seg->findValidT(0, 1, oppPt);
		oppPt = seg->c.ptAtT(edgeT);  // use line instead of curve to keep points on line
		OpPtT oppPtT { oppPt, oppT };
		if (OpMath::IsNaN(edgeT))
			continue;
		if (OpMath::NearlyEndT(edgeT))
			continue;
#elif 0
		float edgeT;
		PathOpsV0Lib::ContextCallbacks& cb = seg->contour->context->contextCallbacks;
		float margin = cb.maxMarginFuncPtr ? cb.maxMarginFuncPtr(seg->c.c) : 8.0f;
		margin *= OpEpsilon;
		OpPtT oppPtT = opp->c.lineCurve(seg->c, oppT, &edgeT, MatchEnds::both, margin);  // check ends
		if (OpMath::IsNaN(oppPtT.t))
			continue;
#else
		OpPtT oppPtT = opp->c.ptTAtT(oppT);
		float edgeT = seg->findLineT(oppPtT.pt);
		if (!(0 <= edgeT) || !(edgeT <= 1))
			continue;
#endif
		seg->ptBounds.pin(&oppPtT.pt);  // required by testLine409
		opp->ptBounds.pin(&oppPtT.pt);
		oppPtTs.push_back(oppPtT);
		edgePtTs.emplace_back(oppPtT.pt, edgeT);
		OpPtT& edgePtT = edgePtTs.back();
		for (size_t earlier = 1; earlier < oppPtTs.size(); ++earlier) {
			if (oppPtTs[earlier - 1].t == oppPtT.t)
				continue;
		}
		for (size_t earlier = 1; earlier < edgePtTs.size(); ++earlier) {
			if (edgePtTs[earlier - 1].t == edgePtT.t)
				continue;
		}
		if (seg->sects.contains(edgePtT, opp))
			continue;
		if (opp->sects.contains(oppPtT, seg))
			continue;
			// don't add sects here if coincident or unsectable will be added below --
			// i guess record this and defer until after coin/unsect has been checked
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
	if (sectE) {
		float midT = OpMath::Average(sectS->ptT.t, sectE->ptT.t);
		// distance from seg point at midT normal to opp segment
		OpPtT midPtT = seg->c.ptTAtT(midT);
		OpPtT oppPtT = seg->distance(midPtT, opp);
		float dist = (midPtT.pt - oppPtT.pt).length();
		auto endFromT = [](OpIntersection* one, OpIntersection* two, MatchEnds match) -> MatchEnds {
			return (one->ptT.t < two->ptT.t) == (MatchEnds::start == match) 
					? MatchEnds::start : MatchEnds::end;
		};
		if (dist <= seg->threshold().length()) {
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
			std::array<CoinEnd, 4> ends {{{ seg, opp, sectS->ptT, OpVector() }, 
				{ seg, opp, sectE->ptT, OpVector() },
				{ opp, seg, sectS->opp->ptT, OpVector() }, 
				{ opp, seg, sectE->opp->ptT, OpVector() }}};
			OpWinder::CoincidentCheck(ends, nullptr, nullptr);
		} else if (dist < seg->threshold().length() * 8) { // !!! who knows what this const should be?
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
	}
	return;
}

void OpSegments::AddEndMatches(OpContour* contour, OpContour* oContour) {
	bool same = contour == oContour;
	for (size_t iDex = 0; iDex < contour->sorted.size(); ++iDex) {
		OpSegment* seg = contour->sorted[iDex];
		if (seg->disabled)
			continue;
		for (size_t oDex = same ? iDex + 1 : 0; oDex < oContour->sorted.size(); ++oDex) {
			OpSegment* opp = oContour->sorted[oDex];
			if (seg->ptBounds.right < opp->ptBounds.left)
				break;
            (void) AddEndMatches(seg, opp);  // ignore return result
		}
	}
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
			if (seg->ptBounds == opp->ptBounds && !findCoincidence(seg, opp))
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

IntersectResult OpSegments::LineCoincidence(OpSegment* seg, OpSegment* opp) {
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
		if (!seg->ptBounds.intersects(opp->ptBounds))  // close bounds intersect, ptBounds do not
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

// note: ends have already been matched for consecutive segments
FoundIntersections OpSegments::findIntersections() {
	OP_DEBUG_CONTEXT();
    PathOpsV0Lib::WindingIntersect windingSect = context.windingCallbacks.windingIntersectFuncPtr;
	for (OpContour* oContour: context.contours) {
		if (oContour->disabled)
			continue;
		for (OpContour* member : oContour->members()) {
			if (member->id > oContour->id)
				break;
            if (windingSect && !(*windingSect)((ContextPtr) &context, oContour->winding(), 
                    member->winding())) {
                AddEndMatches(oContour, member);  // if both are frame, check endpoints only
                continue;
            }
			findIntersection(oContour, member);
		}
	}
	return found;
}

void OpSegments::findIntersection(OpContour* contour, OpContour* oContour) {
	bool same = contour == oContour;
	for (size_t iDex = 0; iDex < contour->sorted.size(); ++iDex) {
		OpSegment* seg = contour->sorted[iDex];
// !!! why ignore disabled? (add example, reasoning
		if (seg->disabled)
			continue;
		for (size_t oDex = same ? iDex + 1 : 0; oDex < oContour->sorted.size(); ++oDex) {
			OpSegment* opp = oContour->sorted[oDex];
			if (opp->disabled)
				continue;
			if (seg->ptBounds.right < opp->ptBounds.left)
				break;
			// loop134071: seg 8 collapses. With pt bounds instead of 'close', seg 7 + 9 don't touch
			if (!seg->ptBounds.intersectsThreshold(opp->ptBounds, contour->context->threshold()))
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
	std::vector<OpIntersection*> matchingSects = AddEndMatches(seg, opp);
	if (seg->willDisable || opp->willDisable)
		return true;
	if (seg->isSmall()) {
		seg->willDisable = true;
		return true;
	}
	if (opp->isSmall()) {
		opp->willDisable = true;
		return true;
	}
	// for line-curve intersection we can directly intersect
	if (seg->c.isLine()) {
		if (opp->c.isLine()) {
			if (seg->disabled)
				return false;
			if (opp->disabled)
				return false;
			IntersectResult lineCoin = LineCoincidence(seg, opp);
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
		AddLineCurveIntersection(opp, seg);
		return true;
	} else if (opp->c.isLine()) {
		AddLineCurveIntersection(seg, opp);
		return true;
	}
	// if the bounds only share a corner, there's nothing more to do
	bool sharesHorizontal = seg->ptBounds.right == opp->ptBounds.left
			|| seg->ptBounds.left == opp->ptBounds.right;
	bool sharesVertical = seg->ptBounds.bottom == opp->ptBounds.top
			|| seg->ptBounds.top == opp->ptBounds.bottom;
	if (sharesHorizontal && sharesVertical)
		return true;
	// look for curve curve intersections (skip coincidence already found)
	OpCurveCurve cc(seg, opp, matchingSects);
	if (cc.boundedEdgeFailed) {
		found = FoundIntersections::fail;
		return false;
	}
	if (!cc.overlap)
		return true;
	SectFound ccResult = cc.divideAndConquer();
#if OP_DEBUG_DUMP
    if (2 == opp->id && 7 == seg->id)
        dmpFile();
    dmp(cc.context->ccStorage);
#endif
	OP_ASSERT(OP_DEBUG_FAST_TEST || cc.debugShowImage());
	// search runs for small opp distances; turn found into limits
	SectFound limitsResult = cc.runsToLimits();
	if (SectFound::add == limitsResult)
		ccResult = limitsResult;
	if (SectFound::add == ccResult || cc.limits.size())
		cc.findUnsectable();
//    OP_ASSERT(cc.limits.size() < 4);
	cc.context->release(cc.context->ccStorage);
	cc.context->ccStorage = nullptr;
	OP_DEBUG_CONTEXT();
	return true;
}
